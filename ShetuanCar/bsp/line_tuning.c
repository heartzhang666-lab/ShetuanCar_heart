/**
 * @file    line_tuning.c
 * @brief   通过 C30D 蓝牙插座设置循迹 P、D 并回显参数。
 *
 * 文件结构：
 *   1. LineTuning_Init()       — 启动 USART2 单字节中断接收
 *   2. HAL_UART_*Callback()    — 收字节入队与串口错误恢复
 *   3. line_tuning_handle()    — 验证命令、应用参数与标定参考值
 *   4. LineTuning_Task()       — 在主循环拼接完整命令行
 *   5. LineTuning_GetGains()   — 提供当前生效的 P、D
 */

#include "line_tuning.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gray_line.h"
#include "usart.h"

// 环形缓冲长度取 2 的幂，允许用位掩码回绕。行长度限制为 31 个 ASCII 字符。
#define LINE_RX_CAPACITY 128U
#define LINE_COMMAND_CAPACITY 32U
#define LINE_PARSE_BUDGET 32U
// 调大 P 会强化回线，调大 D 会强化对偏差变化的抑制并放大噪声。上限限制误触时的突变。
#define LINE_KP_MAX 8.0
#define LINE_KD_MAX 12.0

// USART2 中断写入队列及错误标志，主循环读取并维护读索引。回复缓冲由 TX 中断读到发送完成。
static uint8_t rx_byte;
static volatile uint8_t rx_queue[LINE_RX_CAPACITY];
static volatile uint8_t rx_head=0U;
static volatile uint8_t rx_tail=0U;
static volatile uint8_t rx_error=0U;
static uint8_t line_size=0U;
static uint8_t discard_line=0U;
static char command[LINE_COMMAND_CAPACITY];
static char response[64];

// P/D 在主循环中跨帧保存，修改在下一次计算轮速目标时生效。
static LineTuningGains gains={0.8f,0.0f};

// ================================================================
// 1. C30D 蓝牙串口收发
//
//    PD5/USART2_TX 向蓝牙模块发送，PD6/USART2_RX 接收模块输出，复用 AF7，
//    沿用原厂 9600 bit/s、8 数据位、无校验、1 停止位。中断仅搬运字节，
//    解析与 ADC 标定均在主循环执行，TIM6 的 10 ms 控制可随时抢占。
// ================================================================
void LineTuning_Init(void)
{
  if(HAL_UART_Receive_IT(&huart2,&rx_byte,1U)!=HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t next;

  if(huart!=&huart2)  return;
  next=(uint8_t)((rx_head+1U)&(LINE_RX_CAPACITY-1U));
  if(next!=rx_tail)
  {
    rx_queue[rx_head]=rx_byte;
    rx_head=next;
  }
  else  rx_error=1U;

  if(HAL_UART_Receive_IT(&huart2,&rx_byte,1U)!=HAL_OK)  rx_error=1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart!=&huart2)  return;
  rx_error=1U;

  // 帧错误或溢出作废当前命令。HAL 已结束接收时重新挂上一字节缓冲。
  if(huart2.RxState==HAL_UART_STATE_READY)  (void)HAL_UART_Receive_IT(&huart2,&rx_byte,1U);
}

// ================================================================
// 2. 文本指令处理
//
//    P=0.8、D=0.4 或 GET 后加回车或换行。停车时 WHITE、BLACK 分别采集
//    覆盖八个探头的均匀白底和黑底。只接受完整合法行，错误不修改当前参数。
// ================================================================
static void line_tuning_handle(uint8_t running)
{
  char *end;
  const char *reply="ERR CMD\r\n";

  if(strcmp(command,"GET")==0)
  {
    (void)snprintf(response,sizeof(response),"OK P=%.3f D=%.3f\r\n",(double)gains.kp,(double)gains.kd);
    reply=response;
  }
  else if((command[0]=='P'||command[0]=='D')&&command[1]=='=')
  {
    const char *number=&command[2];
    double value=strtod(number,&end);
    double max=command[0]=='P'?LINE_KP_MAX:LINE_KD_MAX;

    // 比较条件同时拒绝非数字、尾随文本、NaN 和无穷大。
    if(end!=number&&*end=='\0'&&value>=0.0&&value<=max)
    {
      if(command[0]=='P')  gains.kp=(float)value;
      else  gains.kd=(float)value;
      (void)snprintf(response,sizeof(response),"OK P=%.3f D=%.3f\r\n",(double)gains.kp,(double)gains.kd);
      reply=response;
    }
    else  reply="ERR RANGE\r\n";
  }
  else if(strcmp(command,"WHITE")==0||strcmp(command,"BLACK")==0)
  {
    if(running!=0U)  reply="ERR STOP FIRST\r\n";
    else if(GrayLine_CaptureReference(command[0]=='B')!=0U)  reply=command[0]=='B'?"OK BLACK\r\n":"OK WHITE\r\n";
    else  reply="ERR ADC/CONTRAST\r\n";
  }

  if(reply!=response)  (void)snprintf(response,sizeof(response),"%s",reply);
  (void)HAL_UART_Transmit_IT(&huart2,(uint8_t *)response,(uint16_t)strlen(response));
}

void LineTuning_Task(uint8_t running)
{
  uint8_t count;

  if(huart2.gState!=HAL_UART_STATE_READY)  return;
  if(rx_error!=0U)
  {
    uint32_t interrupts=__get_PRIMASK();

    // 封闭清队列和错误标志的临界区，避免将丢失字节后的行误识别为有效命令。
    __disable_irq();
    rx_tail=rx_head;
    rx_error=0U;
    __set_PRIMASK(interrupts);
    line_size=0U;
    discard_line=1U;
    if(huart2.RxState==HAL_UART_STATE_READY)  (void)HAL_UART_Receive_IT(&huart2,&rx_byte,1U);
  }
  for(count=0U;count<LINE_PARSE_BUDGET&&rx_tail!=rx_head;count++)
  {
    uint8_t byte=rx_queue[rx_tail];
    rx_tail=(uint8_t)((rx_tail+1U)&(LINE_RX_CAPACITY-1U));
    if(byte=='\r'||byte=='\n')
    {
      uint8_t length=line_size;
      line_size=0U;
      if(discard_line!=0U)  discard_line=0U;
      else if(length!=0U)
      {
        command[length]='\0';
        line_tuning_handle(running);
        return;  // 本次回复使用静态缓冲，发送完成前不处理下一条。
      }
    }
    else if(discard_line==0U)
    {
      if(byte<32U||byte>126U||line_size>=LINE_COMMAND_CAPACITY-1U)  discard_line=1U;
      else  command[line_size++]=(char)byte;
    }
  }
}

LineTuningGains LineTuning_GetGains(void)
{
  return gains;
}
