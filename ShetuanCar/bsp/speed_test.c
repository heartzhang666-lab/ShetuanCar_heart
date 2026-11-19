/**
 * @file    speed_test.c
 * @brief   在台架上依次对 A 至 D 轮进行五档速度环测试。
 *
 * 文件结构：
 *   1. speed_test_draw()        — 读取中断快照并刷新 OLED
 *   2. speed_test_read_key()    — 对 USER 消抖并识别按下边沿
 *   3. SpeedTest_Init()        — 准备编码器、PWM 和 10 ms 节拍
 *   4. SpeedTest_Task()        — 管理切轮、松键等待和显示
 *   5. SpeedTest_ControlTick() — 执行固定时间采样、五档闭环和停止
 */

#include "speed_test.h"
#include <stdio.h>
#include "main.h"
#include "motor_drive.h"
#include "OLED.h"
#include "tim.h"
#include "wheel_speed.h"

// 五档单位均为计数/10 ms。18～50 附近覆盖原项目的常用区间，10 用于观察低速量化和起转。
static const uint16_t targets[]={10U,20U,30U,40U,50U};
#define SPEED_TEST_STEP_COUNT (sizeof(targets)/sizeof(targets[0]))

// 每档保持 300 个 TIM6 周期，即 3 s。延长可看清较慢的收敛，同时增加单轮连续运行时间。
#define SPEED_TEST_STEP_TICKS 300U
// 持续 500 ms 无正向反馈，且输出达到 50% 时中止。提高阈值或延长等待会推迟堵转和断线判定。
#define SPEED_TEST_FEEDBACK_TICKS 50U
#define SPEED_TEST_FEEDBACK_COMMAND (MOTOR_DRIVE_COMMAND_MAX/2U)

// USER 为 PE0 上拉低有效。增大消抖可抑制更长的跳变，松键延时则用于让操作手离开测试轮。
#define SPEED_TEST_DEBOUNCE_MS 25U
#define SPEED_TEST_SETTLE_MS 800U
// OLED 每 100 ms 刷新一次。缩短间隔使显示更即时，但增加主循环的软件串行开销。
#define SPEED_TEST_REFRESH_MS 100U

#define SPEED_TEST_STOP_REQUEST 0xFEU
#define SPEED_TEST_NO_REQUEST 0xFFU

typedef enum
{
  SPEED_TEST_READY,
  SPEED_TEST_RUNNING,
  SPEED_TEST_DONE,
  SPEED_TEST_FEEDBACK_FAULT
} SpeedTestRunState;

typedef enum
{
  SPEED_TEST_IDLE,
  SPEED_TEST_WAIT_RELEASE,
  SPEED_TEST_SETTLING
} SpeedTestInputState;

typedef struct
{
  int32_t actual;
  uint16_t target;
  uint8_t step;
  SpeedTestRunState state;
} SpeedTestReading;

static char wheel_labels[MOTOR_DRIVE_WHEEL_COUNT][17]=
{
  "A Left rear     ",
  "B Right rear    ",
  "C Right front   ",
  "D Left front    "
};

// 主循环跨周期保存选轮和按键状态，初值超出轮数，表示上电尚未发出测试命令。
static uint8_t selected_wheel=MOTOR_DRIVE_WHEEL_COUNT;
static SpeedTestInputState input_state=SPEED_TEST_IDLE;
static GPIO_PinState raw_key=GPIO_PIN_SET;
static GPIO_PinState stable_key=GPIO_PIN_SET;
static GPIO_PinState last_enable=GPIO_PIN_RESET;
static uint32_t key_since=0U;
static uint32_t release_since=0U;
static uint32_t last_refresh=0U;

// 主循环写入单字节命令，TIM6 读取后清除。读清在中断中完成，主循环不能插入这两个操作之间。
// 中断独占 PID、当前轮、档位计时和故障计数，显示端只在极短临界区读取完整快照。
static volatile uint8_t request=SPEED_TEST_NO_REQUEST;
static volatile SpeedTestReading reading={0,0U,0U,SPEED_TEST_READY};
static uint8_t active_wheel=0U;
static uint16_t step_ticks=0U;
static uint8_t bad_feedback_ticks=0U;
static WheelSpeedController controller;

// ================================================================
// 1. 主循环交互和显示
// ================================================================
static void speed_test_draw(void)
{
  SpeedTestReading snapshot;
  uint32_t interrupt_state=__get_PRIMASK();
  char line[17];
  char running[]="1/5 cnt/10ms RUN";
  char *status="USER: Next wheel";

  // 只复制快照时关中断，串行写 OLED 时保持 TIM6 中断运行。
  __disable_irq();
  snapshot=reading;
  __set_PRIMASK(interrupt_state);

  if(last_enable==GPIO_PIN_RESET)  status="Enable switch   ";
  else if(input_state==SPEED_TEST_WAIT_RELEASE)  status="Release USER    ";
  else if(input_state==SPEED_TEST_SETTLING||request<MOTOR_DRIVE_WHEEL_COUNT)  status="Starting...     ";
  else if(snapshot.state==SPEED_TEST_RUNNING)
  {
    running[0]=(char)('1'+snapshot.step);
    status=running;
  }
  else if(snapshot.state==SPEED_TEST_DONE)  status="Done cnt/10ms   ";
  else if(snapshot.state==SPEED_TEST_FEEDBACK_FAULT)  status="Feedback fault  ";

  if(last_enable==GPIO_PIN_RESET||input_state!=SPEED_TEST_IDLE||request!=SPEED_TEST_NO_REQUEST)
  {
    // 停机切轮与等待期间清除显示，用新一轮的第一次控制采样填入两个速度值。
    snapshot.target=0U;
    snapshot.actual=0;
  }
  OLED_ShowString(1,1,selected_wheel<MOTOR_DRIVE_WHEEL_COUNT?wheel_labels[selected_wheel]:"Wheel speed PI  ");
  (void)snprintf(line,sizeof(line),"TGT%12u ",(unsigned int)snapshot.target);
  OLED_ShowString(2,1,line);
  (void)snprintf(line,sizeof(line),"ACT%+12ld ",(long)snapshot.actual);
  OLED_ShowString(3,1,line);
  OLED_ShowString(4,1,status);
  last_refresh=HAL_GetTick();
}

/**
 * @brief  仅在 USER 的稳定高到低跳变时生成选轮事件。
 * @param  now 当前 HAL 节拍，单位为 ms。
 * @return 1 表示按下一次，0 表示保持、释放或尚未结束消抖。
 */
static uint8_t speed_test_read_key(uint32_t now)
{
  GPIO_PinState level=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);

  if(level!=raw_key)
  {
    raw_key=level;
    key_since=now;
  }
  if(stable_key!=raw_key&&(now-key_since)>=SPEED_TEST_DEBOUNCE_MS)
  {
    stable_key=raw_key;
    return stable_key==GPIO_PIN_RESET;
  }
  return 0U;
}

void SpeedTest_Init(void)
{
  // ------- 1. 启动编码器和零输出 PWM -------
  WheelSpeed_Init();

  // ------- 2. 建立实际按键和使能基线 -------
  last_enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);
  raw_key=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);
  stable_key=raw_key;
  key_since=HAL_GetTick();
  speed_test_draw();

  // ------- 3. 启动 TIM6 的 10 ms 更新中断 -------
  // 清除定时器初始化产生的更新标志，让第一拍也经过完整的 10 ms。
  __HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
  if(HAL_TIM_Base_Start_IT(&htim6)!=HAL_OK)
  {
    // 采样节拍不可用时进入停机错误处理，不以任意间隔调节电机。
    Error_Handler();
  }
}

void SpeedTest_Task(void)
{
  uint32_t now=HAL_GetTick();
  uint8_t pressed=speed_test_read_key(now);
  GPIO_PinState enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);

  // ------- 1. 使能变更后取消待发命令 -------
  if(enable!=last_enable)
  {
    last_enable=enable;
    stable_key=raw_key;  // 恢复使能时已按住的 USER 要先释放，之后重新按下才有效。
    input_state=SPEED_TEST_IDLE;
    request=SPEED_TEST_STOP_REQUEST;
    speed_test_draw();
    return;
  }

  // ------- 2. USER 选轮及松键后安排运行 -------
  if(enable==GPIO_PIN_SET)
  {
    if(pressed!=0U)
    {
      request=SPEED_TEST_STOP_REQUEST;
      selected_wheel++;
      if(selected_wheel>=MOTOR_DRIVE_WHEEL_COUNT)  selected_wheel=0U;
      input_state=SPEED_TEST_WAIT_RELEASE;
      speed_test_draw();
      return;
    }
    if(input_state==SPEED_TEST_WAIT_RELEASE&&stable_key==GPIO_PIN_SET)
    {
      input_state=SPEED_TEST_SETTLING;
      release_since=now;
    }
    else if(input_state==SPEED_TEST_SETTLING&&raw_key==GPIO_PIN_SET&&
            (now-release_since)>=SPEED_TEST_SETTLE_MS)
    {
      request=selected_wheel;
      input_state=SPEED_TEST_IDLE;
    }
  }

  // ------- 3. 低频刷新显示 -------
  // 按无符号差值计算间隔，允许系统毫秒节拍自然回绕。
  if((now-last_refresh)>=SPEED_TEST_REFRESH_MS)  speed_test_draw();
}

// ================================================================
// 2. TIM6 中断内的单轮控制
//
//    PD3 高电平允许运行、低电平撤去输出。采样与驱动由同一中断执行，
//    防止 OLED 通信打乱 10 ms 速度单位。切轮和每档结束均在本中断中处理。
// ================================================================
void SpeedTest_ControlTick(void)
{
  uint8_t command=request;
  request=SPEED_TEST_NO_REQUEST;

  // ------- 1. 停机优先，再处理启动命令 -------
  if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin)==GPIO_PIN_RESET||
     command==SPEED_TEST_STOP_REQUEST)
  {
    if(reading.state==SPEED_TEST_RUNNING)  MotorDrive_StopOutputs();
    reading.target=0U;
    reading.actual=0;
    reading.state=SPEED_TEST_READY;
    return;
  }
  if(command<MOTOR_DRIVE_WHEEL_COUNT)
  {
    MotorDrive_StopOutputs();
    active_wheel=command;
    WheelSpeed_Reset(active_wheel,&controller);
    step_ticks=0U;
    bad_feedback_ticks=0U;
    reading.step=0U;
    reading.target=targets[0];
    reading.actual=0;
    reading.state=SPEED_TEST_RUNNING;
    return;
  }
  if(reading.state!=SPEED_TEST_RUNNING)  return;

  // ------- 2. 整档切换，保留同一轮的 PI 历史 -------
  if(step_ticks>=SPEED_TEST_STEP_TICKS)
  {
    if(reading.step+1U>=SPEED_TEST_STEP_COUNT)
    {
      MotorDrive_StopOutputs();
      reading.state=SPEED_TEST_DONE;  // 保留末档的目标和最后一次实测，便于停机后查看。
      return;
    }
    reading.step++;
    reading.target=targets[reading.step];
    step_ticks=0U;
  }

  // ------- 3. 采样、调整驱动并检查持续无正向反馈 -------
  reading.actual=WheelSpeed_Update(active_wheel,reading.target,&controller);
  step_ticks++;
  if(reading.actual<=0&&controller.output>=(float)SPEED_TEST_FEEDBACK_COMMAND)  bad_feedback_ticks++;
  else  bad_feedback_ticks=0U;
  if(bad_feedback_ticks>=SPEED_TEST_FEEDBACK_TICKS)
  {
    // 持续输出而没有正向计数可能由堵转、相序错误或断线造成，撤去驱动等待人工检查。
    MotorDrive_StopOutputs();
    reading.state=SPEED_TEST_FEEDBACK_FAULT;
  }
}
