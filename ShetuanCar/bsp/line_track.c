/**
 * @file    line_track.c
 * @brief   俯视 X 形麦轮以左右差速循迹，支持 USER 重复启停和蓝牙 PD。
 *
 * 文件结构：
 *   1. line_track_publish()      — 原子提交本帧左右轮目标
 *   2. line_track_set_lost()     — 丢线时限速并保存计时起点
 *   3. line_track_process()      — 更新灰度帧状态与位置 PD
 *   4. line_track_draw()         — 显示黑线误差、探头位图和 P/D
 *   5. LineTrack_Init/Task()     — 上电待机、USER 启停和任务调度
 *   6. LineTrack_ControlTick()   — 固定 10 ms 四轮采样和驱动联锁
 */

#include "line_track.h"
#include <stdio.h>
#include "gray_line.h"
#include "line_tuning.h"
#include "main.h"
#include "motor_drive.h"
#include "OLED.h"
#include "tim.h"
#include "wheel_speed.h"

// 外环差分以 20 ms 一帧为单位。缩短周期可减少延迟，同时需要重新检查 D 系数。
#define TRACK_SCAN_MS 20U
#define TRACK_STALE_MS 100U
#define TRACK_LOST_STOP_MS 2000U
#define TRACK_DEBOUNCE_MS 25U
#define TRACK_OLED_MS 120U

// 下列速度是每 10 ms 的编码器计数。提高基础速度可跑得更快，也会压缩回线时间。
// 转向限幅须小于对应基础速度，使该版本左右目标均保持前进。
#define TRACK_FORWARD_SPEED 20
#define TRACK_TURN_LIMIT 6.0f
#define TRACK_SLOW_SPEED 12
#define TRACK_LOST_TURN_LIMIT 3

typedef enum
{
  TRACK_READY,
  TRACK_RUN,
  TRACK_WIDE,
  TRACK_LOST,
  TRACK_USER_STOP,
  TRACK_LOST_STOP,
  TRACK_ADC_STOP,
  TRACK_DISABLED,
  TRACK_STALE_STOP
} LineTrackStatus;

typedef struct
{
  uint16_t left;
  uint16_t right;
  uint32_t stamp;
  uint8_t armed;
} LineTrackCommand;

// 主循环维护试跑状态。丢线计时不跨试跑保留，重新见到有效线时清除。
static uint8_t running=0U;
static uint8_t lost=0U;
static uint8_t have_line=0U;
static uint8_t derivative_valid=0U;
static uint32_t lost_since=0U;
static uint32_t scan_since=0U;
static uint32_t oled_since=0U;
static float last_error=0.0f;
static int16_t last_turn=0;
static LineTrackStatus status=TRACK_READY;
static GrayLineReading frame;

// PE0 上拉、低有效。保存最近的原始/稳定电平和变化时刻，按住 USER 只触发一次。
static GPIO_PinState raw_key=GPIO_PIN_SET;
static GPIO_PinState stable_key=GPIO_PIN_SET;
static uint32_t key_since=0U;

// 主循环在短临界区提交命令，TIM6 读取。TIM6 检测断使能或超期时清除授权并通知主循环。
static volatile LineTrackCommand command={0U,0U,0U,0U};
static volatile LineTrackStatus interrupt_stop=TRACK_READY;
static volatile uint8_t drive_active=0U;
// 四个轮子的基准、误差和 PI 累积互相独立，只在 TIM6 中断更新，重新起跑时清除。
static WheelSpeedController controller[MOTOR_DRIVE_WHEEL_COUNT];

// ================================================================
// 1. 灰度外环与前向车轮目标
//
//    黑线偏右时 e>0，左侧 A/D 提速、右侧 B/C 降速，车头向右回线。
//    调参阶段在宽线时低速直行，丢线时短暂按上一次方向减速寻找。
// ================================================================
static void line_track_publish(uint16_t left,uint16_t right,uint32_t now)
{
  uint32_t interrupts=__get_PRIMASK();

  __disable_irq();
  // 外环采样途中如已收到中断停机通知，本帧不得重新授权驱动。
  if(interrupt_stop==TRACK_READY||running==0U)
  {
    command.left=left;
    command.right=right;
    command.stamp=now;
    command.armed=running;
  }
  __set_PRIMASK(interrupts);
}

static void line_track_stop(LineTrackStatus reason,uint32_t now)
{
  running=0U;
  lost=0U;
  derivative_valid=0U;
  status=reason;
  line_track_publish(0U,0U,now);
}

static void line_track_set_lost(uint32_t now)
{
  int16_t turn=last_turn;

  if(lost==0U)
  {
    lost=1U;
    lost_since=now;
  }
  derivative_valid=0U;  // 重新捕获的首帧从新的误差起算，避免 D 项突变。
  status=TRACK_LOST;
  if(have_line==0U)
  {
    // 尚未识别到线时原地等待，超过两秒仍由主循环取消本次试跑。
    line_track_publish(0U,0U,now);
    return;
  }
  if(turn>TRACK_LOST_TURN_LIMIT)  turn=TRACK_LOST_TURN_LIMIT;
  if(turn<-TRACK_LOST_TURN_LIMIT)  turn=-TRACK_LOST_TURN_LIMIT;
  line_track_publish((uint16_t)(TRACK_SLOW_SPEED+turn),(uint16_t)(TRACK_SLOW_SPEED-turn),now);
}

static void line_track_process(uint32_t now)
{
  LineTuningGains gains;
  float difference;
  float correction;

  if(frame.state==GRAY_LINE_LOST)
  {
    line_track_set_lost(now);
    return;
  }
  lost=0U;
  if(frame.state==GRAY_LINE_WIDE)
  {
    derivative_valid=0U;
    status=TRACK_WIDE;
    // 宽黑或多段黑线的重心不代表方向。已有见线历史时低速保持前行。
    line_track_publish(have_line?TRACK_SLOW_SPEED:0U,have_line?TRACK_SLOW_SPEED:0U,now);
    return;
  }

  gains=LineTuning_GetGains();
  difference=derivative_valid?frame.error-last_error:0.0f;
  correction=gains.kp*frame.error+gains.kd*difference;
  if(correction>TRACK_TURN_LIMIT)  correction=TRACK_TURN_LIMIT;
  if(correction<-TRACK_TURN_LIMIT)  correction=-TRACK_TURN_LIMIT;
  last_turn=(int16_t)(correction+(correction>=0.0f?0.5f:-0.5f));
  last_error=frame.error;
  derivative_valid=1U;
  have_line=1U;
  status=TRACK_RUN;
  line_track_publish((uint16_t)(TRACK_FORWARD_SPEED+last_turn),(uint16_t)(TRACK_FORWARD_SPEED-last_turn),now);
}

// ================================================================
// 2. USER 启停、OLED 和帧调度
// ================================================================
static void line_track_draw(void)
{
  static char *titles[]=
  {
    "Line track ready","Tracking        ","Wide: slow      ","Line lost       ",
    "USER stop       ","Lost 2s: stop   ","ADC error: stop ","Enable switch   ","Frame timeout   "
  };
  LineTuningGains gains=LineTuning_GetGains();
  char text[17];

  OLED_ShowString(1,1,titles[status]);
  (void)snprintf(text,sizeof(text),"E%+6.2f M=%02X    ",(double)frame.error,(unsigned int)frame.mask);
  OLED_ShowString(2,1,text);
  (void)snprintf(text,sizeof(text),"P%4.2f D%5.2f    ",(double)gains.kp,(double)gains.kd);
  OLED_ShowString(3,1,text);
  OLED_ShowString(4,1,running?"USER: Stop      ":"USER: Start     ");
  oled_since=HAL_GetTick();
}

static uint8_t line_track_key_pressed(uint32_t now)
{
  GPIO_PinState level=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);

  if(level!=raw_key)
  {
    raw_key=level;
    key_since=now;
  }
  if(stable_key!=raw_key&&(now-key_since)>=TRACK_DEBOUNCE_MS)
  {
    stable_key=raw_key;
    return stable_key==GPIO_PIN_RESET;
  }
  return 0U;
}

void LineTrack_Init(void)
{
  GrayLine_Init();
  WheelSpeed_Init();
  LineTuning_Init();
  raw_key=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);
  stable_key=raw_key;
  key_since=HAL_GetTick();
  if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin)==GPIO_PIN_RESET)  status=TRACK_DISABLED;
  line_track_draw();

  // TIM6 由 84 MHz/(84×10000) 得到 100 Hz。清掉初始化生成的更新标志再开启中断。
  __HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
  if(HAL_TIM_Base_Start_IT(&htim6)!=HAL_OK)
  {
    Error_Handler();
  }
}

void LineTrack_Task(void)
{
  uint32_t now=HAL_GetTick();
  uint8_t pressed=line_track_key_pressed(now);
  GPIO_PinState enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);
  LineTrackStatus abort_reason=interrupt_stop;

  // ------- 1. 应用停机联锁和 USER 边沿 -------
  if(abort_reason!=TRACK_READY)
  {
    interrupt_stop=TRACK_READY;
    line_track_stop(abort_reason,now);
  }
  if(enable==GPIO_PIN_RESET)
  {
    if(status!=TRACK_DISABLED)  line_track_stop(TRACK_DISABLED,now);
    pressed=0U;
    // 开关关闭期间已经按住的按键不可在开关恢复后延迟触发。
    stable_key=raw_key;
  }
  else if(status==TRACK_DISABLED)  status=TRACK_READY;
  if(pressed!=0U)
  {
    if(running!=0U)  line_track_stop(TRACK_USER_STOP,now);
    else
    {
      running=1U;
      lost=0U;
      have_line=0U;
      derivative_valid=0U;
      last_turn=0;
      status=TRACK_RUN;
      scan_since=now-TRACK_SCAN_MS;
    }
  }
  if(running!=0U&&lost!=0U&&(now-lost_since)>=TRACK_LOST_STOP_MS)
  {
    line_track_stop(TRACK_LOST_STOP,now);
  }

  // ------- 2. 完成八路采样后提交外环目标 -------
  if((now-scan_since)>=TRACK_SCAN_MS)
  {
    scan_since=now;
    if(GrayLine_Read(&frame)==0U)
    {
      if(running!=0U)  line_track_stop(TRACK_ADC_STOP,now);
    }
    else if(running!=0U)  line_track_process(HAL_GetTick());
  }

  // ------- 3. 处理低优先级通信和显示 -------
  LineTuning_Task((uint8_t)(running!=0U||drive_active!=0U));
  if((HAL_GetTick()-oled_since)>=TRACK_OLED_MS)  line_track_draw();
}

// ================================================================
// 3. TIM6 速度环与命令超时保护
//
//    中断只读本帧目标和更新电机，USART2、ADC 与 OLED 留在主循环。
//    PD3 低电平、失去授权或目标超期时撤去 PWM，恢复使能仍需 USER。
// ================================================================
void LineTrack_ControlTick(void)
{
  LineTrackStatus reason=TRACK_READY;
  uint8_t wheel;

  if(command.armed!=0U)
  {
    if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin)==GPIO_PIN_RESET)  reason=TRACK_DISABLED;
    else if((HAL_GetTick()-command.stamp)>=TRACK_STALE_MS)  reason=TRACK_STALE_STOP;
    if(reason!=TRACK_READY)
    {
      command.armed=0U;
      interrupt_stop=reason;
    }
  }
  if(command.armed==0U||command.left==0U||command.right==0U)
  {
    if(drive_active!=0U)
    {
      MotorDrive_StopOutputs();
      drive_active=0U;
    }
    return;
  }
  if(drive_active==0U)
  {
    for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)  WheelSpeed_Reset(wheel,&controller[wheel]);
    drive_active=1U;
    return;  // 用下一拍的完整 10 ms 计数更新首次驱动。
  }
  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    uint16_t target=(wheel==0U||wheel==3U)?command.left:command.right;
    (void)WheelSpeed_Update(wheel,target,&controller[wheel]);
  }
}
