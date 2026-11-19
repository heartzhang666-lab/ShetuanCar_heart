/**
 * @file    motor_calibration.c
 * @brief   使用 USER 按键和 OLED 逐轮验证四只电机在慢衰减下的实际转向。
 *
 * 文件结构：
 *   1. motor_cal_write_step()  — 按电气极性为所选轮生成占空比
 *   2. motor_cal_read_key()    — 消抖并识别按下边沿
 *   3. motor_cal_disable()     — 取消测试并提示使能状态
 *   4. MotorCal_Init()         — 启动零占空比 PWM，显示上电待机状态
 *   5. MotorCal_Task()         — 执行选轮、试转、间歇和停止
 */

#include "motor_calibration.h"
#include "main.h"
#include "OLED.h"
#include "motor_drive.h"

// USER（PE0）上拉输入、低有效。消抖时间增大可抑制更长的触点抖动，但切轮响应会变慢。
#define MOTOR_CAL_DEBOUNCE_MS 25U

// 试转时间越长越便于观察高档转向，电机持续受力也越久。暂停时间用于留出惯性滑行时间。
#define MOTOR_CAL_DRIVE_MS 800U
#define MOTOR_CAL_PAUSE_MS 1200U

typedef enum
{
  MOTOR_CAL_IDLE,
  MOTOR_CAL_PAUSE,
  MOTOR_CAL_DRIVE
} MotorCalPhase;

// ================================================================
// 1. 单轮试转档位
//
//    轮位顺序与 motor_drive.c 中的 AT8236 输入表一致。
//    OLED 的正负表示电气方向，在试转中直接发送原始指令以核对实际前进极性。
// ================================================================
static char wheel_labels[MOTOR_DRIVE_WHEEL_COUNT][17]=
{
  "A Left rear     ",
  "B Right rear    ",
  "C Right front   ",
  "D Left front    "
};

// 共有八个阶段，同一占空比先测 + 再测 -，便于直接比较这两个方向。
static const uint8_t duty_levels[]={20U,40U,60U,80U};
#define MOTOR_CAL_STEP_COUNT (2U*(sizeof(duty_levels)/sizeof(duty_levels[0])))

// 这些状态由主循环独占，跨循环保存轮位、当前阶段、消抖和定时起点。
// 初始索引指向 D，第一次有效按键将索引回绕到 A。初始化时不启动任何试转。
static uint8_t selected_wheel=MOTOR_DRIVE_WHEEL_COUNT-1U;
static uint8_t step_index=0U;
static MotorCalPhase phase=MOTOR_CAL_IDLE;
static uint32_t phase_since=0U;
static GPIO_PinState raw_key=GPIO_PIN_SET;
static GPIO_PinState stable_key=GPIO_PIN_SET;
static uint32_t key_since=0U;
static GPIO_PinState last_enable=GPIO_PIN_RESET;

static void motor_cal_write_step(void)
{
  int8_t duty[MOTOR_DRIVE_WHEEL_COUNT]={0};
  int8_t percent=(int8_t)duty_levels[step_index/2U];

  // 偶数档使用原始正方向，奇数档反转。其余轮子的指令为零，保持滑行。
  duty[selected_wheel]=(step_index&1U)?-percent:percent;
  MotorDrive_RunRaw(duty);
}

/**
 * @brief  对 USER 按键消抖，只在稳定的高到低边沿生成一次切换事件。
 * @param  now 本次轮询时间，单位为 ms。
 * @return 1 表示得到一次有效按下，0 表示保持、释放或仍在消抖。
 */
static uint8_t motor_cal_read_key(uint32_t now)
{
  GPIO_PinState level=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);

  if(level!=raw_key)
  {
    raw_key=level;
    key_since=now;
  }
  if(stable_key!=raw_key&&(now-key_since)>=MOTOR_CAL_DEBOUNCE_MS)
  {
    stable_key=raw_key;
    return stable_key==GPIO_PIN_RESET;
  }
  return 0U;
}

static void motor_cal_show_step(void)
{
  uint8_t duty=duty_levels[step_index/2U];
  char text[]="Run +00%        ";

  text[4]=(step_index&1U)?'-':'+';
  text[5]=(char)('0'+duty/10U);
  text[6]=(char)('0'+duty%10U);
  OLED_ShowString(3,1,text);
}

static void motor_cal_disable(void)
{
  MotorDrive_StopOutputs();
  phase=MOTOR_CAL_IDLE;
  last_enable=GPIO_PIN_RESET;
  OLED_ShowString(3,1,"Stopped         ");
  OLED_ShowString(4,1,"Enable switch   ");
}

// ================================================================
// 2. 逐轮试转流程
//
//    PD3 高电平允许运行，低电平撤去驱动。每次有效按下 USER 先清除旧轮输出，
//    再选择 A → B → C → D → A。重新拨到允许位置后等待下一次按键。
// ================================================================
void MotorCal_Init(void)
{
  // ------- 1. 全部通道以零占空比启动 -------
  MotorDrive_Init();

  // ------- 2. 用上电实测按键电平建立消抖基线 -------
  raw_key=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);
  stable_key=raw_key;
  key_since=HAL_GetTick();
  last_enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);

  // ------- 3. 显示待机状态，由用户决定何时试转 -------
  OLED_ShowString(1,1,"Motor slow decay");
  OLED_ShowString(2,1,"Wheel: -        ");
  OLED_ShowString(3,1,"Stopped         ");
  OLED_ShowString(4,1,last_enable==GPIO_PIN_SET?"USER: Next wheel":"Enable switch   ");
}

void MotorCal_Task(void)
{
  uint32_t now=HAL_GetTick();
  uint8_t pressed=motor_cal_read_key(now);
  GPIO_PinState enable=HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin);

  // ------- 1. 优先处理使能开关 -------
  if(enable==GPIO_PIN_RESET)
  {
    if(last_enable==GPIO_PIN_SET)  motor_cal_disable();
    return;
  }
  if(last_enable==GPIO_PIN_RESET)
  {
    last_enable=enable;
    stable_key=raw_key;  // 使能恢复时已按住的按键需要先释放，避免其消抖完成后自动起转。
    OLED_ShowString(4,1,"USER: Next wheel");
    // 恢复使能当轮只刷新提示，再收到新的按下边沿才开始试转。
    return;
  }

  // ------- 2. 切换到下一轮并预留滑行间隔 -------
  if(pressed!=0U)
  {
    MotorDrive_StopOutputs();
    selected_wheel=(uint8_t)((selected_wheel+1U)%MOTOR_DRIVE_WHEEL_COUNT);
    step_index=0U;
    phase=MOTOR_CAL_PAUSE;
    OLED_ShowString(2,1,wheel_labels[selected_wheel]);
    OLED_ShowString(3,1,"Starting...     ");
    phase_since=HAL_GetTick();
    return;
  }

  // ------- 3. 试转到时后关闭 PWM，或者在滑行结束后开始下一段 -------
  // 使用无符号差值判断等待时间，HAL tick 自然回绕时仍可正常工作。
  if(phase==MOTOR_CAL_DRIVE&&(now-phase_since)>=MOTOR_CAL_DRIVE_MS)
  {
    MotorDrive_StopOutputs();
    step_index++;
    if(step_index>=MOTOR_CAL_STEP_COUNT)
    {
      phase=MOTOR_CAL_IDLE;
      OLED_ShowString(3,1,"Done            ");
    }
    else
    {
      phase=MOTOR_CAL_PAUSE;
      OLED_ShowString(3,1,"Coast           ");
      phase_since=HAL_GetTick();
    }
  }
  else if(phase==MOTOR_CAL_PAUSE&&(now-phase_since)>=MOTOR_CAL_PAUSE_MS)
  {
    // OLED 软件串行刷新期间保持电机停机。刷新完成后再次确认 PD3，避免此时关开关仍启动。
    motor_cal_show_step();
    if(HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin)==GPIO_PIN_RESET)
    {
      motor_cal_disable();
      return;
    }
    motor_cal_write_step();
    phase=MOTOR_CAL_DRIVE;
    phase_since=HAL_GetTick();
  }
}
