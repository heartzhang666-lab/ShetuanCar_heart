/**
 * @file    motor_calibration.c
 * @brief   使用 USER 按键和 OLED 逐轮验证四只电机在慢衰减下的实际转向。
 *
 * 文件结构：
 *   1. MotorCal_StopOutputs()  — 清零全部驱动输入，输出进入滑行状态
 *   2. motor_cal_write_step()  — 根据轮位、方向和占空比设置 PWM
 *   3. motor_cal_read_key()    — 消抖并识别按下边沿
 *   4. motor_cal_disable()     — 取消测试并提示使能状态
 *   5. MotorCal_Init()         — 启动零占空比 PWM，显示上电待机状态
 *   6. MotorCal_Task()         — 执行选轮、试转、间歇和停止
 */

#include "motor_calibration.h"
#include "main.h"
#include "OLED.h"
#include "tim.h"

#define MOTOR_CAL_WHEEL_COUNT 4U

// USER（PE0）上拉输入、低有效。消抖时间增大可抑制更长的触点抖动，但切轮响应会变慢。
#define MOTOR_CAL_DEBOUNCE_MS 25U

// 试转时间越长越便于观察高档转向，电机持续受力也越久。暂停时间用于留出惯性滑行时间。
#define MOTOR_CAL_DRIVE_MS 800U
#define MOTOR_CAL_PAUSE_MS 1200U

typedef struct
{
  TIM_HandleTypeDef *in1_timer;
  uint32_t in1_channel;
  TIM_HandleTypeDef *in2_timer;
  uint32_t in2_channel;
  char label[17];
} MotorCalWheel;

typedef enum
{
  MOTOR_CAL_IDLE,
  MOTOR_CAL_PAUSE,
  MOTOR_CAL_DRIVE
} MotorCalPhase;

// ================================================================
// 1. C30D 板载 AT8236 输入与轮位
//
//    这里的 IN1/IN2 指 AT8236 芯片引脚。PWM 引脚由 MCU 复用推挽输出，带下拉。
//    A 左后 PB9/PB8，B 右后 PE5/PE6。
//    C 右前 PE11/PE9，D 左前 PE14/PE13。C/D 共用 TIM1 的周期，各自有独立比较寄存器。
//    试转的“+”是 IN1 保持高电平、IN2 输出反相占空比的 PWM，“-”交换两路。
//    调制脚为低时驱动电机，为高时两个输入都为高，绕组续流并进入慢衰减。
//    这两个符号用于辨别电气极性，车体前进方向以实物轮子顶部朝车头转动为准。
// ================================================================
static MotorCalWheel wheels[MOTOR_CAL_WHEEL_COUNT]=
{
  {&htim11,TIM_CHANNEL_1,&htim10,TIM_CHANNEL_1,"A Left rear     "},
  {&htim9,TIM_CHANNEL_1,&htim9,TIM_CHANNEL_2,"B Right rear    "},
  {&htim1,TIM_CHANNEL_2,&htim1,TIM_CHANNEL_1,"C Right front   "},
  {&htim1,TIM_CHANNEL_4,&htim1,TIM_CHANNEL_3,"D Left front    "}
};

// 共有八个阶段，同一占空比先测 + 再测 -，便于直接比较这两个方向。
static const uint8_t duty_levels[]={20U,40U,60U,80U};
#define MOTOR_CAL_STEP_COUNT (2U*(sizeof(duty_levels)/sizeof(duty_levels[0])))

// 这些状态由主循环独占，跨循环保存轮位、当前阶段、消抖和定时起点。
// 初始索引指向 D，第一次有效按键将索引回绕到 A。初始化时不启动任何试转。
static uint8_t selected_wheel=MOTOR_CAL_WHEEL_COUNT-1U;
static uint8_t step_index=0U;
static MotorCalPhase phase=MOTOR_CAL_IDLE;
static uint32_t phase_since=0U;
static GPIO_PinState raw_key=GPIO_PIN_SET;
static GPIO_PinState stable_key=GPIO_PIN_SET;
static uint32_t key_since=0U;
static GPIO_PinState last_enable=GPIO_PIN_RESET;

// 定时器初始化完成后才允许写比较寄存器。早期时钟错误进入 Error_Handler 时此值仍为 0。
static uint8_t pwm_ready=0U;

// 保存当前调制脚，停机时先使它为低，再清除恒高脚，避免换相时出现反向脉冲。
static TIM_HandleTypeDef *active_pwm_timer=NULL;
static uint32_t active_pwm_channel=0U;

void MotorCal_StopOutputs(void)
{
  uint8_t i;

  if(pwm_ready==0U)  return;
  if(active_pwm_timer!=NULL)
  {
    // 比较值写入预装载寄存器。先强制装载调制脚的低电平，再撤去恒高脚。
    __HAL_TIM_SET_COMPARE(active_pwm_timer,active_pwm_channel,0U);
    active_pwm_timer->Instance->EGR=TIM_EGR_UG;
    active_pwm_timer=NULL;
  }
  for(i=0U;i<MOTOR_CAL_WHEEL_COUNT;i++)
  {
    __HAL_TIM_SET_COMPARE(wheels[i].in1_timer,wheels[i].in1_channel,0U);
    __HAL_TIM_SET_COMPARE(wheels[i].in2_timer,wheels[i].in2_channel,0U);
  }

  // 四个定时器都装载零比较值，AT8236 的两路输入均为低，电机滑行。
  // 标定 PWM 未启用更新中断，软件更新事件只用于同步装载和重置计数器。
  htim1.Instance->EGR=TIM_EGR_UG;
  htim9.Instance->EGR=TIM_EGR_UG;
  htim10.Instance->EGR=TIM_EGR_UG;
  htim11.Instance->EGR=TIM_EGR_UG;
}

static void motor_cal_write_step(void)
{
  const MotorCalWheel *wheel=&wheels[selected_wheel];
  uint8_t duty=duty_levels[step_index/2U];
  uint8_t reverse=step_index&1U;
  TIM_HandleTypeDef *hold_timer=reverse?wheel->in2_timer:wheel->in1_timer;
  uint32_t hold_channel=reverse?wheel->in2_channel:wheel->in1_channel;
  TIM_HandleTypeDef *pwm_timer=reverse?wheel->in1_timer:wheel->in2_timer;
  uint32_t pwm_channel=reverse?wheel->in1_channel:wheel->in2_channel;
  uint32_t pwm_counts=__HAL_TIM_GET_AUTORELOAD(pwm_timer)+1U;

  // PWM1 的 CCR=ARR+1 表示恒高。A 轮的两路定时器独立，先装载恒高脚再启动调制脚，
  // 避免调制脚先变高时产生反向驱动。共享计数器的轮子共用一次更新事件。
  __HAL_TIM_SET_COMPARE(hold_timer,hold_channel,__HAL_TIM_GET_AUTORELOAD(hold_timer)+1U);
  if(hold_timer!=pwm_timer)  hold_timer->Instance->EGR=TIM_EGR_UG;

  // OLED 上的占空比是有效驱动占比，调制脚为低才驱动。20% 对应高 80 us、低 20 us。
  // 高电平区间两路同时为高，电流通过桥内低侧通路续流。
  active_pwm_timer=pwm_timer;
  active_pwm_channel=pwm_channel;
  __HAL_TIM_SET_COMPARE(pwm_timer,pwm_channel,pwm_counts-(pwm_counts*duty)/100U);
  pwm_timer->Instance->EGR=TIM_EGR_UG;
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
  MotorCal_StopOutputs();
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
  uint8_t i;

  // ------- 1. 全部通道以零占空比启动 -------
  pwm_ready=1U;
  MotorCal_StopOutputs();
  for(i=0U;i<MOTOR_CAL_WHEEL_COUNT;i++)
  {
    if(HAL_TIM_PWM_Start(wheels[i].in1_timer,wheels[i].in1_channel)!=HAL_OK||
       HAL_TIM_PWM_Start(wheels[i].in2_timer,wheels[i].in2_channel)!=HAL_OK)
    {
      // 启动失败时保留零驱动，统一进入工程错误处理。
      Error_Handler();
    }
  }

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
    MotorCal_StopOutputs();
    selected_wheel=(uint8_t)((selected_wheel+1U)%MOTOR_CAL_WHEEL_COUNT);
    step_index=0U;
    phase=MOTOR_CAL_PAUSE;
    OLED_ShowString(2,1,wheels[selected_wheel].label);
    OLED_ShowString(3,1,"Starting...     ");
    phase_since=HAL_GetTick();
    return;
  }

  // ------- 3. 试转到时后关闭 PWM，或者在滑行结束后开始下一段 -------
  // 使用无符号差值判断等待时间，HAL tick 自然回绕时仍可正常工作。
  if(phase==MOTOR_CAL_DRIVE&&(now-phase_since)>=MOTOR_CAL_DRIVE_MS)
  {
    MotorCal_StopOutputs();
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
