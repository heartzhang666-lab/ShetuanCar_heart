/**
 * @file    motor_drive.c
 * @brief   按慢衰减方式控制 C30D 板载四路 AT8236。
 *
 * 文件结构：
 *   1. motor_drive_update_outputs() — 提交预装载的比较值
 *   2. MotorDrive_StopOutputs()     — 按安全顺序将八路输入拉低
 *   3. MotorDrive_Init()            — 以零占空比启用各 PWM 通道
 *   4. MotorDrive_RunRaw()          — 根据带符号占空比建立四路驱动
 */

#include "motor_drive.h"
#include <stddef.h>
#include "main.h"
#include "tim.h"

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
} MotorDriveInput;

// ================================================================
// 1. C30D 四路 PWM 接线
//
//    每行依次为 AT8236 芯片 IN1、IN2，GPIO 复用推挽输出并下拉。
//    A 左后 PB9/PB8，B 右后 PE5/PE6，C 右前 PE11/PE9，D 左前 PE14/PE13。
//    C、D 共用 TIM1，A 的两路分别由 TIM11、TIM10 输出。
// ================================================================
static const MotorDriveInput inputs[MOTOR_DRIVE_WHEEL_COUNT][2]=
{
  {{&htim11,TIM_CHANNEL_1},{&htim10,TIM_CHANNEL_1}},
  {{&htim9,TIM_CHANNEL_1},{&htim9,TIM_CHANNEL_2}},
  {{&htim1,TIM_CHANNEL_2},{&htim1,TIM_CHANNEL_1}},
  {{&htim1,TIM_CHANNEL_4},{&htim1,TIM_CHANNEL_3}}
};

// 每轮跨主循环记住当前调制脚，停止时先清调制脚，避免误产生短暂反向驱动。
static const MotorDriveInput *active_pwm[MOTOR_DRIVE_WHEEL_COUNT]={NULL};
// 时钟或定时器初始化出错时，ready 仍为 0，停机接口不会访问未初始化的句柄。
static uint8_t ready=0U;

static void motor_drive_update_outputs(void)
{
  // 四个 PWM 定时器未启用更新中断。软件更新事件提交比较值并重置其计数器。
  htim1.Instance->EGR=TIM_EGR_UG;
  htim9.Instance->EGR=TIM_EGR_UG;
  htim10.Instance->EGR=TIM_EGR_UG;
  htim11.Instance->EGR=TIM_EGR_UG;
}

void MotorDrive_StopOutputs(void)
{
  uint8_t wheel;

  if(ready==0U)  return;
  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    const MotorDriveInput *pwm=active_pwm[wheel];
    if(pwm!=NULL)
    {
      __HAL_TIM_SET_COMPARE(pwm->timer,pwm->channel,0U);
      active_pwm[wheel]=NULL;
    }
  }
  motor_drive_update_outputs();

  // 调制脚已变低，此时撤去恒高脚，00 使 AT8236 进入滑行状态。
  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    __HAL_TIM_SET_COMPARE(inputs[wheel][0].timer,inputs[wheel][0].channel,0U);
    __HAL_TIM_SET_COMPARE(inputs[wheel][1].timer,inputs[wheel][1].channel,0U);
  }
  motor_drive_update_outputs();
}

void MotorDrive_Init(void)
{
  uint8_t wheel;

  ready=1U;
  MotorDrive_StopOutputs();
  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    if(HAL_TIM_PWM_Start(inputs[wheel][0].timer,inputs[wheel][0].channel)!=HAL_OK||
       HAL_TIM_PWM_Start(inputs[wheel][1].timer,inputs[wheel][1].channel)!=HAL_OK)
    {
      // 任何通道启动失败均进入工程错误处理，已启动的通道保持零输出。
      Error_Handler();
    }
  }
}

// ================================================================
// 2. AT8236 慢衰减输出
//
//    正负分别选择 IN1、IN2 为恒高脚。调制脚低电平时驱动，高电平时两路
//    同为高，绕组经桥内低侧通路续流。PWM1 高有效，因此用反相比较值。
// ================================================================
void MotorDrive_RunRaw(const int8_t duty[MOTOR_DRIVE_WHEEL_COUNT])
{
  uint8_t wheel;

  if(ready==0U)  return;

  // ------- 1. 清除上一次驱动 -------
  MotorDrive_StopOutputs();

  // ------- 2. 先提交所有非零命令的恒高脚 -------
  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    if(duty[wheel]!=0)
    {
      uint8_t direction=duty[wheel]<0?1U:0U;
      const MotorDriveInput *hold=&inputs[wheel][direction];

      // CCR=ARR+1 表示恒高。先装载恒高脚，再使调制脚输出，避免不同定时器间出现反向脉冲。
      __HAL_TIM_SET_COMPARE(hold->timer,hold->channel,__HAL_TIM_GET_AUTORELOAD(hold->timer)+1U);
      active_pwm[wheel]=&inputs[wheel][1U-direction];
    }
  }
  motor_drive_update_outputs();

  // ------- 3. 装载反相 PWM -------
  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    const MotorDriveInput *pwm=active_pwm[wheel];
    if(pwm!=NULL)
    {
      uint32_t counts=__HAL_TIM_GET_AUTORELOAD(pwm->timer)+1U;
      uint32_t percent=(uint32_t)(duty[wheel]<0?-duty[wheel]:duty[wheel]);

      if(percent>100U)  percent=100U;
      __HAL_TIM_SET_COMPARE(pwm->timer,pwm->channel,counts-(counts*percent)/100U);
    }
  }
  motor_drive_update_outputs();
}
