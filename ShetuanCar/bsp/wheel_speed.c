/**
 * @file    wheel_speed.c
 * @brief   复用 Car_C30D 直线速度环，实现四轮各自的前向闭环调速。
 *
 * 文件结构：
 *   1. WheelSpeed_Init()   — 启动四组 TI12 编码器和 PWM 输出
 *   2. WheelSpeed_Reset()  — 为本轮试跑建立计数和误差基线
 *   3. WheelSpeed_Read()   — 只读取带方向计数差值，不改变驱动输出
 *   4. WheelSpeed_Update() — 用采样结果求增量 PID 并更新该轮驱动
 */

#include "wheel_speed.h"
#include "main.h"
#include "motor_drive.h"
#include "tim.h"

// ================================================================
// 1. Car_C30D 直线速度环参数
//
//    保留旧工程 10 ms 采样、计数/周期反馈和 0～40000 驱动量。
//    增大 Kp 会使误差变化时修正更强，过大会引起轮速摆动。
//    增大 Ki 能更快消除稳态误差，过大会增加起步超调。
//    Kd 保持为 0，启用后会对误差变化加阻尼，也可能放大量化噪声。
// ================================================================
#define WHEEL_SPEED_KP 20.0f
#define WHEEL_SPEED_KI 12.0f
#define WHEEL_SPEED_KD 0.0f

static TIM_HandleTypeDef *const encoders[MOTOR_DRIVE_WHEEL_COUNT]={&htim2,&htim3,&htim4,&htim5};

// A 左后、B 右后、C 右前、D 左前。B、C 前进时原始计数减小，独立于电机驱动极性。
static const int8_t encoder_direction[MOTOR_DRIVE_WHEEL_COUNT]={1,-1,-1,1};

void WheelSpeed_Init(void)
{
  uint8_t wheel;

  for(wheel=0U;wheel<MOTOR_DRIVE_WHEEL_COUNT;wheel++)
  {
    if(HAL_TIM_Encoder_Start(encoders[wheel],TIM_CHANNEL_ALL)!=HAL_OK)
    {
      // 任一路启动失败都结束初始化，避免把缺失的反馈当成静止。
      Error_Handler();
    }
  }
  MotorDrive_Init();
}

void WheelSpeed_Reset(uint8_t wheel,WheelSpeedController *controller)
{
  controller->last_counter=__HAL_TIM_GET_COUNTER(encoders[wheel]);
  controller->last_error=0.0f;
  controller->older_error=0.0f;
  controller->output=0.0f;
}

// ================================================================
// 2. 轮速采样和增量控制
//
//    TIM2/5 使用 32 位满量程，TIM3/4 使用 16 位满量程。
//    原始差值按计数器位宽取模。相邻 10 ms 的位移折算值须小于半个计数范围，
//    上半区解释为负增量，从而在 CNT 跨过 0 或上限时维持速度符号。
// ================================================================
int32_t WheelSpeed_Read(uint8_t wheel,WheelSpeedController *controller)
{
  TIM_HandleTypeDef *timer=encoders[wheel];
  uint32_t counter=__HAL_TIM_GET_COUNTER(timer);
  uint32_t mask=__HAL_TIM_GET_AUTORELOAD(timer);
  uint32_t difference=(counter-controller->last_counter)&mask;
  int32_t raw_difference=(difference<=(mask>>1U))?(int32_t)difference:-1-(int32_t)(mask-difference);

  // 读取同时推进采样基准，但不改变 PI 历史或 PWM，供终点制动时确认轮子已停止。
  controller->last_counter=counter;
  return raw_difference*encoder_direction[wheel];
}

int32_t WheelSpeed_Update(uint8_t wheel,uint16_t target,WheelSpeedController *controller)
{
  // ------- 1. 读取本周期带方向计数，原有换算、增量 PI 和输出限幅保持不变 -------
  int32_t speed=WheelSpeed_Read(wheel,controller);
  float error=(float)target-(float)speed;

  // ------- 2. 复用旧工程的增量式更新 -------
  // P 作用于相邻误差之差，I 每周期加入当前误差，D 使用二阶差分。
  controller->output+=WHEEL_SPEED_KP*(error-controller->last_error)+
                      WHEEL_SPEED_KI*error+
                      WHEEL_SPEED_KD*(error-2.0f*controller->last_error+controller->older_error);
  controller->older_error=controller->last_error;
  controller->last_error=error;

  // ------- 3. 限幅并连续更新 PWM -------
  // 保持旧直线速度环的非负驱动限制。超速时减小驱动，零驱动为 AT8236 慢衰减制动。
  // 直接限制累计输出，可避免饱和期间继续把越界值累加到后续周期。
  if(controller->output<0.0f)  controller->output=0.0f;
  if(controller->output>(float)MOTOR_DRIVE_COMMAND_MAX)  controller->output=(float)MOTOR_DRIVE_COMMAND_MAX;
  MotorDrive_SetWheelDuty(wheel,MotorDrive_ForwardDirection[wheel],(uint16_t)controller->output);
  return speed;
}
