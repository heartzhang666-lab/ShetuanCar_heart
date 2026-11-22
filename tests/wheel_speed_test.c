/**
 * @file    wheel_speed_test.c
 * @brief   验证只读反馈接口的正负方向、16/32 位回绕以及原 PI 数值不变。
 */
#include <assert.h>
#include <stdio.h>
#include "../ShetuanCar/bsp/wheel_speed.c"

TIM_HandleTypeDef htim2={0U,UINT32_MAX};
TIM_HandleTypeDef htim3={0U,UINT16_MAX};
TIM_HandleTypeDef htim4={0U,UINT16_MAX};
TIM_HandleTypeDef htim5={0U,UINT32_MAX};
const int8_t MotorDrive_ForwardDirection[4]={1,-1,1,1};
static uint32_t write_count;
static uint16_t last_output;
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer,uint32_t channels)
{
  (void)timer;
  (void)channels;
  return HAL_OK;
}
void Error_Handler(void) {assert(!"unexpected Error_Handler");}
void MotorDrive_Init(void) {}
void MotorDrive_SetWheelDuty(uint8_t wheel,int8_t direction,uint16_t output)
{
  assert(direction==MotorDrive_ForwardDirection[wheel]);
  write_count++;
  last_output=output;
}

int main(void)
{
  WheelSpeedController c;
  uint8_t wheel;

  WheelSpeed_Init();
  for(wheel=0U;wheel<4U;wheel++)
  {
    TIM_HandleTypeDef *timer=encoders[wheel];
    timer->counter=2U;
    WheelSpeed_Reset(wheel,&c);
    timer->counter=timer->autoreload-2U;
    assert(WheelSpeed_Read(wheel,&c)==-5*encoder_direction[wheel]);
    timer->counter=2U;
    assert(WheelSpeed_Read(wheel,&c)==5*encoder_direction[wheel]);
    assert(WheelSpeed_Read(wheel,&c)==0);
  }
  assert(write_count==0U);  // 只读接口不得触发驱动或 PI 更新。
  htim2.counter=0U;
  WheelSpeed_Reset(0U,&c);
  assert(WheelSpeed_Update(0U,10U,&c)==0);
  assert(last_output==320U);  // (Kp+Ki)*10，原参数 20/12/0。
  htim2.counter=10U;
  assert(WheelSpeed_Update(0U,10U,&c)==10);
  assert(last_output==120U);  // 误差回零时 P 增量减去 200，保留原增量 PI 行为。
  c.output=39999.0f;
  assert(WheelSpeed_Update(0U,100U,&c)==0);
  assert(last_output==MOTOR_DRIVE_COMMAND_MAX);
  htim2.counter=10000U;
  (void)WheelSpeed_Update(0U,1U,&c);
  assert(last_output==0U);
  puts("PASS four-wheel direction, 16/32-bit wrap, read-only sampling and unchanged PI/saturation");
  return 0;
}
