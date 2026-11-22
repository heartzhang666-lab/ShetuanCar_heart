/**
 * @file    line_track_test.c
 * @brief   用模拟灰度和 HAL 验证真实循迹源码的事件、保护及竞赛状态流转。
 *
 * 文件结构：
 *   1. HAL/外设替身 — 仅记录目标和制动，不模拟车辆动力学
 *   2. 时间与帧注入 — 每 10 ms 中断、主循环执行，ADC 每 20 ms 提供一帧
 *   3. 场景测试     — 起跑、转弯、交叉、终点、异常、回绕及重启
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

// 纳入实际源码以检查内部状态，避免复制一份状态机形成无法发现回归的镜像测试。
#include "../ShetuanCar/bsp/line_track.c"

static uint32_t test_now;
static GPIO_PinState test_enable=GPIO_PIN_SET;
static GPIO_PinState test_key=GPIO_PIN_SET;
static GrayLineReading test_sample;
static uint8_t test_adc_ok=1U;
static int32_t test_brake_motion=0;
static uint16_t test_targets[4];
static uint8_t test_brakes;
static uint32_t test_stops;
TIM_HandleTypeDef htim6;
const int8_t MotorDrive_ForwardDirection[4]={1,-1,1,1};

uint32_t HAL_GetTick(void) {return test_now;}
GPIO_PinState HAL_GPIO_ReadPin(uint32_t port,uint32_t pin)
{
  (void)pin;
  return port==USER_KEY_GPIO_Port?test_key:test_enable;
}
uint32_t __get_PRIMASK(void) {return 0U;}
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t state) {(void)state;}
void Error_Handler(void) {assert(!"unexpected Error_Handler");}
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer) {(void)timer;return HAL_OK;}
void GrayLine_Init(void) {}
uint8_t GrayLine_Read(GrayLineReading *reading) {*reading=test_sample;return test_adc_ok;}
void LineTuning_Init(void) {}
void LineTuning_Task(uint8_t active) {(void)active;}
LineTuningGains LineTuning_GetGains(void) {LineTuningGains g={0.8f,0.0f};return g;}
void OLED_ShowString(uint8_t line,uint8_t column,char *text)
{
  (void)line;
  (void)column;
  assert(strlen(text)<=16U);
}
void WheelSpeed_Init(void) {}
void WheelSpeed_Reset(uint8_t wheel,WheelSpeedController *state)
{
  (void)wheel;
  memset(state,0,sizeof(*state));
}
int32_t WheelSpeed_Update(uint8_t wheel,uint16_t target,WheelSpeedController *state)
{
  (void)state;
  test_targets[wheel]=target;
  return (int32_t)target;
}
int32_t WheelSpeed_Read(uint8_t wheel,WheelSpeedController *state)
{
  (void)state;
  return wheel==2U?test_brake_motion:0;
}
void MotorDrive_StopOutputs(void)
{
  test_stops++;
  memset(test_targets,0,sizeof(test_targets));
  test_brakes=0U;
}
void MotorDrive_SetWheelDuty(uint8_t wheel,int8_t direction,uint16_t output)
{
  assert(direction==MotorDrive_ForwardDirection[wheel]);
  assert(output==0U);
  test_targets[wheel]=0U;
  test_brakes|=(uint8_t)(1U<<wheel);
}

// 每 10 ms 按中断后主循环的顺序执行，覆盖命令刚过期、制动确认等边界。
static void test_ticks(uint32_t milliseconds)
{
  uint32_t elapsed;
  for(elapsed=0U;elapsed<milliseconds;elapsed+=10U)
  {
    test_now+=10U;
    LineTrack_ControlTick();
    LineTrack_Task();
  }
}

static void test_feed(GrayLineState state,uint8_t mask,float error,uint16_t frames)
{
  test_sample.state=state;
  test_sample.mask=mask;
  test_sample.error=error;
  test_ticks((uint32_t)frames*TRACK_SCAN_MS);
}

static void test_reset(uint32_t now)
{
  test_enable=GPIO_PIN_SET;
  test_key=GPIO_PIN_SET;
  test_adc_ok=1U;
  test_brake_motion=0;
  interrupt_stop=TRACK_READY;
  line_track_stop(TRACK_READY,test_now);
  LineTrack_ControlTick();
  test_now=now;
  raw_key=stable_key=GPIO_PIN_SET;
  test_sample.state=GRAY_LINE_NORMAL;
  test_sample.mask=0x18U;
  test_sample.error=0.0f;
  LineTrack_Init();
  line_track_start(test_now);
}

static void test_depart(void)
{
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,TRACK_CLEAR_FRAMES);
  assert(departed!=0U);
  assert(crossings==0U);
}

static void test_cross_once(void)
{
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,30U);
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,TRACK_CONFIRM_FRAMES);
  assert(phase==TRACK_PASS_CROSS);
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,TRACK_CLEAR_FRAMES);
  assert(phase==TRACK_FOLLOW);
}

static void test_start_and_pd(void)
{
  test_reset(1000U);
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,10U);
  assert(crossings==0U&&departed==0U&&running!=0U);
  test_depart();
  test_feed(GRAY_LINE_NORMAL,0x60U,3.0f,2U);
  assert(test_targets[0]==test_targets[3]);
  assert(test_targets[1]==test_targets[2]);
  assert(test_targets[0]>test_targets[1]);
  test_feed(GRAY_LINE_NORMAL,0x06U,-3.0f,2U);
  assert(test_targets[0]<test_targets[1]);
  assert(test_targets[0]>=TRACK_MIN_SPEED&&test_targets[1]>=TRACK_MIN_SPEED);
  puts("PASS start marker and signed PD wheel mapping");
}

static void test_corners(void)
{
  test_reset(1000U);
  test_depart();
  test_feed(GRAY_LINE_NORMAL,0x01U,-7.0f,2U);
  assert(phase==TRACK_SEEK_CORNER&&status==TRACK_TURN_LEFT);
  test_feed(GRAY_LINE_LOST,0U,0.0f,3U);
  assert(test_targets[0]==TRACK_TURN_INNER_SPEED&&test_targets[1]==TRACK_TURN_OUTER_SPEED);
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,9U);
  assert(phase==TRACK_FOLLOW&&running!=0U);
  test_feed(GRAY_LINE_NORMAL,0x80U,7.0f,2U);
  assert(phase==TRACK_SEEK_CORNER&&status==TRACK_TURN_RIGHT);
  test_feed(GRAY_LINE_LOST,0U,0.0f,2U);
  assert(test_targets[0]==TRACK_TURN_OUTER_SPEED&&test_targets[1]==TRACK_TURN_INNER_SPEED);
  test_feed(GRAY_LINE_LOST,0U,0.0f,80U);
  assert(running==0U&&status==TRACK_ELEMENT_STOP);
  puts("PASS left/right corner recovery and corner timeout");
}

static void test_intersections_and_finish(void)
{
  uint32_t hold_time;
  test_reset(1000U);
  test_depart();
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,15U);
  assert(crossings==0U&&phase==TRACK_PASS_CROSS&&status!=TRACK_FINISH_HOLD);
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,TRACK_CLEAR_FRAMES);
  assert(crossings==1U);
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,2U);
  assert(crossings==1U&&phase==TRACK_FOLLOW);  // 同一路口的短时重复不计数。
  test_cross_once();
  assert(crossings==2U);
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,2U);
  assert(status!=TRACK_FINISH_HOLD);  // 第二次路口的边界抖动不能变成终点。
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,30U);
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,1U);
  assert(status!=TRACK_FINISH_HOLD);
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,1U);
  assert(finish_frames==0U);  // 单帧全黑脉冲不能跨帧累积。
  test_feed(GRAY_LINE_WIDE,0x7EU,0.0f,3U);
  assert(phase==TRACK_FOLLOW);  // 横线先出现部分黑，也不能误锁入第三次交叉。
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,2U);
  assert(status==TRACK_FINISH_HOLD);
  test_ticks(20U);
  assert(test_brakes==0x0FU);
  test_brake_motion=1;
  test_ticks(2100U);
  assert(status==TRACK_FINISH_HOLD&&still_ticks==0U);
  test_brake_motion=0;
  hold_time=test_now;
  test_key=GPIO_PIN_RESET;
  test_ticks(1000U);
  assert(status==TRACK_FINISH_HOLD);  // USER 不得提前跳过保持。
  test_key=GPIO_PIN_SET;
  test_ticks(1100U);
  assert(status==TRACK_FINISHED&&running==0U);
  assert((test_now-hold_time)>=TRACK_FINISH_HOLD_MS);
  assert(command.armed==0U);
  test_key=GPIO_PIN_RESET;
  test_ticks(40U);
  assert(running!=0U&&crossings==0U&&departed==0U);  // 必须新的按下才能再跑。
  puts("PASS crossing debounce, two crossings, finish confirmation, hold and restart");
}

static void test_faults(void)
{
  test_reset(1000U);
  test_depart();
  test_feed(GRAY_LINE_LOST,0U,0.0f,45U);
  assert(running==0U&&status==TRACK_LOST_STOP);

  test_reset(1000U);
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,80U);
  assert(running==0U&&status==TRACK_ELEMENT_STOP);

  test_reset(1000U);
  test_depart();
  test_adc_ok=0U;
  test_ticks(40U);
  assert(running==0U&&status==TRACK_ADC_STOP);

  test_reset(1000U);
  test_depart();
  test_enable=GPIO_PIN_RESET;
  test_ticks(10U);
  assert(running==0U&&command.armed==0U&&brake_active==0U);
  test_enable=GPIO_PIN_SET;
  test_ticks(40U);
  assert(running==0U);

  test_reset(1000U);
  test_depart();
  test_now+=TRACK_STALE_MS;
  LineTrack_ControlTick();
  assert(command.armed==0U&&interrupt_stop==TRACK_STALE_STOP);
  line_track_publish(20U,20U,test_now);
  assert(command.armed==0U);  // 主循环中的旧帧不得重新授权。
  LineTrack_Task();
  assert(status==TRACK_STALE_STOP&&running==0U);

  test_reset(1000U);
  test_depart();
  test_now=race_since+TRACK_RACE_LIMIT_MS;
  LineTrack_ControlTick();
  assert(command.armed==0U&&interrupt_stop==TRACK_RACE_STOP);
  LineTrack_Task();
  assert(status==TRACK_RACE_STOP&&running==0U);
  puts("PASS lost/start/ADC/enable/stale/race protections");
}

static void test_recovery_and_wrap(void)
{
  test_reset(UINT32_MAX-60U);
  test_depart();
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,5U);
  assert(running!=0U&&status==TRACK_RUN);
  test_feed(GRAY_LINE_LOST,0U,0.0f,5U);
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,45U);
  assert(running!=0U&&lost==0U);

  test_reset(1000U);
  test_depart();
  test_feed(GRAY_LINE_LOST,0U,0.0f,5U);
  test_feed(GRAY_LINE_WIDE,0xFFU,0.0f,2U);
  test_feed(GRAY_LINE_LOST,0U,0.0f,40U);
  assert(running!=0U&&phase==TRACK_PASS_CROSS);  // 交叉用独立超时，不能被旧丢线误杀。
  test_feed(GRAY_LINE_NORMAL,0x18U,0.0f,5U);
  assert(crossings==1U&&running!=0U);
  puts("PASS sensor recovery, independent element timer and millisecond wrap");
}

int main(void)
{
  test_start_and_pd();
  test_corners();
  test_intersections_and_finish();
  test_faults();
  test_recovery_and_wrap();
  puts("All line tracking scenarios passed.");
  return 0;
}
