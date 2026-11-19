/**
 * @file    encoder_calibration.c
 * @brief   在电机断驱状态下用 OLED 观察四个车轮的正交编码器计数。
 *
 * 文件结构：
 *   1. encoder_cal_show_count()  — 显示选中车轮的有符号原始累计计数
 *   2. encoder_cal_select_next() — 按轮位顺序切换并建立零点
 *   3. encoder_cal_update()      — 从定时器环形计数器取差并累计
 *   4. EncoderCal_Init()         — 启动四组编码器接口并等待 USER
 *   5. EncoderCal_Task()         — 处理按键、采样和定时刷新
 */

#include "encoder_calibration.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "OLED.h"
#include "tim.h"

// PE0 上拉输入，按下为低电平。时长增加能过滤更长的抖动，切轮响应也会变慢。
#define ENCODER_CAL_DEBOUNCE_MS 25U
// 只限制 OLED 的刷新频率。间隔缩短时计数显示更及时，软件串行写屏开销也会增加。
#define ENCODER_CAL_REFRESH_MS 100U
#define ENCODER_CAL_WHEEL_COUNT 4U

typedef struct
{
  TIM_HandleTypeDef *timer;
  char label[17];
} EncoderCalWheel;

// ================================================================
// 1. 轮位与正交输入
//
//    A 左后 TIM2/PA15、PB3，B 右后 TIM3/PB4、PB5。
//    C 右前 TIM4/PB6、PB7，D 左前 TIM5/PA0、PA1。
//    各通道上拉输入，TI12 对两相的上升沿和下降沿计数。
//    OLED 显示电气相序产生的原始符号，车轮顶部朝车头转动时的符号由实物决定。
// ================================================================
static EncoderCalWheel wheels[ENCODER_CAL_WHEEL_COUNT]=
{
  {&htim2,"A Left rear     "},
  {&htim3,"B Right rear    "},
  {&htim4,"C Right front   "},
  {&htim5,"D Left front    "}
};

// 选轮索引在首次按键前等于轮数，表示未选轮。选轮后跨循环保存基准、累计值和已显示的计数。
static uint8_t selected_wheel=ENCODER_CAL_WHEEL_COUNT;
static uint32_t last_counter=0U;
static int32_t total_count=0;
static int32_t shown_count=0;
static uint32_t last_refresh=0U;

// 消抖需要跨循环记住原始电平、确认后的电平以及最后一次跳变的时刻。
static GPIO_PinState raw_key=GPIO_PIN_SET;
static GPIO_PinState stable_key=GPIO_PIN_SET;
static uint32_t key_changed=0U;

static void encoder_cal_show_count(void)
{
  char text[17];

  // 符号始终可见。字段宽度固定，计数变短时覆盖上一帧留下的字符。
  (void)snprintf(text,sizeof(text),"CNT %+11ld ",(long)total_count);
  OLED_ShowString(3,1,text);
  shown_count=total_count;
  last_refresh=HAL_GetTick();
}

static void encoder_cal_select_next(void)
{
  selected_wheel++;
  if(selected_wheel>=ENCODER_CAL_WHEEL_COUNT)  selected_wheel=0U;

  // 将当前硬件计数作为新轮的零点，后续只累计相对变化。
  last_counter=__HAL_TIM_GET_COUNTER(wheels[selected_wheel].timer);
  total_count=0;
  OLED_ShowString(2,1,wheels[selected_wheel].label);
  encoder_cal_show_count();
}

// ================================================================
// 2. 处理硬件计数回绕
//
//    TIM2/5 使用 32 位满量程，TIM3/4 使用 16 位满量程。
//    按计数器位宽对前后两次计数作取模减法，将上半区解释为负增量。
//    相邻采样间的实际变化量须小于半个计数范围，主循环连续采样以覆盖手拨转速。
// ================================================================
static void encoder_cal_update(void)
{
  TIM_HandleTypeDef *timer=wheels[selected_wheel].timer;
  uint32_t counter=__HAL_TIM_GET_COUNTER(timer);
  uint32_t mask=__HAL_TIM_GET_AUTORELOAD(timer);
  uint32_t difference=(counter-last_counter)&mask;
  int32_t increment;
  int64_t sum;

  if(difference==0U)  return;
  last_counter=counter;
  increment=(difference<=(mask>>1U))?(int32_t)difference:-1-(int32_t)(mask-difference);
  sum=(int64_t)total_count+increment;

  // 超过 OLED 的有符号 32 位计数范围时饱和，防止长时间累加导致符号突变。
  if(sum>INT32_MAX)  sum=INT32_MAX;
  if(sum<INT32_MIN)  sum=INT32_MIN;
  total_count=(int32_t)sum;
}

void EncoderCal_Init(void)
{
  uint8_t i;

  // ------- 1. 启动四个编码器定时器的双通道计数 -------
  for(i=0U;i<ENCODER_CAL_WHEEL_COUNT;i++)
  {
    if(HAL_TIM_Encoder_Start(wheels[i].timer,TIM_CHANNEL_ALL)!=HAL_OK)
    {
      // 有任一路未启动时结束初始化，避免将恒零误当成有效标定结果。
      Error_Handler();
    }
  }

  // ------- 2. 建立按键基线 -------
  raw_key=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);
  stable_key=raw_key;
  key_changed=HAL_GetTick();

  // ------- 3. 显示待选轮状态 -------
  OLED_ShowString(1,1,"Encoder hand cal");
  OLED_ShowString(2,1,"Press USER      ");
  OLED_ShowString(4,1,"USER: Next/zero ");
  encoder_cal_show_count();
}

void EncoderCal_Task(void)
{
  uint32_t now=HAL_GetTick();
  GPIO_PinState level=HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin);

  // ------- 1. 确认 USER 按下边沿，按住时只切换一次 -------
  if(level!=raw_key)
  {
    raw_key=level;
    key_changed=now;
  }
  if(stable_key!=raw_key&&(now-key_changed)>=ENCODER_CAL_DEBOUNCE_MS)
  {
    stable_key=raw_key;
    if(stable_key==GPIO_PIN_RESET)
    {
      encoder_cal_select_next();
      return;
    }
  }

  // ------- 2. 对选中轮连续采样，首次按键前保持待机 -------
  if(selected_wheel>=ENCODER_CAL_WHEEL_COUNT)  return;
  encoder_cal_update();

  // ------- 3. 定时显示变动后的计数 -------
  // HAL 时间戳用无符号差值，系统节拍回绕时仍能判断相邻刷新间隔。
  if(total_count!=shown_count&&(now-last_refresh)>=ENCODER_CAL_REFRESH_MS)  encoder_cal_show_count();
}
