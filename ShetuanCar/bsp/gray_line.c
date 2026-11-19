/**
 * @file    gray_line.c
 * @brief   读取感为八路模拟灰度，计算黑线强度和带符号位置。
 *
 * 文件结构：
 *   1. GrayLine_Init()             — 准备用于通道稳定等待的周期计数器
 *   2. gray_line_read_raw()        — 反相寻址，逐路采样并取平均
 *   3. GrayLine_Read()             — 归一化并区分单线、宽线和丢线
 *   4. GrayLine_CaptureReference() — 停车采集并验证黑白端点
 */

#include "gray_line.h"
#include "adc.h"
#include "main.h"

#define GRAY_CHANNEL_COUNT 8U
// 延时增加有利于模拟开关和 PC0 稳定，但会拉长八路采样帧。
#define GRAY_SETTLE_US 15U
// 多样本平均可抑制 ADC 抖动，样本过多会增加循迹外环延迟。
#define GRAY_SAMPLES 8U
#define GRAY_ADC_TIMEOUT_MS 2U
// 复用旧车探头物理顺序。若扫线实测左右反向，改为 0 重新核对。
#define GRAY_REVERSE_ORDER 1

#define GRAY_FULL_STRENGTH 4096U
// 白底死区过小容易把背景噪声当成线，过大则会漏掉黑线边缘。
#define GRAY_BACKGROUND_STRENGTH 128U
#define GRAY_MIN_LINE_SUM 800U
#define GRAY_STRONG_THRESHOLD (GRAY_FULL_STRENGTH/2U)
#define GRAY_WIDE_PROBE_COUNT 5U
// 黑白至少相差 200 个 12 位 ADC 计数，避免归一化放大低对比通道噪声。
#define GRAY_MIN_CONTRAST 200U
#define GRAY_REFERENCE_FRAMES 8U

static const int8_t weights[GRAY_CHANNEL_COUNT]={-7,-5,-3,-1,1,3,5,7};

// 先沿用旧车端点，让首次上电可查看读数或低速试跑。停机后可用 WHITE、BLACK 覆盖整组 RAM 值。
static uint16_t white_reference[GRAY_CHANNEL_COUNT]={3106,3104,3078,3165,3130,3053,3128,3148};
static uint16_t black_reference[GRAY_CHANNEL_COUNT]={87,86,90,135,166,91,150,110};

void GrayLine_Init(void)
{
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
}

// ================================================================
// 1. 复用开关与 ADC 采样
//
//    PC1/2/3 是推挽地址输出，分别对应 AD0/AD1/AD2，反相地址 0 为 111。
//    切换地址 → 等待模拟电压稳定 → 在 PC0 的 ADC1_IN10 上读 12 位计数。
//    标定和运行采样都在主循环中调用，TIM6 中断不会等待 ADC。
// ================================================================
static uint8_t gray_line_read_raw(uint16_t raw[GRAY_CHANNEL_COUNT])
{
  uint8_t address;

  for(address=0U;address<GRAY_CHANNEL_COUNT;address++)
  {
    uint8_t index=GRAY_REVERSE_ORDER?(GRAY_CHANNEL_COUNT-1U-address):address;
    uint32_t start;
    uint32_t sum=0U;
    uint8_t sample;

    HAL_GPIO_WritePin(GRAY_AD0_GPIO_Port,GRAY_AD0_Pin,(address&1U)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(GRAY_AD1_GPIO_Port,GRAY_AD1_Pin,(address&2U)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(GRAY_AD2_GPIO_Port,GRAY_AD2_Pin,(address&4U)?GPIO_PIN_RESET:GPIO_PIN_SET);
    start=DWT->CYCCNT;
    while((DWT->CYCCNT-start)<(SystemCoreClock/1000000U)*GRAY_SETTLE_US)
    {
      __NOP();
    }
    for(sample=0U;sample<GRAY_SAMPLES;sample++)
    {
      if(HAL_ADC_Start(&hadc1)!=HAL_OK||HAL_ADC_PollForConversion(&hadc1,GRAY_ADC_TIMEOUT_MS)!=HAL_OK)
      {
        // 任一路失败都撤销本帧，杜绝把失败值 0 当成黑线。
        (void)HAL_ADC_Stop(&hadc1);
        return 0U;
      }
      sum+=HAL_ADC_GetValue(&hadc1);
      (void)HAL_ADC_Stop(&hadc1);
    }
    raw[index]=(uint16_t)(sum/GRAY_SAMPLES);
  }
  return 1U;
}

// ================================================================
// 2. 黑线位置和帧有效性
//
//    白底时强度为零，黑底时为 4096，以强度为权重求位置。
//    同时识别过宽或分段的黑色分布，避免把交叉处对称读数当作正常居中。
// ================================================================
uint8_t GrayLine_Read(GrayLineReading *reading)
{
  uint16_t raw[GRAY_CHANNEL_COUNT];
  uint32_t sum=0U;
  int32_t weighted_sum=0;
  uint8_t strong_count=0U;
  uint8_t groups=0U;
  uint8_t i;

  if(gray_line_read_raw(raw)==0U)  return 0U;
  reading->mask=0U;
  reading->error=0.0f;
  for(i=0U;i<GRAY_CHANNEL_COUNT;i++)
  {
    uint32_t strength=0U;

    if(raw[i]<=black_reference[i])  strength=GRAY_FULL_STRENGTH;
    else if(raw[i]<white_reference[i])
    {
      strength=(uint32_t)(white_reference[i]-raw[i])*GRAY_FULL_STRENGTH/
                (uint32_t)(white_reference[i]-black_reference[i]);
    }
    if(strength<GRAY_BACKGROUND_STRENGTH)  strength=0U;
    if(strength>=GRAY_STRONG_THRESHOLD)
    {
      if(i==0U||(reading->mask&(1U<<(i-1U)))==0U)  groups++;
      reading->mask|=(uint8_t)(1U<<i);
      strong_count++;
    }
    sum+=strength;
    weighted_sum+=(int32_t)strength*weights[i];
  }
  if(sum<GRAY_MIN_LINE_SUM)
  {
    reading->state=GRAY_LINE_LOST;
    return 1U;
  }
  reading->error=(float)weighted_sum/(float)sum;
  reading->state=(strong_count>=GRAY_WIDE_PROBE_COUNT||groups>1U)?GRAY_LINE_WIDE:GRAY_LINE_NORMAL;
  return 1U;
}

uint8_t GrayLine_CaptureReference(uint8_t black)
{
  uint16_t raw[GRAY_CHANNEL_COUNT];
  uint32_t sums[GRAY_CHANNEL_COUNT]={0U};
  uint16_t reference[GRAY_CHANNEL_COUNT];
  uint8_t frame;
  uint8_t i;

  for(frame=0U;frame<GRAY_REFERENCE_FRAMES;frame++)
  {
    if(gray_line_read_raw(raw)==0U)  return 0U;
    for(i=0U;i<GRAY_CHANNEL_COUNT;i++)  sums[i]+=raw[i];
  }
  for(i=0U;i<GRAY_CHANNEL_COUNT;i++)
  {
    int32_t contrast;

    reference[i]=(uint16_t)(sums[i]/GRAY_REFERENCE_FRAMES);
    contrast=black?(int32_t)white_reference[i]-reference[i]:(int32_t)reference[i]-black_reference[i];
    // 参考物需覆盖八个探头。任一路对比度不足时保留上一组端点。
    if(contrast<(int32_t)GRAY_MIN_CONTRAST)  return 0U;
  }
  for(i=0U;i<GRAY_CHANNEL_COUNT;i++)
  {
    if(black!=0U)  black_reference[i]=reference[i];
    else  white_reference[i]=reference[i];
  }
  return 1U;
}
