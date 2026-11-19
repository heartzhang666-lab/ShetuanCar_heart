/**
 * @file    gray_line.h
 * @brief   八路灰度采样、黑白参考值和黑线位置接口。
 */

#ifndef GRAY_LINE_H
#define GRAY_LINE_H

#include <stdint.h>

typedef enum
{
  GRAY_LINE_LOST,
  GRAY_LINE_NORMAL,
  GRAY_LINE_WIDE
} GrayLineState;

typedef struct
{
  float error;          // 探头权重为 -7～+7，黑线偏右为正。
  uint8_t mask;         // bit0 为最左探头，bit7 为最右探头，置位表示强黑线。
  GrayLineState state;
} GrayLineReading;

// 启用 Cortex-M4 周期计数器，用于地址切换后的微秒级稳定等待。
void GrayLine_Init(void);

// 主循环读取完整的八路采样帧。返回 0 表示 ADC 失败，结果作废。
uint8_t GrayLine_Read(GrayLineReading *reading);

/**
 * @brief  停车时采集一组覆盖全部八个探头的黑底或白底参考值。
 * @param  black 1 采黑底，0 采白底。
 * @return 1 表示全部通道的参考值有效并已更新，0 表示采样失败或黑白对比不足。
 */
uint8_t GrayLine_CaptureReference(uint8_t black);

#endif
