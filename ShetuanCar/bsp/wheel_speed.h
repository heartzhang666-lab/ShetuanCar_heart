/**
 * @file    wheel_speed.h
 * @brief   单轮增量式速度环的数据结构与 10 ms 更新接口。
 */

#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>

typedef struct
{
  uint32_t last_counter;  // 保存上一次硬件计数，供下一周期取有符号差值。
  float last_error;       // e[k-1]，用于本周期 P、D 增量。
  float older_error;      // e[k-2]，用于 D 项的二阶差分。
  float output;           // 跨周期累加的有效驱动量，范围 0～40000。
} WheelSpeedController;

// 编码器和 PWM 定时器初始化后调用，启动四路计数并建立零电机输出。
void WheelSpeed_Init(void);

/**
 * @brief  为一次新的单轮试验读取计数基线、清除增量 PID 历史。
 * @param  wheel 按 A、B、C、D 排列的轮位下标，范围 0～3。
 * @param  controller 该轮使用的速度环状态。
 *
 * 停机换轮后应先重置，连续调整同一轮的目标时保留状态。
 */
void WheelSpeed_Reset(uint8_t wheel,WheelSpeedController *controller);

/**
 * @brief  仅采样轮速并推进计数基准，不更新 PI 历史，也不写 PWM。
 * @param  wheel 按 A、B、C、D 排列，范围 0～3。
 * @param  controller 已经 Reset 的该轮状态；不可让多个调用者在同一拍重复推进基准。
 * @return 前进为正的轮速，单位为计数/采样周期，规定调用周期为 10 ms。
 *
 * 终点制动时由 TIM6 使用，零增量表示该周期没有可测的轮子转动。
 * WheelSpeed_Update() 内部已经调用本函数，正常调速时不要额外读取同一状态。
 */
int32_t WheelSpeed_Read(uint8_t wheel,WheelSpeedController *controller);

/**
 * @brief  在固定 10 ms 节拍更新一次所选车轮的前向速度环。
 * @param  wheel 与本轮状态绑定的轮位下标，范围 0～3。
 * @param  target 大于零的前进目标，单位为计数/10 ms。
 * @param  controller 使用 WheelSpeed_Reset() 建立基线后的状态。
 * @return 实测有符号轮速，单位为计数/10 ms，前进为正。
 *
 * 调用方负责电机使能检查和换轮时撤去输出。
 */
int32_t WheelSpeed_Update(uint8_t wheel,uint16_t target,WheelSpeedController *controller);

#endif
