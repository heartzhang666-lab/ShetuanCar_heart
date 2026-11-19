/**
 * @file    line_tuning.h
 * @brief   蓝牙串口循迹参数和主循环命令接口。
 */

#ifndef LINE_TUNING_H
#define LINE_TUNING_H

#include <stdint.h>

typedef struct
{
  float kp;  // 位置误差到轮速纠偏量的比例系数。
  float kd;  // 相邻 20 ms 帧误差差值到轮速纠偏量的系数。
} LineTuningGains;

// USART2 配置完成后调用，启动蓝牙异步接收。
void LineTuning_Init(void);

/**
 * @brief  主循环处理 P、D、GET 及黑白标定命令，响应通过串口中断发送。
 * @param  running 非零表示正在循迹或电机仍有驱动，此时拒绝改动黑白参考值。
 */
void LineTuning_Task(uint8_t running);

// 主循环读取生效的参数。上电默认 P=0.8、D=0，串口设置只保存在 RAM。
LineTuningGains LineTuning_GetGains(void);

#endif
