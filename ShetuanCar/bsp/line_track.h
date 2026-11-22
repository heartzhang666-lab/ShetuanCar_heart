/**
 * @file    line_track.h
 * @brief   灰度赛道状态机、USER 启停、20 ms 外环与 10 ms 内环接口。
 */

#ifndef LINE_TRACK_H
#define LINE_TRACK_H

/**
 * @brief  ADC、USART2、OLED 及定时器初始化后调用，上电待机并启动 TIM6。
 *
 * 轮位和方向使用硬件接线文档的已标定结果。前置八路灰度用于实时回线；
 * 当前赛道一圈需两次通过中央交叉点，起终点横线须覆盖八个探头。
 * 上电默认参数仅为联调初值，实际灰度端点、折角与停车位置需落地验证。
 */
void LineTrack_Init(void);

/**
 * @brief  主循环处理采样、曲线 PD、折角、交叉、终点保持及蓝牙命令。
 *
 * USER 按下启停；终点制动期间四轮连续 2 s 无编码器增量才结束；期间不能重新起跑，结束后需新按下。
 * 120 s、ADC 错误、丢线或元素超时均取消本次运行；不会自行恢复驱动。
 */
void LineTrack_Task(void);

// 仅从 TIM6 的 10 ms 更新中断调用，更新四轮 PI，执行使能、竞赛时限和帧超时联锁。
void LineTrack_ControlTick(void);

#endif
