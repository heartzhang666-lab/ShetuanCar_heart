/**
 * @file    line_track.h
 * @brief   USER 启停循迹、20 ms 灰度外环和 10 ms 速度内环接口。
 */

#ifndef LINE_TRACK_H
#define LINE_TRACK_H

// ADC、USART2、OLED 及定时器初始化后调用，上电待机。
void LineTrack_Init(void);

// 主循环处理 USER、传感器、PD、蓝牙命令和 OLED 显示。
void LineTrack_Task(void);

// 仅从 TIM6 更新中断调用，读取轮速目标并更新四路速度环。
void LineTrack_ControlTick(void);

#endif
