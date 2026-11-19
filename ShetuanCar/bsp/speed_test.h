/**
 * @file    speed_test.h
 * @brief   使用 USER 和 OLED 逐轮观察五档速度响应的台架测试接口。
 */

#ifndef SPEED_TEST_H
#define SPEED_TEST_H

// 初始化四路驱动和编码器，再启动 TIM6 的 10 ms 更新中断。
void SpeedTest_Init(void);

// 主循环调用，处理 USER、使能状态和 OLED 刷新。
void SpeedTest_Task(void);

// 仅从 TIM6 更新中断调用，采样编码器并更新本轮驱动。
void SpeedTest_ControlTick(void);

#endif
