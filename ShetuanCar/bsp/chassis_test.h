/**
 * @file    chassis_test.h
 * @brief   俯视 X 形麦轮底盘的 USER 按键动作测试接口。
 */

#ifndef CHASSIS_TEST_H
#define CHASSIS_TEST_H

// 在 GPIO、PWM 定时器和 OLED 初始化后调用。电机先保持停机。
void ChassisTest_Init(void);

// 主循环连续调用。切换动作、检查使能并在限定时间后停止驱动。
void ChassisTest_Task(void);

#endif
