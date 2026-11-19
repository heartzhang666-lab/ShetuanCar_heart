/**
 * @file    motor_drive.h
 * @brief   C30D 四路 AT8236 慢衰减驱动接口。
 */

#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

#define MOTOR_DRIVE_WHEEL_COUNT 4U

// 四个 PWM 定时器配置完成后启动八个通道，初始比较值均为 0。
void MotorDrive_Init(void);

/**
 * @brief  从滑行输出切换到一组短时试转命令。
 * @param  duty 四路按 A、B、C、D 排列，绝对值为有效占空比百分数，超出 100% 时限幅。
 *
 * 正值对应 IN1 恒高，负值对应 IN2 恒高，0 使该轮滑行。调用方负责检查使能和换向间隔。
 */
void MotorDrive_RunRaw(const int8_t duty[MOTOR_DRIVE_WHEEL_COUNT]);

// 清除四路电机输出。PWM 初始化前或进入 Error_Handler 时也可以调用。
void MotorDrive_StopOutputs(void);

#endif
