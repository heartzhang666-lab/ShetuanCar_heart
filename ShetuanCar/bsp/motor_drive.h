/**
 * @file    motor_drive.h
 * @brief   C30D 四路 AT8236 慢衰减驱动接口。
 */

#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

#define MOTOR_DRIVE_WHEEL_COUNT 4U
#define MOTOR_DRIVE_COMMAND_MAX 40000U

// A 至 D 的前进电气极性，来自接线文档的实物标定结果。
extern const int8_t MotorDrive_ForwardDirection[MOTOR_DRIVE_WHEEL_COUNT];

// 四个 PWM 定时器配置完成后启动八个通道，初始比较值均为 0。
void MotorDrive_Init(void);

/**
 * @brief  从滑行输出切换到一组短时试转命令。
 * @param  duty 四路按 A、B、C、D 排列，绝对值为有效占空比百分数，超出 100% 时限幅。
 *
 * 正值对应 IN1 恒高，负值对应 IN2 恒高，0 使该轮滑行。调用方负责检查使能和换向间隔。
 */
void MotorDrive_RunRaw(const int8_t duty[MOTOR_DRIVE_WHEEL_COUNT]);

/**
 * @brief  连续调节一个车轮的慢衰减驱动，调速时保持 PWM 计数器运行。
 * @param  wheel 按 A 至 D 排列的车轮下标，范围 0～3。
 * @param  direction 原始电气极性，+1 选择 IN1 恒高，-1 选择 IN2 恒高。
 * @param  command 有效驱动量，范围 0～40000。0 对应该轮慢衰减制动。
 *
 * 调用前须初始化 PWM。换向前需先停机，直接反向请求会撤去电机输出。
 */
void MotorDrive_SetWheelDuty(uint8_t wheel,int8_t direction,uint16_t command);

// 清除四路电机输出。PWM 初始化前或进入 Error_Handler 时也可以调用。
void MotorDrive_StopOutputs(void);

#endif
