/**
 * @file    motor_calibration.h
 * @brief   四轮电机方向标定的初始化、主循环和异常关断接口。
 */

#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

// 在 GPIO、四个 PWM 定时器及 OLED 初始化后调用。上电后保持停机，等待 USER 按键。
void MotorCal_Init(void);

// 在主循环连续调用，处理按键、使能开关和八段短时试转。
void MotorCal_Task(void);

// 撤去四只电机的驱动，可在 PWM 尚未启动或应用进入错误处理时调用。
void MotorCal_StopOutputs(void);

#endif
