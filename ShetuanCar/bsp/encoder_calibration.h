/**
 * @file    encoder_calibration.h
 * @brief   手拨单轮编码器标定的初始化和主循环接口。
 */

#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

// 在编码器定时器、USER 按键和 OLED 初始化后调用。等待按键选中第一只轮子。
void EncoderCal_Init(void);

// 在主循环调用，处理切轮、清零并显示当前轮的有符号原始累计计数。
void EncoderCal_Task(void);

#endif
