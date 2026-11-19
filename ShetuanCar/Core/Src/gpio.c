/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  // 切换到输出模式前预置电平：PD11 为数据态、PD12 释放复位，
  // PD13 数据和 PD14 时钟均保持低电平，避免初始化 GPIO 时产生采样沿。
  HAL_GPIO_WritePin(GPIOD,OLED_DC_Pin|OLED_RST_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD,OLED_SDA_Pin|OLED_SCL_Pin,GPIO_PIN_RESET);

  // OLED 四根信号均由 MCU 驱动，采用低速推挽输出，无内部上下拉。
  GPIO_InitStruct.Pin=OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull=GPIO_NOPULL;
  GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD,&GPIO_InitStruct);

  // PD3 使能开关沿用板卡上拉输入，高电平允许运行，低电平撤去驱动。
  GPIO_InitStruct.Pin=MOTOR_ENABLE_Pin;
  GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull=GPIO_PULLUP;
  HAL_GPIO_Init(MOTOR_ENABLE_GPIO_Port,&GPIO_InitStruct);

  // PE0 是 USER 按键，上拉输入，按下接地。标定任务对低有效边沿消抖。
  GPIO_InitStruct.Pin=USER_KEY_Pin;
  HAL_GPIO_Init(USER_KEY_GPIO_Port,&GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
