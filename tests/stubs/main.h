/** @file main.h @brief 主机状态机测试使用的最小 HAL 替身，不进入 Keil 工程。 */
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef enum {GPIO_PIN_RESET=0,GPIO_PIN_SET=1} GPIO_PinState;
#define USER_KEY_GPIO_Port 1U
#define USER_KEY_Pin 1U
#define MOTOR_ENABLE_GPIO_Port 2U
#define MOTOR_ENABLE_Pin 2U
#define HAL_OK 0
uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(uint32_t port,uint32_t pin);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t state);
void Error_Handler(void);
#endif
