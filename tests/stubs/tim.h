/** @file tim.h @brief 主机测试仅模拟 TIM6 的启动与清标志。 */
#ifndef TEST_TIM_H
#define TEST_TIM_H
#include <stdint.h>
typedef struct {uint32_t counter;uint32_t autoreload;} TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim2,htim3,htim4,htim5,htim6;
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->counter)
#define __HAL_TIM_GET_AUTORELOAD(timer) ((timer)->autoreload)
#define TIM_CHANNEL_ALL 0xFFFFU
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer,uint32_t channels);
#define TIM_FLAG_UPDATE 1U
#define __HAL_TIM_CLEAR_FLAG(timer,flag) ((void)(timer),(void)(flag))
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
#endif
