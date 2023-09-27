#ifndef INC_INTERRUPT_HANDLING_H_
#define INC_INTERRUPT_HANDLING_H_

#include "stm32f4xx_hal.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_INTERRUPT_HANDLING_H_ */
