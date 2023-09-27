#include "interrupt_handling.h"

#include "main_fsm.h"
#include "main.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim == &htim2) {
		//executes every 50ms
		static uint32_t timer_period_counter = 0;

		static const uint32_t timer_period_ms = 50;
		static const uint32_t sys_voltage_period_ms = 350;
		static const uint32_t sensor_period_ms = 200;

		static const uint32_t timer_periods_to_wait_before_checking_sys_voltage = sys_voltage_period_ms / timer_period_ms;
		static const uint32_t timer_periods_to_wait_before_checking_sensor = sensor_period_ms / timer_period_ms;

		if(timer_period_counter % timer_periods_to_wait_before_checking_sys_voltage == 0)
			main_fsm.inputs.should_read_voltage = true;
		if(timer_period_counter % timer_periods_to_wait_before_checking_sensor == 0)
			main_fsm.inputs.should_read_sensor = true;

		timer_period_counter++;
	}
}

// callback for button push
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == B1_Pin) {
		//toggle the pause state of the system
		main_fsm.inputs.button_pressed = true;
	}
}
