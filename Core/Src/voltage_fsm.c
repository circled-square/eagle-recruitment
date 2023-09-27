#include "voltage_fsm.h"
#include "main.h" // pin definitions
#include <stdio.h>

voltage_fsm_t voltage_fsm = { .state = VOLTAGE_FSM_STATE_OK_VOLTAGE, .inputs = { .sys_voltage_mV = 0.f } };

voltage_fsm_state_t run_voltage_state(voltage_fsm_state_t s, voltage_fsm_inputs_t* inputs) {
	static const uint16_t low_threshold_mV = 1800, high_threshold_mV = 2700;

	voltage_fsm_state_t next_state =
			inputs->sys_voltage_mV > high_threshold_mV ? VOLTAGE_FSM_STATE_OVERVOLTAGE
			: inputs->sys_voltage_mV < low_threshold_mV ? VOLTAGE_FSM_STATE_UNDERVOLTAGE
			: VOLTAGE_FSM_STATE_OK_VOLTAGE;

	transition_between_voltage_states(s, next_state);
	return next_state;
}

void transition_between_voltage_states(voltage_fsm_state_t beg, voltage_fsm_state_t end) {
	// only when transitioning  between different states it is necessary to set/reset pins
	if(beg == end)
		return;

	switch(beg) {
	case VOLTAGE_FSM_STATE_OVERVOLTAGE:
		HAL_GPIO_WritePin(GPIOB, OVERVOLTAGE_LED_Pin, GPIO_PIN_RESET);
		break;
	case VOLTAGE_FSM_STATE_UNDERVOLTAGE:
		HAL_GPIO_WritePin(GPIOB, UNDERVOLTAGE_LED_Pin, GPIO_PIN_RESET);
		break;
	case VOLTAGE_FSM_STATE_OK_VOLTAGE:
		break;
	default:
		char buf[128];
		snprintf(buf, 128, "invalid voltage fsm state beg=\"%d\" passed to transition_between_voltage_states", beg);
		UART_println(buf);
		while(1);
	}

	switch(end) {
	case VOLTAGE_FSM_STATE_OVERVOLTAGE:
		HAL_GPIO_WritePin(GPIOB, OVERVOLTAGE_LED_Pin, GPIO_PIN_SET);
		break;
	case VOLTAGE_FSM_STATE_UNDERVOLTAGE:
		HAL_GPIO_WritePin(GPIOB, UNDERVOLTAGE_LED_Pin, GPIO_PIN_SET);
		break;
	case VOLTAGE_FSM_STATE_OK_VOLTAGE:
		break;
	default:
		char buf[128];
		snprintf(buf, 128, "invalid voltage fsm state end=\"%d\" passed to transition_between_voltage_states", end);
		UART_println(buf);
		while(1);
	}
}
