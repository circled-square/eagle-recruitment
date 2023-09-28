#include "voltage_fsm.h"
#include "main.h"
#include <stdio.h>

voltage_fsm_t voltage_fsm = {
	.state = VOLTAGE_FSM_STATE_OK_VOLTAGE,
	.inputs = {
		.sys_voltage_mV = 0.f
	}
};

voltage_fsm_state_t run_voltage_state(voltage_fsm_state_t s, voltage_fsm_inputs_t* inputs) {
	static const uint16_t low_threshold_mV = 1800, high_threshold_mV = 2700;

	voltage_fsm_state_t next_state =
		inputs->sys_voltage_mV > high_threshold_mV ? VOLTAGE_FSM_STATE_OVERVOLTAGE
		: inputs->sys_voltage_mV < low_threshold_mV ? VOLTAGE_FSM_STATE_UNDERVOLTAGE
		: VOLTAGE_FSM_STATE_OK_VOLTAGE;

	uint32_t time_ms = HAL_GetTick();
	UART_printf("[%lu] Voltage: %u mV -> %s", time_ms, inputs->sys_voltage_mV,
			next_state == VOLTAGE_FSM_STATE_OVERVOLTAGE ? "overvoltage"
			: next_state == VOLTAGE_FSM_STATE_UNDERVOLTAGE ? "undervoltage" : "ok"
	);

	if(s != next_state)
		transition_between_voltage_states(s, next_state);
	return next_state;
}

void transition_between_voltage_states(voltage_fsm_state_t beg, voltage_fsm_state_t end) {
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
		UART_printf("invalid voltage fsm state beg=\"%d\" passed to transition_between_voltage_states", beg);
		Error_Handler();
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
		UART_printf("invalid voltage fsm state end=\"%d\" passed to transition_between_voltage_states", end);
		Error_Handler();
	}
}
