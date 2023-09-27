#include "main_fsm.h"

#include "main.h" // for reading hall sensor and voltage
#include "voltage_fsm.h"
#include "sensor_reading.h"
#include <stdint.h>
#include <stdio.h>


main_fsm_t main_fsm = {
	.state = FSM_STATE_INITIAL,
	.inputs = {
		.button_pressed = false,
		.should_read_sensor = false,
		.should_read_voltage = false
	}
};


fsm_state_t run_state_initial(fsm_inputs_t* inputs) {
	if(inputs->button_pressed) {
		inputs->button_pressed = false;
		return FSM_STATE_PAUSED;
	} else if(inputs->should_read_voltage) {
		inputs->should_read_voltage = false;
		return FSM_STATE_READ_VOLTAGE;
	} else if(inputs->should_read_sensor) {
		inputs->should_read_sensor = false;
		return FSM_STATE_READ_SENSOR;
	} else {
		return FSM_STATE_INITIAL;
	}
}

//this state is implemented as a separate state machine: see voltage_sfm.h/c
fsm_state_t run_state_read_voltage(fsm_inputs_t* inputs) {
	voltage_fsm.inputs.sys_voltage_mV = read_sys_voltage_mV();
	voltage_fsm.state = run_voltage_state(voltage_fsm.state, &voltage_fsm.inputs);

	return FSM_STATE_INITIAL;
}

fsm_state_t run_state_read_sensor(fsm_inputs_t* inputs) {
	int16_t magnetic_field_Gs = read_hall_sensor_Gs();

	char buf[128];

	if (magnetic_field_Gs < -1000) {
		snprintf(buf, 128, "Hall sensor reading was out of the expected range (%d Gs < -1000Gs)", magnetic_field_Gs);
	} else if (magnetic_field_Gs > 1000) {
		snprintf(buf, 128, "Hall sensor reading was out of the expected range (%d Gs > 1000Gs)", magnetic_field_Gs);
	} else {
		//the value read by the sensor is within the expected range
		snprintf(buf, 128, "Hall sensor  %d Gs", magnetic_field_Gs);
	}

	UART_println(buf);

	return FSM_STATE_INITIAL;
}

fsm_state_t run_state_paused(fsm_inputs_t* inputs) {
	UART_println("Board in waiting state - please press the emergency button");
	HAL_Delay(500);

	if(inputs->button_pressed) {
		inputs->button_pressed = false;
		// reset timer inputs, since they definitely elapsed while we were busy waiting and we didn't react, so
		// we should wait for them to elapse again to avoid reading more often than every 200-350ms
		inputs->should_read_sensor = inputs->should_read_voltage = false;
		return FSM_STATE_INITIAL;
	} else {
		return FSM_STATE_PAUSED;
	}
}

fsm_state_t run_state(fsm_state_t s, fsm_inputs_t* inputs) {
	switch(s) {
	case FSM_STATE_INITIAL:
		return run_state_initial(inputs);
	case FSM_STATE_READ_VOLTAGE:
		return run_state_read_voltage(inputs);
	case FSM_STATE_READ_SENSOR:
		return run_state_read_sensor(inputs);
	case FSM_STATE_PAUSED:
		return run_state_paused(inputs);
	default:
		char buf[128];
		snprintf(buf, 128, "invalid fsm state \"%d\" passed to run_state", s);
		UART_println(buf);
		while(1);
	}
}
