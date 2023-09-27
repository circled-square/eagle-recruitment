#ifndef INC_VOLTAGE_FSM_H_
#define INC_VOLTAGE_FSM_H_

#include <stdint.h>

typedef enum {
	VOLTAGE_FSM_STATE_UNDERVOLTAGE = -1,
	VOLTAGE_FSM_STATE_OK_VOLTAGE = 0,
	VOLTAGE_FSM_STATE_OVERVOLTAGE = 1
} voltage_fsm_state_t;

typedef struct {
	uint16_t sys_voltage_mV;
} voltage_fsm_inputs_t;

typedef struct {
	voltage_fsm_state_t state;
	voltage_fsm_inputs_t inputs;
} voltage_fsm_t;


//since this fsm deals with globally defined variables it makes sense to define a single global instance
extern voltage_fsm_t voltage_fsm;

voltage_fsm_state_t run_voltage_state(voltage_fsm_state_t s, voltage_fsm_inputs_t* inputs);
void transition_between_voltage_states(voltage_fsm_state_t beg, voltage_fsm_state_t end);
#endif /* INC_VOLTAGE_FSM_H_ */
