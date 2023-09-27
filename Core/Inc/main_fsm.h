#ifndef INC_MAIN_FSM_H_
#define INC_MAIN_FSM_H_

#include <stdbool.h>

typedef enum {
	FSM_STATE_INITIAL,
	FSM_STATE_READ_VOLTAGE, //this state is implemented as a separate state machine: see voltage_sfm.h/c
	FSM_STATE_READ_SENSOR,
	FSM_STATE_PAUSED
} fsm_state_t;

typedef struct {
	bool button_pressed;
	bool should_read_sensor;
	bool should_read_voltage;
} fsm_inputs_t;


typedef struct {
	fsm_state_t state;
	fsm_inputs_t inputs;
} main_fsm_t;


//since this fsm deals with globally defined variables it makes sense to define a single global instance
extern main_fsm_t main_fsm;

fsm_state_t run_state_initial(fsm_inputs_t* inputs);
fsm_state_t run_state_read_voltage(fsm_inputs_t* inputs);//this state is implemented as a separate state machine: see voltage_sfm.h/c
fsm_state_t run_state_read_sensor(fsm_inputs_t* inputs);
fsm_state_t run_state_paused(fsm_inputs_t* inputs);
fsm_state_t run_state(fsm_state_t s, fsm_inputs_t* inputs);

#endif /* INC_MAIN_FSM_H_ */
