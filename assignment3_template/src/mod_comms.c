#include <stdio.h>

#include "mod_state_machine.h"
#include "mod_comms.h"
#include "cmsis_os2.h"

/* Misc. module specific varables */
static uint8_t _is_init = 0;
static uint8_t _is_running = 0;

///////////////////////////////////// INTERFACE FUNCTIONS /////////////////////////////////////
void module_comms_start_received(void)
{
    // send start signal to state machine module
    module_state_machine_start_signal();
    
    // Update the state machine module
    module_state_machine_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("State machine state: %i\n", sstateMachineState->smState);
}

void module_comms_heartbeat(void)
{
    // Read current state and error code
    modStateMachineTypeDef* smState;
    smState = module_state_machine_get_state();
    
    /* Print state to serial */
    printf("Status: State %i, Error: %i\n", smState->smState, smState->Error);
}


///////////////////////////////////// ADDITIONAL FUNCTIONS /////////////////////////////////////
/* start outputs module */
void module_comms_start(void)
{
    if(!_is_running && _is_init)
    {
        // TODO: Mabey something goes here?
        _is_running = 1;
    }
    printf("Comms module on\n");
}

/* stop outputs module */
void module_comms_stop(void)
{
    if(_is_running && _is_init)
    {
        // TODO: Mabey something goes here?
        _is_running = 0;
    }
    printf("Comms module off\n");
}