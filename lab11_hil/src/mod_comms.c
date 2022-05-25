#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "mod_state_machine.h"
#include "mod_outputs.h"

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
    
    // Retrieve output module state
    relayState* outputState;
    outputState = module_outputs_get_state();
    
    // Print fault detection state
    printf("State machine state: %i\n", sstateMachineState->smState);
    printf("Output module state: %i\n", *outputState);
}
