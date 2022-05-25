#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "mod_state_machine.h"
#include "mod_outputs.h"
#include "mod_comms.h"
#include "fault_codes.h"

/* Misc. module specific varables */
static uint8_t _is_init = 0;
static uint8_t _is_running = 0;
static uint8_t _start_received = 0;
static uint8_t _fault_detected = 0;

/* Structures defined for the state machine thread */
static osThreadId_t _statemachine_thread_id;
const osThreadAttr_t statemachineTask_attr = 
{
    .name = "statemachine",
    .priority = osPriorityNormal
};

modStateMachineTypeDef sStateMachineState = 
{
    .Error = FAULT_HEALTHY,
    .smState = SM_WAITING
}; //State machine structure


///////////////////////////////////// INTERFACE FUNCTIONS /////////////////////////////////////
/* Update output module error code. This is used to update the state machine */
void module_state_machine_set_fault(errorCode newError)
{
    // Save the updated error code from fault detection module
    sStateMachineState.Error = newError;
    // If an error has been detected, set the fault flag for servicing
    if(sStateMachineState.Error != FAULT_HEALTHY)
    {
        // 0 = healthy, 1= fault
        _fault_detected = 1;
    }
    else
    {
        // 0 = healthy, 1= fault
        _fault_detected = 0;
    }
}

/* Function to be called when start signal received from control room */
void module_state_machine_start_signal(void)
{
    if (sStateMachineState.smState == SM_WAITING)
    {
        _start_received = 1;
    }
}


///////////////////////////////////// THREAD FUNCTIONS /////////////////////////////////////
/* Thread to manage state machine module */
void module_state_machine_task(void *argument)
{
    UNUSED(argument);
    while(1)
    {
        // Update state of fault detection module
        module_state_machine_update();
        // Wait 100ms between updates
        osDelay(100);
    }
}

/* Module update routine */
void module_state_machine_update(void)
{
    // TODO: Implement state machine switch() here
    switch(sStateMachineState.smState)
    {
        case SM_HEALTHY:
        /* TODO: Complete this section */
        // Hint: Use the '_fault_detected' flag above to determine if a fault has been detected
            if (_fault_detected == 1) {
                module_outputs_set_state(OPEN);
                sStateMachineState.smState = SM_FAULT;
        }
        break;

        case SM_WAITING:
        /* TODO: Complete this section */
            if (_fault_detected == 1) {
                module_outputs_set_state(CLOSED);
                _start_received = 0;
                sStateMachineState.smState =SM_HEALTHY;
         }
        break;

        case SM_FAULT:
        /* TODO: Complete this section */
            if (_fault_detected == 0) {
                _start_received = 0;
                sStateMachineState.smState = SM_WAITING;
            }
        break;
        
        default:
        // should never get here
        sStateMachineState.smState = SM_FAULT;
        break;
    }
}


///////////////////////////////////// INITIALISATION FUNCTIONS /////////////////////////////////////
/* Start thread to manage fault detection module */
void module_state_machine_init(void)
{
    if (!_is_init)
    {
        // Initialise thread
        _statemachine_thread_id = osThreadNew(module_state_machine_task, NULL, &statemachineTask_attr);
        _is_init = 1;
        _is_running = 1;
    }
}


///////////////////////////////////// ADDITIONAL FUNCTIONS /////////////////////////////////////
/* start outputs module */
void module_state_machine_start(void)
{
    if(!_is_running && _is_init)
    {
        osThreadResume(_statemachine_thread_id);
        _is_running = 1;
    }
    printf("State machine module on\n");
}

/* stop outputs module */
void module_state_machine_stop(void)
{
    if(_is_running && _is_init)
    {
        osThreadSuspend(_statemachine_thread_id);
        _is_running = 0;
    }
    printf("State machine module off\n");
}

/* Retrieve output module state */
modStateMachineTypeDef* module_state_machine_get_state(void)
{
    return &sStateMachineState;
}

void module_state_machine_manual_error(int newError)
{
    // Set input fault status
    module_state_machine_set_fault(newError);
    
    // update state machine
    module_state_machine_update();
    
    // Retrieve output module state
    relayState* outputState;
    outputState = module_outputs_get_state();
    
    // Print fault detection state
    printf("State machine state: %i\n", sStateMachineState.smState);
    printf("Output module state: %i\n", *outputState);
}
