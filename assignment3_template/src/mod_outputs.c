#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "mod_outputs.h"
#include "mod_state_machine.h"
#include "fault_codes.h"

/* Misc. module specific varables */
static uint8_t _is_init = 0;
static uint8_t _is_running = 0;

/* state of contactor */
static relayState outputsState = OPEN;

/* Structures defined for the outputs thread */
static osThreadId_t _outputs_thread_id;
static osThreadAttr_t _outputs_attr = 
{
    .name = "outputsModule",
    .priority = osPriorityNormal //Set initial thread priority
};


///////////////////////////////////// INTERFACE FUNCTIONS /////////////////////////////////////
/* Update output module state */
void module_outputs_set_state(relayState newState)
{
    outputsState = newState;
}


///////////////////////////////////// THREAD FUNCTIONS /////////////////////////////////////
/* Thread to manage outputs module */
void module_outputs_task(void *argument)
{
    UNUSED(argument);
    while(1)
    {
        // Update state of fault detection module
        module_outputs_update();
        // Wait 100ms between updates
        osDelay(100);
    }
}

/* function to update output base on state machine module */
void module_outputs_update(void)
{
    /* TODO: Update the output pin state */
}


///////////////////////////////////// INITIALISATION FUNCTIONS /////////////////////////////////////
/* Start thread to manage fault detection module */
void module_outputs_init(void)
{
    if (!_is_init)
    {
        /* TODO: Initiaise the output pin */
        
        
        // Initialise thread
        _outputs_thread_id = osThreadNew(module_outputs_task, NULL, &_outputs_attr);
        _is_init = 1;
        _is_running = 1;
    }
}


///////////////////////////////////// ADDITIONAL FUNCTIONS /////////////////////////////////////
/* start outputs module */
void module_outputs_start(void)
{
    if(!_is_running && _is_init)
    {
        osThreadResume(_outputs_thread_id);
        _is_running = 1;
    }
    printf("Output module on\n");
}

/* stop outputs module */
void module_outputs_stop(void)
{
    if(_is_running && _is_init)
    {
        osThreadSuspend(_outputs_thread_id);
        _is_running = 0;
    }
    printf("Output module off\n");
}

/* Retrieve output module state */
relayState* module_outputs_get_state(void)
{
    return &outputsState;
}

void module_outputs_manual_state(int newState)
{
    // Set outputs state
    module_outputs_set_state(newState);
    
    // update module
    module_outputs_update();
    
    // Print the updated value
    printf("Output state: %i\n", outputsState);
}

