#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "mod_inputs.h"
#include "mod_fault_detection.h"

/* Misc. module specific varables */
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfigADC;
static uint8_t _is_init = 0;
static uint8_t _is_running = 0;

/* Structures defined for the fault detection thread */
static osThreadId_t _inputs_thread_id;
static osThreadAttr_t _inputs_attr = 
{
    .name = "inputsModule",
    .priority = osPriorityHigh //Set initial thread priority
};

///////////////////////////////////// THREAD FUNCTIONS /////////////////////////////////////
/* Task to manage fault detection module */
void module_inputs_task(void *argument)
{
    UNUSED(argument);
    while(1)
    {
        // Update state of fault detection module
        module_inputs_update();
        // Wait 500ms between updates
        osDelay(500);
    }
}

/* Update sensor measuments and push to fault detection module */
void module_inputs_update(void)
{
    // create local error flag to track faults
    errorCode localErrorFlag = FAULT_HEALTHY;
    
    // TODO: Update your sensor measurments and push them to the fault detection module
}


///////////////////////////////////// INITIALISATION FUNCTIONS /////////////////////////////////////
/* initialise inputs module as a thread */
void module_inputs_init(void)
{
    // initialise hardware
    module_inputs_configure_hardware();
    // create thread for kernel to manage
    _inputs_thread_id = osThreadNew(module_inputs_task, NULL, &_inputs_attr);
    _is_init = 1;
    _is_running = 1;
}

/* configure hardware for inputs module */
void module_inputs_configure_hardware(void)
{
    // TODO: Initialise inputs related hardware here
}


///////////////////////////////////// ADDITIONAL FUNCTIONS /////////////////////////////////////
/* start inputs module */
void module_inputs_start(void)
{
    if(!_is_running && _is_init)
    {
        osThreadResume(_inputs_thread_id);
        _is_running = 1;
    }
    printf("Inputs module on\n");
}

/* stop inputs module */
void module_inputs_stop(void)
{
    if(_is_running && _is_init)
    {
        osThreadSuspend(_inputs_thread_id);
        _is_running = 0;
    }
    printf("Inputs module off\n");
}
