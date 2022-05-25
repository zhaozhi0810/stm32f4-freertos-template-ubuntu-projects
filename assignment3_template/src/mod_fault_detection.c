#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "mod_fault_detection.h"
#include "mod_state_machine.h"
#include "fault_codes.h"
#include "ref_r410a_props.h"

/* Misc. module specific varables */
static uint8_t _is_init = 0;
static uint8_t _is_running = 0;

/* Structure to store values pushed from inputs module */
static modFaultDetecTypeDef sFaultDetecState;

/* Structures defined for the fault detection thread */
static osThreadId_t _fault_detect_thread_id;
static osThreadAttr_t _fault_detect_attr = 
{
    .name = "faultDetectModule",
    .priority = osPriorityNormal //Set initial thread priority
};

///////////////////////////////////// INTERFACE FUNCTIONS /////////////////////////////////////
/* Update the state of the estop */
void module_fault_detect_set_estop(estopState newEstop)
{
    // 0= healthy, 1 = fault
    sFaultDetecState.Estop = newEstop;
}

/* Update pressure 1 */
void module_fault_detect_set_pressure1(float newPressure)
{
    sFaultDetecState.pressure1 = newPressure;
}

/* Update the tank temperature */
void module_fault_detect_set_temperature1(float newTemperature)
{
    sFaultDetecState.temperature1 = newTemperature;
}

/* Update the tank temperature */
void module_fault_detect_set_temperature2(float newTemperature)
{
    sFaultDetecState.temperature2 = newTemperature;
}

/* Update the tank temperature */
void module_fault_detect_set_temperature3(float newTemperature)
{
    sFaultDetecState.temperature3 = newTemperature;
}

/* Update the hardware error codes */
/* Note: This function is to be used for hardware error detection only */
void module_fault_detect_set_error(errorCode newError)
{
    sFaultDetecState.inputError = newError;
}


///////////////////////////////////// THREAD FUNCTIONS /////////////////////////////////////
/* Thread to manage fault detection module */
void module_fault_detect_task(void *argument)
{
    UNUSED(argument);
    while(1)
    {
        // Update state of fault detection module
        module_fault_detect_update();
        // Wait 500ms between updates
        osDelay(500);
    }
}

/* Module update routine */
void module_fault_detect_update(void)
{
    // TODO: Check for errors and push appropriate error codes to the state machine module 
}

///////////////////////////////////// INITIALISATION FUNCTIONS /////////////////////////////////////
/* Start thread to manage fault detection module */
void module_fault_detect_init(void)
{
    if (!_is_init)
    {
        // Initialise thread
        _fault_detect_thread_id = osThreadNew(module_fault_detect_task, NULL, &_fault_detect_attr);
        _is_init = 1;
        _is_running = 1;
    }
}


///////////////////////////////////// ADDITIONAL FUNCTIONS /////////////////////////////////////
/* start fault detection module */
void module_fault_detect_start(void)
{
    if(!_is_running && _is_init)
    {
        osThreadResume(_fault_detect_thread_id);
        _is_running = 1;
    }
    printf("Fault detection module on\n");
}

/* stop fault detection module */
void module_fault_detect_stop(void)
{
    if(_is_running && _is_init)
    {
        osThreadSuspend(_fault_detect_thread_id);
        _is_running = 0;
    }
    printf("Fault detection module off\n");
}

/* Function to read pointer to internal state structure */
modFaultDetecTypeDef* module_fault_detect_get_state(void)
{
    return &sFaultDetecState;
}

void module_fault_detect_manual_estop(int newEstop)
{
    // Set estop value
    module_fault_detect_set_estop(newEstop);
    
    // Update the module
    module_fault_detect_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("Fault detection module output error code: %i\n", sstateMachineState->Error);
}

void module_fault_detect_manual_error(int newError)
{
    // Set estop value
    module_fault_detect_set_error(newError);
    
    // Update the module
    module_fault_detect_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("Fault detection module output error code: %i\n", sstateMachineState->Error);
}

void module_fault_detect_manual_pressure1(float newPressure)
{
    // Set estop value
    module_fault_detect_set_pressure1(newPressure);
    
    // Update the module
    module_fault_detect_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("Fault detection module output error code: %i\n", sstateMachineState->Error);
}

void module_fault_detect_manual_temperature1(float newTemperature)
{
    // Set estop value
    module_fault_detect_set_temperature1(newTemperature);
    
    // Update the module
    module_fault_detect_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("Fault detection module output error code: %i\n", sstateMachineState->Error);
}

void module_fault_detect_manual_temperature2(float newTemperature)
{
    // Set estop value
    module_fault_detect_set_temperature2(newTemperature);
    
    // Update the module
    module_fault_detect_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("Fault detection module output error code: %i\n", sstateMachineState->Error);
}

void module_fault_detect_manual_temperature3(float newTemperature)
{
    // Set estop value
    module_fault_detect_set_temperature3(newTemperature);
    
    // Update the module
    module_fault_detect_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("Fault detection module output error code: %i\n", sstateMachineState->Error);
}

void module_fault_detect_print_inputs(void)
{
    // Print input values to console
    printf("Estop status (0=healthy, 1=fault): %i\n", sFaultDetecState.Estop);
    printf("Pressure 1 status (kPa): %f\n", sFaultDetecState.pressure1);
    printf("Temperature 1 status (C): %f\n", sFaultDetecState.temperature1);
    printf("Temperature 2 status (C): %f\n", sFaultDetecState.temperature2);
    printf("Temperature 3 status (C): %f\n", sFaultDetecState.temperature3);
    printf("Error code: %i\n", sFaultDetecState.inputError);
}
