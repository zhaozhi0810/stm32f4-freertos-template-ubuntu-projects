#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "mod_fault_detection.h"
#include "mod_state_machine.h"
#include "fault_codes.h"

/* Misc. module specific varables */
float maxPressure = 400;
float maxTemperature = 60;
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

/* Update the tank pressure */
void module_fault_detect_set_pressure(float newPressure)
{
    sFaultDetecState.pressure = newPressure;
}

/* Update the tank temperature */
void module_fault_detect_set_temperature(float newTemperature)
{
    sFaultDetecState.temperature = newTemperature;
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
    /* TODO: Implement fault detection logic here. Use 'module_state_machine_set_fault()' to send fault information to the state machine module */
    /* Note: The max temperature and max pressure are defined at the top of the scrip as 'maxTemperature' and 'maxPressure' */
    if (sFaultDetecState.pressure > maxPressure) {
        module_state_machine_set_fault(FAULT_PRESSURE_OVER);
    }
    else if (sFaultDetecState.temperature > maxTemperature) {
        module_state_machine_set_fault(FAULT_TEMPERATURE_OVER);
    }
    else if (sFaultDetecState.Estop == 1) {
        module_state_machine_set_fault(FAULT_ESTOP);
    }
    else if (sFaultDetecState.inputError != 0) {
        module_state_machine_set_fault(sFaultDetecState.inputError);
    }
    else {
        module_state_machine_set_fault(FAULT_HEALTHY);
    }
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

void module_fault_detect_manual_pressure(float newPressure)
{
    // Set estop value
    module_fault_detect_set_pressure(newPressure);
    
    // Update the module
    module_fault_detect_update();
    
    // Retrieve state machine state
    modStateMachineTypeDef* sstateMachineState;
    sstateMachineState = module_state_machine_get_state();
    
    // Print fault detection state
    printf("Fault detection module output error code: %i\n", sstateMachineState->Error);
}

void module_fault_detect_manual_temperature(float newTemperature)
{
    // Set estop value
    module_fault_detect_set_temperature(newTemperature);
    
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
    printf("Pressure status (kPa): %f\n", sFaultDetecState.pressure);
    printf("Temperature status (C): %f\n", sFaultDetecState.temperature);
    printf("Error code: %i\n", sFaultDetecState.inputError);
}
