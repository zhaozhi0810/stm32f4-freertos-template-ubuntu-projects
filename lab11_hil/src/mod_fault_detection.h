#ifndef MOD_FAULT_DETECTION_H
#define MOD_FAULT_DETECTION_H

#include <stdint.h>

#include "fault_codes.h"

typedef enum{
    HEALTHY = 0,
    FAULT,          // FAULT will be == 1
} estopState;

typedef struct
{
    errorCode   inputError;     // Hardware errors detected by the inputs module
    float       pressure;       // Temperature as read from the inputs module
    float       temperature;    // Pressure as read from the inputs module
    estopState  Estop;          // State of the estop (HEALTHY, FAULT)
} modFaultDetecTypeDef;

void module_fault_detect_update(void);
void module_fault_detect_init(void);
void module_fault_detect_task(void *argument);
modFaultDetecTypeDef* module_fault_detect_get_state(void);
void module_fault_detect_start(void);
void module_fault_detect_stop(void);
void module_fault_detect_set_estop(estopState);
void module_fault_detect_set_pressure(float);
void module_fault_detect_set_temperature(float);
void module_fault_detect_set_error(errorCode);
void module_fault_detect_manual_estop(int);
void module_fault_detect_manual_error(int);
void module_fault_detect_manual_pressure(float);
void module_fault_detect_manual_temperature(float);
void module_fault_detect_print_inputs(void);

#endif