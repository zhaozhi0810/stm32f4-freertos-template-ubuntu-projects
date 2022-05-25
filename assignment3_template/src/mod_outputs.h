#ifndef MOD_OUTPUTS_H
#define MOD_OUTPUTS_H

#include <stdint.h>
#include "fault_codes.h"

typedef enum{
    OPEN = 0,
    CLOSED = 1,
} relayState;

void module_outputs_init(void);
void module_outputs_task(void *argument);
void module_outputs_update(void);
void module_outputs_start(void);
void module_outputs_stop(void);
void module_outputs_set_state(relayState);
relayState* module_outputs_get_state(void);
void module_outputs_manual_state(int);

#endif