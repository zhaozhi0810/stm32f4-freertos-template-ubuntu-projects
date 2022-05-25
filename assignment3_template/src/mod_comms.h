#ifndef MOD_COMMS_H
#define MOD_COMMS_H

#include <stdint.h>

void module_comms_start_received(void);
void module_comms_heartbeat(void);
void module_comms_start(void);
void module_comms_stop(void);

#endif