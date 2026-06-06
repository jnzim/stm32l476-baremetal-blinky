#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

typedef enum {
    STATE_IDLE       = DRIVE_IDLE,
    STATE_OPEN_LOOP  = DRIVE_OPEN_LOOP,
    STATE_ALIGN      = DRIVE_ALIGN,
    STATE_SERVO_ON   = DRIVE_SERVO_ON,
    STATE_FAULT      = DRIVE_FAULT
} DriveState;

typedef struct 
{
    uint8_t  state;
    uint8_t  fault_flags;
    uint32_t samples_consumed;
    // ... other motor/control fields
} drive_t;

extern drive_t drive; 

// Lifecycle
void       drive_init(void);
void       drive_sm_run(void);
DriveState drive_get_state(void);
uint8_t    starting(void);

// Status queries — read-only
bool drive_is_idle(void);
bool drive_is_open_loop(void);
bool drive_is_aligned(void);
bool drive_is_servo_on(void);
bool drive_is_faulted(void);

// Request functions — called from ISRs, flags consumed by drive_sm_run()
void drive_request_servo_on(void);
void drive_request_open_loop(float v_mag, float d_theta);
void drive_request_stop(void);
void drive_request_fault(void);

#endif // DRIVE_H