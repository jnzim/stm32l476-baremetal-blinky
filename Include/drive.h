#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

typedef enum {
    STATE_IDLE      = 0,
    STATE_OPEN_LOOP = 1,
    STATE_ALIGN     = 2,
    STATE_SERVO_ON  = 3,
    STATE_FAULT     = 4,
} DriveState;

// Lifecycle
void       drive_init(void);
void       drive_update(void);
DriveState drive_get_state(void);
uint8_t    starting(void);

// Request functions — called from ISRs, flags consumed by drive_update()
void drive_request_servo_on(void);
void drive_request_open_loop(float v_mag, float d_theta);
void drive_request_stop(void);
void drive_request_fault(void);

#endif // DRIVE_H