#ifndef _SERVO_SM_H_
#define _SERVO_SM_H_

typedef enum
{
    INIT = 0,
    ALIGN,
    IDLE,
    RUN,
    FAULT
} State;

typedef enum
{
    RUN_MODE_NONE = 0,

    RUN_MODE_PROFILE,

    RUN_MODE_CURRENT_STEP,
    RUN_MODE_CURRENT_CHIRP,

    RUN_MODE_VELOCITY_STEP,
    RUN_MODE_VELOCITY_CHIRP,

    RUN_MODE_POSITION_STEP,
    RUN_MODE_POSITION_CHIRP
} RunMode;


void SM_Run(void);

#endif