#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float integrator;
    float out_min;
    float out_max;
} PIState;

typedef struct {
    float kp;
} PState;

void  pi_init(PIState *s, float kp, float ki, float out_min, float out_max);
float pi_step(PIState *s, float error, float dt);

void  p_init(PState *s, float kp);
float p_step(PState *s, float error);

#endif