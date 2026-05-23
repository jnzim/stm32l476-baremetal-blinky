#include "control.h"
#include "config.h"

void pi_init(PIState *s, float kp, float ki, float out_min, float out_max)
{
    s->kp         = kp;
    s->ki         = ki;
    s->integrator = 0.0f;
    s->out_min    = out_min;
    s->out_max    = out_max;
}

float pi_step(PIState *s, float error, float dt)
{
    s->integrator += s->ki * error * dt;
    if (s->integrator > s->out_max) s->integrator = s->out_max;
    if (s->integrator < s->out_min) s->integrator = s->out_min;
    float out = s->kp * error + s->integrator;
    if (out > s->out_max) out = s->out_max;
    if (out < s->out_min) out = s->out_min;
    return out;
}

void p_init(PState *s, float kp)
{
    s->kp = kp;
}

float p_step(PState *s, float error)
{
    return s->kp * error;
}