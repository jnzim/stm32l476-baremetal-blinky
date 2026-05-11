#include "plant.h"

void plant_init(PlantState *s)
{
    s->vel = 0.0f;
    s->pos = 0.0f;
    s->i_q = 0.0f;
}

void plant_step(PlantState *s, float v_q, float dt)
{
    // Torque from q-axis voltage (mechanical model only, no L/R lag)
    float torque = PLANT_KT * (v_q / PLANT_R);

    // Newton-Euler: J*dw/dt = torque - B*w
    float accel = (torque - PLANT_B * s->vel) / PLANT_J;

    // Euler integration
    s->vel += accel * dt;
    s->pos += s->vel * dt;
    s->i_q  = v_q / PLANT_R;
}