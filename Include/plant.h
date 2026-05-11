#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>

#define PLANT_R         3.25f
#define PLANT_L         0.0018f
#define PLANT_KT        0.072f
#define PLANT_J         3.2e-6f
#define PLANT_B         1.0e-6f

typedef struct {
    float vel;
    float pos;
    float i_q;
} PlantState;

void  plant_init(PlantState *s);
void  plant_step(PlantState *s, float v_q, float dt);

#endif