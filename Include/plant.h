#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>
#include "protocol.h"


#define PLANT_R         3.9f
#define PLANT_L         0.00268f
#define PLANT_KT        0.1125f
#define PLANT_J         1.7e-6f
#define PLANT_B         0.0001f



typedef struct {
    float   vel;        // rad/s
    float   pos;        // rad
    float   i_q;        // A
    int32_t pos_counts; // encoder counts
    int32_t vel_counts; // counts/sec
} PlantState;

void  plant_init(PlantState *s);
void  plant_step(PlantState *s, float v_q, float dt);

#endif