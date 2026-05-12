#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>
#include "protocol.h"

#define PLANT_R         3.25f
#define PLANT_L         0.0018f
#define PLANT_KT        0.072f
#define PLANT_J         3.2e-6f
#define PLANT_B         0.001f   // was 1e-6, increase 1000x

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