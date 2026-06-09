#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>
#include "protocol.h"

#define PLANT_KT 0.06f        // Nm/A — corrected from datasheet
#define PLANT_KE 0.0618f      // V/(rad/s) — should equal KT for DC motor
#define PLANT_R  3.9f         // ohm — verify from datasheet
#define PLANT_L  0.002f       // H — from datasheet (2.0 mH, not 2.68)
#define PLANT_J  1.7e-6f      // kg*m^2, plus load later
#define PLANT_B  1e-4f        // N*m*s/rad — friction/damping (guessed)





typedef struct {
    float   vel_rad;     // rad/s
    float   pos_rad;     // rad
    float   i_q;        // A
    int32_t pos_counts; // encoder counts
    int32_t vel_counts; // counts/sec
} PlantState;

void  plant_init(PlantState *s);
void  plant_step(PlantState *s, float v_q, float dt);
extern PlantState plant;

#endif