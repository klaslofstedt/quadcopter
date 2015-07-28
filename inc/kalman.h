#ifndef KALMAN_H
#define KALMAN_H

#include "quadcopter_structures.h"

#define M_PI 3.14159265358979323846

void Kalman_Calc(IMU_DATA_t* a, IMU_DATA_t* g, PID_DATA_t* r, PID_DATA_t* p, PID_DATA_t* y);
void UpdateStageRoll();
void PropagationStageRoll();
void UpdateStagePitch();
void PropagationStagePitch();
void CorrectGyroBias();

#endif
