#ifndef PID_h
#define PID_h

#include "quadcopter_structures.h"

#define Kp 0.0060
#define Ki 0.0
#define Kd 0.0018
/*#define Kp 0.0
#define Ki 0.0
#define Kd 0.0*/

void PID_Calc(PID_DATA_t* pid);

#endif
