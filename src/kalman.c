#include "kalman.h"


const float Ts = 0.00333333333;
const float Q[2] = {0.0000009386,0};
const float R[2] = {3.89,3.89};
float Proll[3] = {1,0,1};
float Ppitch[3] = {1,0,1};

float roll = 0;
float biasroll = 0;
float vroll = 0;
float pitch = 0;
float biaspitch = 0;
float vpitch = 0;
float yaw = 0;
float vyaw = 0;

float acc[3];
float gyr[3];

// acc in three dimensions, gyr in three dimensions - input
// roll: acc, vel, setpoint - output
void Kalman_Calc(IMU_DATA_t* a, IMU_DATA_t* g, PID_DATA_t* r, PID_DATA_t* p, PID_DATA_t* y)
{
    acc[0] = a->Roll;
    acc[1] = a->Pitch;
    acc[2] = a->Yaw;
    gyr[0] = g->Roll;
    gyr[1] = g->Pitch;
    gyr[2] = g->Yaw;
	UpdateStageRoll();
	PropagationStageRoll();
	UpdateStagePitch();
	PropagationStagePitch();
	CorrectGyroBias();

    r->Degrees = roll * (180 / M_PI);
    r->Velocity = vroll * (180 / M_PI);
    p->Degrees = pitch * (180 / M_PI);
    p->Velocity = vpitch * (180 / M_PI);
}

void UpdateStageRoll()
{
	float alpha = Proll[0]/(Proll[0] + R[0]);
	float gamma = Proll[1]/(Proll[0] + R[0]);
	biasroll = biasroll - gamma*(acc[1] + roll);
	roll = (1-alpha)*roll - alpha*acc[1];

	Proll[0] = (1-alpha)*Proll[0];
	Proll[1] = (1-alpha)*Proll[1];
	Proll[2] =  -gamma*Proll[1] + Proll[2];
}

void PropagationStageRoll()
{
	roll = roll + Ts*(gyr[0] - biasroll);
	Proll[0] = Proll[0] - 2*Ts*Proll[1] + Ts*Ts*Proll[2] + Q[0];
	Proll[1] = Proll[1] - Ts*Proll[2];
	Proll[2] = Proll[2] + Q[1];
}

void UpdateStagePitch()
{
	float alpha = Ppitch[0]/(Ppitch[0] + R[1]);
	float gamma = Ppitch[1]/(Ppitch[0] + R[1]);
	biaspitch = biaspitch + gamma*(acc[0] - pitch);
	pitch = (1-alpha)*pitch + alpha*acc[0];

	Ppitch[0] = (1-alpha)*Ppitch[0];
	Ppitch[1] = (1-alpha)*Ppitch[1];
	Ppitch[2] =  -gamma*Ppitch[1] + Ppitch[2];
}

void PropagationStagePitch()
{
	pitch = pitch + Ts*(gyr[1] - biaspitch);
	//biasroll = biasroll;

	Ppitch[0] = Ppitch[0] - 2*Ts*Ppitch[1] + Ts*Ts*Ppitch[2] + Q[0];
	Ppitch[1] = Ppitch[1] - Ts*Ppitch[2];
	Ppitch[2] = Ppitch[2] + Q[1];
}

void CorrectGyroBias()
{
	vroll = gyr[0] - biasroll;
	vpitch = gyr[1] - biaspitch;
}
