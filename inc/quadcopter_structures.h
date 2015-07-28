#ifndef quadcopter_structures_H
#define quadcopter_structures_H


typedef struct{
	float Degrees;
	float Velocity;
	float SetPoint;
}ROLL_t;

typedef struct{
	float Degrees;
	float Velocity;
	float SetPoint;
}PITCH_t;

typedef struct{
	float Degrees;
	float Velocity;
	float SetPoint;
}YAW_t;

typedef struct{
	float Roll;
	float Pitch;
	float Yaw;
} MPU_ACC_t;

typedef struct{
	float Roll;
	float Pitch;
	float Yaw;
} MPU_GYR_t;

typedef struct{
	float Roll;
	float Pitch;
	float Yaw;
} IMU_DATA_t;

typedef struct{
	float Degrees; // döp om till Angle
	float Velocity;
	float SetPoint;

	float I_Term;
	float Output;
}PID_DATA_t;

#endif
