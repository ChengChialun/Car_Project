#ifndef SERVO_H_
#define SERVO_H_
#include "main.h"
#include "tim.h"

typedef struct {
	float Kp, Kd;
	float last_deviation;
	float target_yaw;
	float last_output;
	float k; //0.005 ~ 0.02
} Servo_PID_t;
void Servo_Soft_Start(TIM_HandleTypeDef *tim, unsigned int channel , int target_angle);
void Set_Servo_Angle(TIM_HandleTypeDef *tim, unsigned int channel, int16_t angle);
void Servo_Init();
void Servo_PID_Update(TIM_HandleTypeDef *tim, unsigned int channel);
void Differential_Control(float base_speed, float *out_L, float *out_R);
#endif /* SERVO_H_ */
