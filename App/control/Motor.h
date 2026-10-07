#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "math.h"
#include "power_manager.h"

//PPR(Pulses Per Revolution) = 13, so Counts_PER_REV = 13*4 = 52
#define PULSE_PER_REV 52.0f
#define SAMPLE_TIME 0.01f // 10ms
#define REDUCTION_RATIO 30.0f
#define MIN_PERCENT 10
#define K_SYNC 0.05	//for Straight Correction range is 0.01~0.05
#define MAX_RPM 350
typedef struct {
	float Kp, Ki, Kd;
	float integral;
	float last_deviation;
	float target_RPM;
	float last_output;
} PID_t;

enum Direction{
	FORWORD = 0,
	REVERSES = 1,
	SHORT_BRAKE = 2,
	LEFT = 3,
	RIGHT = 4,
	STOP = 5,
	STANDBY = 6
};

//enc1: TIM2(encoder 2)
//enc2: TIM3(encoder 3)
typedef struct {
	TIM_HandleTypeDef *htim;
	int16_t last_count; 
	int16_t diff;
	float rpm;
	float rpm_filtered;
	float reduction_rpm;
}Encoder_t;
extern Encoder_t rEncoder, lEncoder;

/*
*	Regarding Motor
*/
void Mortor_Controler(TIM_HandleTypeDef *tim, unsigned int channel, enum Direction dir, float percent);
void Motor_Init();
void Stop_Motor();
void Set_Pwm_Duty_Percent(TIM_HandleTypeDef *tim,unsigned int channel, float percent);
void PID_Update(PID_t *pid, float target_rpm, float current_rpm);
void Reset_PID(PID_t *pid);
float Straight_Correction(float rpm_L, float rpm_R);
/*
*	Regarding Encoder
*/
void Update_Encoder(Encoder_t *enc);
#endif
