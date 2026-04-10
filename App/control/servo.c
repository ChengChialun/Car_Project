#include "servo.h"
#include "math.h"
#include "imu_fusion.h"
#include "Motor.h"
//#define SERVO_SCALE 11.1111f	1000 / 90	(2.5ms ~ 0.5ms)
//#define SERVO_SCALE 5.5556f		//500.0f / 90.0f (2ms ~ 1ms)
//以下為手動找出的pulse
//且假設以中心點0度, 控制角度為正時是順時針轉, 為負時是逆時針
#define ANGLE_MAX   	30
#define SERVO_CENTER 	1580 		//(LEFT_MAX + RIGHT_MAX) / 2
#define LEFT_MAX   		200
#define RIGHT_MAX   	1200
#define SERVO_SCALE_LEFT 20.6667f	//(LEFT_MAX - SERVO_CENTER) / 30
#define SERVO_SCALE_RIGHT 10.6667f		//(SERVO_CENTER - RIGHT_MAX) / 30
#define MAX_STEP 		5.0f
//#define SERVO_SCALE 	16.6667f	//(LEFT_MAX - SERVO_CENTER) / 30
//center 1500
//right  1260
//left	 2200
extern Servo_PID_t Servo_PID;
extern PID_t pid_L;
extern PID_t pid_R;
extern volatile Euler_t euler;
void Set_Servo_Angle(TIM_HandleTypeDef *tim, unsigned int channel, int16_t angle)
{
	uint16_t pulse;
	//limitation angle is +-30
	if( angle > ANGLE_MAX )		angle = ANGLE_MAX;
	if( angle < -ANGLE_MAX )	angle = -ANGLE_MAX;
	//uint16_t pulse = 1500 + (angle * 1000 / 180); // 0.5ms~2.5ms
	//uint16_t pulse = 1500 + ((angle * 1000) / 90);
//    uint16_t pulse = 1500 + angle * SERVO_SCALE;
	if( angle >= 0 )
		pulse = SERVO_CENTER + angle * SERVO_SCALE_LEFT;
	else
		pulse = SERVO_CENTER + angle * SERVO_SCALE_RIGHT;
	Servo_PID.target_yaw = angle;
	//pulse = 2200;
	//pulse = 1580;
    __HAL_TIM_SET_COMPARE(tim, channel, pulse);
}

void Servo_Init()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void Servo_Soft_Start(TIM_HandleTypeDef *tim, unsigned int channel , int target_angle)
{
 /*   static int current_angle = 0;

    while(current_angle != target_angle)
    {
        if(current_angle < target_angle) current_angle++;
        else current_angle--;

        Set_Servo_Angle(tim, channel, current_angle);
        HAL_Delay(10);
    }*/
}

void Servo_PID_Update(TIM_HandleTypeDef *tim, unsigned int channel)
{
    // ===== read current yaw =====
    float current_yaw = euler.yaw;
    // ===== deviation =====
    float angle_deviation = Servo_PID.target_yaw - current_yaw;

    // wrap [-180,180]
    if(angle_deviation > 180)  angle_deviation -= 360;
    if(angle_deviation < -180) angle_deviation += 360;

    // Dead Zone
    if(fabs(angle_deviation) < 1.0f)
    	angle_deviation = 0;

	//derivative
    float diff = (angle_deviation - Servo_PID.last_deviation);
    if(diff > 180)  diff -= 360;
    if(diff < -180) diff += 360;
    float derivative = diff * 100.0f; // 10ms loop

    //PD control("I" is not necessary)
    float steering_angle = Servo_PID.Kp * angle_deviation + Servo_PID.Kd * derivative;

    // limit angle
    if(steering_angle > 30)  steering_angle = 30;
    if(steering_angle < -30) steering_angle = -30;

    // 每次最多改5度
    float delta = steering_angle - Servo_PID.last_output;

    if(delta > MAX_STEP)  steering_angle = Servo_PID.last_output + MAX_STEP;
    if(delta < -MAX_STEP) steering_angle = Servo_PID.last_output - MAX_STEP;
    //save
    Servo_PID.last_deviation = angle_deviation;
    Servo_PID.last_output = steering_angle;
    // control servo
    Set_Servo_Angle(tim, channel, steering_angle);
}

void Differential_Control(float base_speed, float *out_L, float *out_R)
{
	float steering_angle = Servo_PID.last_output;

	//dead zone
	if( fabs(steering_angle) <= 1.0f ) {
        *out_L = base_speed;
        *out_R = base_speed;
        return;
    }
	// 差速控制
	float factor_L = 1 - Servo_PID.k * steering_angle;
	float factor_R = 1 + Servo_PID.k * steering_angle;

	// normalize
	float max_factor = fmaxf(fabs(factor_L), fabs(factor_R));
	if(max_factor > 1.0f)
	{
	    factor_L /= max_factor;
	    factor_R /= max_factor;
	}

	*out_L = base_speed * factor_L;
	*out_R = base_speed * factor_R;
}
