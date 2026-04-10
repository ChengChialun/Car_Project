#include "system_interrupt.h"

/**************************************************************************
Function: Read encoder count per unit time
Input   : TIMX:Timer
Output  : none
**************************************************************************/
/*int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;	
	   case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
	   default: Encoder_TIM=0;
	 }
//	 if(Encoder_TIM<0)  Encoder_TIM=-Encoder_TIM;
		return Encoder_TIM;
}*/



//callback function for timer interrupts
//(ISR)TIMx_IRQHandler->(Callback)HAL_TIM_PeriodElapsedCallback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6) //My System interrupts on Timer 6
	{
		static int cnt_5ms = 0;
		static int cnt_10ms = 0;
		static int cnt_100ms = 0;
		static int cnt_1s = 0;
		cnt_5ms++;
		cnt_10ms++;
		cnt_100ms++;
		cnt_1s++;
		if(cnt_10ms >= 5) // 5ms
		{
			flag_5ms = 1; //enable 5 miliseconds flag for IMU Calibration
			cnt_5ms = 0;
		}
		if(cnt_10ms >= 10) // 10ms
		{
			flag_10ms = 1; //enable 10 miliseconds flag
			cnt_10ms = 0;
		}
		if(cnt_100ms >= 100) // 100ms
		{
			flag_100ms = 1;
			cnt_100ms = 0;
		}
		if(cnt_1s >= 1000) // 1ms
		{
			flag_1s = 1;
			cnt_1s = 0;
		}
	}
}