#include "power_manager.h"

/****************************************************
* Below is regarding Power Management Strategies
* Voltage				| Label										| Behavior				
*-------------------------------------------------------
*	> 10.5V				| NORMAL_POWER_STATUS			| Normal
*	10.5V ~ 10.0V	| PROTECTION_POWER_STATUS	| PWM limitation , Inform , blinking led , buzzer
*	< 10.0V				| STOP_POWER_STATUS				| Stop
****************************************************/

float Get_Voltage(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);  // Wait for conversion is finished until 10ms timeout
	uint32_t raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//official formula
	voltage = raw * 3.3f * 11.0f / 4095.0f; //11 is Voltage Divider
	
	if(0 == filtered_voltage)	filtered_voltage = voltage;
	filtered_voltage = filtered_voltage * 0.7f + voltage * 0.3f;

	return filtered_voltage;
}

float Get_Power_Limit()
{
    float limit = 1.0f;

    switch (PowerStatus)
    {
        case NORMAL_POWER_STATUS:
            limit = 1.0f;
            break;

        case PROTECTION_POWER_STATUS:
            limit = (filtered_voltage - 10.0f) / 0.5f;  // 10.5 -> 1 , 10.0 -> 0
            if (limit < 0) limit = 0;
            if (limit > 1) limit = 1;
            break;

        case STOP_POWER_STATUS:
            limit = 0.0f;
            break;
    }

    return limit;
}

void Power_Manager()
{
	static uint8_t low_voltage_counter = 0;
	static uint8_t stop_latch = 0;

	Get_Voltage();

	if(stop_latch){
		PowerStatus = STOP_POWER_STATUS;
		return;
  }
	
	if (filtered_voltage >= 10.5f){
		PowerStatus = NORMAL_POWER_STATUS;
		low_voltage_counter = 0;
	}
	else if (filtered_voltage > 10.0f){
		PowerStatus = PROTECTION_POWER_STATUS;
		low_voltage_counter = 0;
	}
	else{
		low_voltage_counter++;
		if(low_voltage_counter > LOW_POWER_TIMES)	//continuous 5 times
		{
			PowerStatus = STOP_POWER_STATUS;
			stop_latch = 1;
		}
	}
}