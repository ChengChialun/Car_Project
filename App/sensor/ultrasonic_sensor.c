#include "ultrasonic_sensor.h"
#define FRONT_ULTRASONIC 		1
#define REAR_ULTRASONIC 		2
#define SOUND_SPEED 			0.0343f
#define SOUND_SPEED_SIMPLIFY	0.01715f	//0.0343 / 2
#define ULTRASONIC_TIMEOUT 		50			//milliseconds
#define MAX_DISTANCE			400			//mm
#define MIN_DISTANCE			2			//mma
#define LIMIT_STEP				5			//according d= v * t = (car speed) * (move time) = 1m/s * 50ms
#define INTERVAL_DELAY_TIME		25

volatile uint32_t usonic_last_time = 0;
extern Ultrasonic_t front_sonic, rear_sonic;
void Ultrasonic_Trigger(uint8_t dir, TIM_HandleTypeDef *htim)
{
#if US100_ENABLED
	uint8_t cmd = US100_CMD_TRIGER;
	if(FRONT_ULTRASONIC == dir) //front
	{
		memset(front_sonic.rx_buf, 0, US100_RX_LENGTH);
		HAL_UART_Receive_DMA(&FRONT_US_UART_HANDLE, front_sonic.rx_buf, US100_RX_LENGTH);
		HAL_UART_Transmit(&FRONT_US_UART_HANDLE, &cmd, 1, 10);
	}
	else if(REAR_ULTRASONIC == dir) //rear
	{
		memset(rear_sonic.rx_buf, 0, US100_RX_LENGTH);
		HAL_UART_Receive_DMA(&REAR_US_UART_HANDLE, rear_sonic.rx_buf, US100_RX_LENGTH);
		HAL_UART_Transmit(&REAR_US_UART_HANDLE, &cmd, 1, 10);
	}
#else
	__HAL_TIM_SET_COUNTER(htim, 0);
	if(FRONT_ULTRASONIC == dir) //front
	{
		HAL_GPIO_WritePin(Front_Ultrasonic_GPIO_Port, Front_Ultrasonic_Pin, GPIO_PIN_RESET);
		Delay_us(10); // 10us
		HAL_GPIO_WritePin(Front_Ultrasonic_GPIO_Port, Front_Ultrasonic_Pin, GPIO_PIN_SET);
		Delay_us(10); // 10us
		HAL_GPIO_WritePin(Front_Ultrasonic_GPIO_Port, Front_Ultrasonic_Pin, GPIO_PIN_RESET);
	}
    else if(REAR_ULTRASONIC == dir) //rear
    {
    	HAL_GPIO_WritePin(Rear_Ultrasonic_GPIO_Port, Rear_Ultrasonic_Pin, GPIO_PIN_RESET);
		Delay_us(10); // 10us
    	HAL_GPIO_WritePin(Rear_Ultrasonic_GPIO_Port, Rear_Ultrasonic_Pin, GPIO_PIN_SET);
    	Delay_us(10); // 10us
    	HAL_GPIO_WritePin(Rear_Ultrasonic_GPIO_Port, Rear_Ultrasonic_Pin, GPIO_PIN_RESET);
    }
#endif
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5)
	{
#if !(US100_ENABLED)
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(front_sonic.start_detect == 0)
			{
				front_sonic.start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				front_sonic.start_detect = 1;
			}
			else
			{
				front_sonic.end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				front_sonic.start_detect = 0;
				front_sonic.finish_detect = 1;
			}
		}
		else if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
		{
			if(rear_sonic.start_detect == 0)
			{
				rear_sonic.start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				rear_sonic.start_detect = 1;
			}
			else
			{
				rear_sonic.end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				rear_sonic.start_detect = 0;
				rear_sonic.finish_detect = 1;
			}
		}
#endif
	}
}

// sort and get median
// n is recommend odd (3, 5, 7, 9)
float Get_Median_Filter(float *data, uint8_t n) {
    float temp;
    uint8_t i, j;

    // Insertion Sort
    for (i = 1; i < n; i++) {
        temp = data[i];
        j = i;
        while (j > 0 && data[j - 1] > temp) {
            data[j] = data[j - 1];
            j--;
        }
        data[j] = temp;
    }

    // return median
    return data[n / 2];
}

void calc_distance(Ultrasonic_t *Ultrasonic, TIM_HandleTypeDef *htim)
{
	float tmp;

#if US100_ENABLED
    Ultrasonic->raw_mm = ((uint16_t)Ultrasonic->rx_buf[0] << 8) | Ultrasonic->rx_buf[1];
    if(0 == Ultrasonic->raw_mm || Ultrasonic->raw_mm > MAX_DISTANCE)
    {
    	Ultrasonic->distance = INVALID_DISTANCE;
    	return;
    }
	tmp = Ultrasonic->raw_mm;
#else
	uint32_t diff;
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
	if(Ultrasonic->end > Ultrasonic->start)
		diff = Ultrasonic->end - Ultrasonic->start;
	else//when timer overflow
		diff = ((arr + 1) - Ultrasonic->start) + Ultrasonic->end;

	tmp = diff * SOUND_SPEED_SIMPLIFY;
#endif


	if(tmp < MAX_DISTANCE && tmp > MIN_DISTANCE)
	{
		// ring buffer
		Ultrasonic->history_data[Ultrasonic->idx] = tmp;
		Ultrasonic->idx = (Ultrasonic->idx + 1) % HISTORY_DATA_SIZE;	//point to the old data for replace

		float buf[HISTORY_DATA_SIZE];
		memcpy(buf, Ultrasonic->history_data, sizeof(buf));

		float filter = Get_Median_Filter(buf, HISTORY_DATA_SIZE);	//get median
		/*float last = Ultrasonic->distance;

		//rate limit(避免前後資料因異常資料而變化過大)
		if( filter - last > LIMIT_STEP )
		{
			filter = last + LIMIT_STEP;
		}
		else if ( last - filter > LIMIT_STEP )
		{
			filter = last - LIMIT_STEP;
		}*/
		Ultrasonic->distance = filter;
	}
}

uint32_t Ultrasonic_Init(TIM_HandleTypeDef *htim)
{
#if !(US100_ENABLED)
	if (HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_3) != HAL_OK) {
	    return SYSTEM_ERROR;
	}
	if (HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4) != HAL_OK) {
		return SYSTEM_ERROR;
	}
#endif
	usonic_last_time = 0;
	front_sonic.start_detect = 0;
	front_sonic.finish_detect = 0;
	front_sonic.idx = 0;
	rear_sonic.start_detect = 0;
	rear_sonic.finish_detect = 0;
	rear_sonic.idx = 0;
	sonic_state = TRIG_FRONT;

	return OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &FRONT_US_UART_HANDLE)
    {
/*        uint16_t dist = ((uint16_t)front_sonic.rx_buf[0] << 8) | front_sonic.rx_buf[1];
        if(0 == dist || dist > MAX_DISTANCE)
        	return;
        front_sonic.raw_mm = dist;*/

        front_sonic.finish_detect = 1;
    }
    if(huart == &REAR_US_UART_HANDLE)
    {
/*        uint16_t dist = ((uint16_t)rear_sonic.rx_buf[0] << 8) | rear_sonic.rx_buf[1];
        if(0 == dist || dist > MAX_DISTANCE)
        	return;
        rear_sonic.raw_mm = dist;*/

        rear_sonic.finish_detect = 1;
    }
}

void Ultrasonic_Task(TIM_HandleTypeDef *htim)
{
    uint32_t now = HAL_GetTick();

    switch(sonic_state)
    {
        case TRIG_FRONT:
            Ultrasonic_Trigger(FRONT_ULTRASONIC, htim);
            usonic_last_time = now;
            sonic_state = WAIT_FRONT;
            break;

        case WAIT_FRONT:
            if(front_sonic.finish_detect)
            {
                front_sonic.finish_detect = 0;
                calc_distance(&front_sonic, htim);
                sonic_state = INTERVAL_DELAY_REAR;
                usonic_last_time = now;
            }
            else if(now - usonic_last_time > ULTRASONIC_TIMEOUT) // timeout 50 ms
            {
                sonic_state = TRIG_REAR;
                front_sonic.start_detect = 0;
                front_sonic.finish_detect = 0;
                front_sonic.distance = INVALID_DISTANCE;
            }
            break;
        case INTERVAL_DELAY_REAR://25ms
        	if(now - usonic_last_time > INTERVAL_DELAY_TIME)
        	{
        		sonic_state = TRIG_REAR;
        	}
        	break;
        case TRIG_REAR:
            Ultrasonic_Trigger(REAR_ULTRASONIC, htim);
            usonic_last_time = now;
            sonic_state = WAIT_REAR;
            break;

        case WAIT_REAR:
            if(rear_sonic.finish_detect)
            {
                rear_sonic.finish_detect = 0;
                calc_distance(&rear_sonic, htim);
                sonic_state = INTERVAL_DELAY_FRONT;
                usonic_last_time = now;
            }
            else if(now - usonic_last_time > ULTRASONIC_TIMEOUT)
            {
                sonic_state = TRIG_FRONT;
                rear_sonic.start_detect = 0;
                rear_sonic.finish_detect = 0;
                rear_sonic.distance = INVALID_DISTANCE;
            }
            break;
        case INTERVAL_DELAY_FRONT://25ms
        	if(now - usonic_last_time > INTERVAL_DELAY_TIME)
        	{
        		sonic_state = TRIG_FRONT;
        	}
        	break;
    }
}
