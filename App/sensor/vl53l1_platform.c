/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "vl53l1_platform.h"
#include "debug.h"

#include "../../Modules/TCA9548A/TCA9548A.h"
#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
#define VL53L1X_ADDR 		(0x29 << 1)
#ifdef VL53L1_LOG_ENABLE
#define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif

#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif

VL53L1_Error status;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
#ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
//#define VL53L1_GetI2cBus(...)	__disable_irq()
#define VL53L1_GetI2cBus(...)
#endif

#ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
//#define VL53L1_PutI2cBus(...)	__enable_irq()
#define VL53L1_PutI2cBus(...)
#endif

uint8_t _I2CBuffer[256];
VL53L1_Error Get_Distance(VL53L1_Dev_t *dev);

int _I2CWrite(uint16_t Dev, uint8_t *pdata, uint32_t count , uint8_t channel) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = TCA_SelectChannel(&VL53L1X_I2C_HANDLE, channel);
    status += HAL_I2C_Master_Transmit(&VL53L1X_I2C_HANDLE, Dev, pdata, count, i2c_time_out);
    /*uint8_t buffer[2];
    status = HAL_I2C_Mem_Read(&VL53L1X_I2C_HANDLE, Dev, pdata, I2C_MEMADD_SIZE_16BIT, buffer, 1, 100);
        if (status == HAL_OK) {
            printf("Read Success: 0x%02X\n", buffer[0]);
        } else {
            printf("Read Failed, HAL Status: %d\n", status);
        }*/
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&VL53L1X_I2C_HANDLE);
    }
    return status;
}

int _I2CRead(uint16_t Dev, uint8_t *pdata, uint32_t count, uint8_t channel) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = TCA_SelectChannel(&VL53L1X_I2C_HANDLE, channel);
    status += HAL_I2C_Master_Receive(&VL53L1X_I2C_HANDLE, Dev|1, pdata, count, i2c_time_out);
    /*uint8_t buffer[2];
    status = HAL_I2C_Mem_Read(&VL53L1X_I2C_HANDLE, 0x52, 0x010F, I2C_MEMADD_SIZE_16BIT, buffer, 1, 100);
    if (status == HAL_OK) {
        printf("Read Success: 0x%02X\n", buffer[0]);
    } else {
        printf("Read Failed, HAL Status: %d\n", status);
    }*/
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&VL53L1X_I2C_HANDLE);
    }
    return status;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, count + 2, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, 2, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(pdev->i2c_slave_address, pdata, count, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data;

    VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, 3, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data >> 8;
    _I2CBuffer[3] = data & 0x00FF;

    VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, 4, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = (data >> 24) & 0xFF;
    _I2CBuffer[3] = (data >> 16) & 0xFF;
    _I2CBuffer[4] = (data >> 8)  & 0xFF;
    _I2CBuffer[5] = (data >> 0 ) & 0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, 6, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint8_t data;

    Status = VL53L1_RdByte(pdev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L1_WrByte(pdev, index, data);
done:
    return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

	_I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
	VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, 2, pdev->i2c_channel);
    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(pdev->i2c_slave_address, data, 1, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
	VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, 2, pdev->i2c_channel);

    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(pdev->i2c_slave_address, _I2CBuffer, 2, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
done:
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(pdev->i2c_slave_address, _I2CBuffer, 2, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(pdev->i2c_slave_address, _I2CBuffer, 4, pdev->i2c_channel);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];

done:
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{

    /* Returns current tick count in [ms] */

	VL53L1_Error status  = VL53L1_ERROR_NONE;

	//*ptick_count_ms = timeGetTime();
	*ptick_count_ms = 0;

#ifdef VL53L1_LOG_ENABLE
	trace_print(
		VL53L1_TRACE_LEVEL_DEBUG,
		"VL53L1_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);
#endif

	return status;
}

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 0;

	trace_print(VL53L1_TRACE_LEVEL_INFO, "VL53L1_GetTimerFrequency: Freq : %dHz\n", *ptimer_freq_hz);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	(void)pdev;
	HAL_Delay(wait_ms);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
	(void)pdev;
	Delay_us(wait_us);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{

	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t     start_time_ms = 0;
	uint32_t     current_time_ms = 0;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53L1_LOG_ENABLE
	uint8_t      trace_functions = VL53L1_TRACE_FUNCTION_NONE;
#endif

//	char   register_name[VL53L1_MAX_STRING_LENGTH];

    /* look up register name */
#ifdef PAL_EXTENDED
	VL53L1_get_register_name(
			index,
			register_name);
#else
//	VL53L1_COPYSTRING(register_name, "");
#endif

	/* Output to I2C logger for FMT/DFT  */

    /*trace_i2c("WaitValueMaskEx(%5d, 0x%04X, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, index, value, mask, poll_delay_ms); */
    trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, register_name, value, mask, poll_delay_ms);

	/* calculate time limit in absolute time */
    VL53L1_GetTickCount(&start_time_ms);

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = VL53L1_get_trace_functions();
	VL53L1_set_trace_functions(VL53L1_TRACE_FUNCTION_NONE);
#endif

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53L1_ERROR_NONE) &&
		   (polling_time_ms < timeout_ms) &&
		   (found == 0)) {

		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_RdByte(
							pdev,
							index,
							&byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
					pdev,
					poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53L1_GetTickCount(&current_time_ms);
		polling_time_ms = current_time_ms - start_time_ms;

	}

#ifdef VL53L1_LOG_ENABLE
	/* Restore function logging */
	VL53L1_set_trace_functions(trace_functions);
#endif

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}

VL53L1_Error VL53L1_CommsInitialise(VL53L1_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	// select TCA9548A channel
	status = TCA_SelectChannel(&VL53L1X_I2C_HANDLE, pdev->i2c_channel);
	if(status != HAL_OK) return status;

	// Boot & init
    status = VL53L1_WaitDeviceBooted(pdev);
    if(status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_DataInit(pdev);
    if(status != VL53L1_ERROR_NONE) return status;
    status = VL53L1_StaticInit(pdev);
    if(status != VL53L1_ERROR_NONE) return status;

    // Mode
    status = VL53L1_SetDistanceMode(pdev, VL53L1_DISTANCEMODE_LONG);
    if(status != VL53L1_ERROR_NONE) return status;

    // Timing
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pdev, 50000); // 50ms
    if(status != VL53L1_ERROR_NONE) return status;
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pdev, 60);     // must > 50ms
    if(status != VL53L1_ERROR_NONE) return status;

    // Distance threshold
	// below 500mm -> interrupt
    VL53L1_DetectionConfig_t detectionConfig;
    memset(&detectionConfig, 0, sizeof(detectionConfig));
    detectionConfig.DetectionMode = VL53L1_DETECTION_DISTANCE_ONLY;
    detectionConfig.IntrNoTarget = 0;
    detectionConfig.Distance.CrossMode = VL53L1_THRESHOLD_IN_WINDOW;
    detectionConfig.Distance.Low = 0;
    detectionConfig.Distance.High = DISTANCE_THRESHOLD;
    status = VL53L1_SetThresholdConfig(pdev, &detectionConfig);
    if(status != VL53L1_ERROR_NONE) return status;

	// Falling edge trigger (default is falling)
    status = VL53L1_SetInterruptPolarity(pdev, VL53L1_DEVICEINTERRUPTPOLARITY_ACTIVE_LOW);

    if(status != VL53L1_ERROR_NONE) return status;
	// Start autonomous ranging
    status = VL53L1_StartMeasurement(pdev);

	return status;
}

VL53L1_Error VL53L1_Init_All()
{
	status = VL53L1_ERROR_NONE;
	FR_TOF = VL53L1_Dev_DEFAULT;
	FR_TOF.i2c_channel = FRONT_RIGHT_TOF_CHANNEL;
	FL_TOF = VL53L1_Dev_DEFAULT;
	FL_TOF.i2c_channel = FRONT_LEFT_TOF_CHANNEL;
	RR_TOF = VL53L1_Dev_DEFAULT;
	RR_TOF.i2c_channel = REAR_RIGHT_TOF_CHANNEL;
	RL_TOF = VL53L1_Dev_DEFAULT;
	RL_TOF.i2c_channel = REAR_LEFT_TOF_CHANNEL;
#if DEBUG_MODE
	/*uint8_t byteData;
	uint16_t wordData;
	VL53L1_RdByte(&RL_TOF, 0x010F, &byteData);
	printf("VL53L1X Model_ID: %02X\n\r", byteData);
	VL53L1_RdByte(&RL_TOF, 0x0110, &byteData);
	printf("VL53L1X Module_Type: %02X\n\r", byteData);
	VL53L1_RdWord(&RL_TOF, 0x010F, &wordData);
	printf("VL53L1X: %02X\n\r", wordData);*/
#endif
	status = VL53L1_CommsInitialise(&FR_TOF, VL53L1_I2C, DOF_I2C_SPEED);
	status += VL53L1_CommsInitialise(&FL_TOF, VL53L1_I2C, DOF_I2C_SPEED);
	status += VL53L1_CommsInitialise(&RR_TOF, VL53L1_I2C, DOF_I2C_SPEED);
	status += VL53L1_CommsInitialise(&RL_TOF, VL53L1_I2C, DOF_I2C_SPEED);
	return status;
}

VL53L1_Error Get_TOF_Distance()
{
	VL53L1_Dev_t *pdev;
	if((TOF_interrupt & FRONT_RIGHT_TOF_FLAG) == FRONT_RIGHT_TOF_FLAG)
	{
		TOF_interrupt &= ~(FRONT_RIGHT_TOF_FLAG);
		pdev = &FR_TOF;
		status = Get_Distance(pdev);
	}
	if((TOF_interrupt & FRONT_LEFT_TOF_FLAG) == FRONT_LEFT_TOF_FLAG)
	{
		TOF_interrupt &= ~(FRONT_LEFT_TOF_FLAG);
		pdev = &FL_TOF;
		status = Get_Distance(pdev);
	}
	if((TOF_interrupt & REAR_RIGHT_TOF_FLAG) == REAR_RIGHT_TOF_FLAG)
	{
		TOF_interrupt &= ~(REAR_RIGHT_TOF_FLAG);
		pdev = &RR_TOF;
		status = Get_Distance(pdev);
	}
	if((TOF_interrupt & REAR_LEFT_TOF_FLAG) == REAR_LEFT_TOF_FLAG)
	{
		TOF_interrupt &= ~(REAR_LEFT_TOF_FLAG);
		pdev = &RL_TOF;
		status = Get_Distance(pdev);
	}

	return status;
}

VL53L1_Error Get_Distance(VL53L1_Dev_t *dev)
{
	VL53L1_RangingMeasurementData_t data;
	status = VL53L1_GetRangingMeasurementData(dev, &data);
	dev->distance =  data.RangeMilliMeter;

	// do something
	//Process_Distance(distance);
	#if DEBUG_MODE
		if(dev->i2c_channel == FRONT_RIGHT_TOF_CHANNEL)
			printf("FR tof:%d mm\r\n", dev->distance);
		else if(dev->i2c_channel == FRONT_LEFT_TOF_CHANNEL)
			printf("FL tof:%d mm\r\n", dev->distance);
		else if(dev->i2c_channel == REAR_RIGHT_TOF_CHANNEL)
			printf("RR tof:%d mm\r\n", dev->distance);
		else if(dev->i2c_channel == REAR_LEFT_TOF_CHANNEL)
			printf("RL tof:%d mm\r\n", dev->distance);
	#endif

	status += VL53L1_ClearInterruptAndStartMeasurement(dev);

	return status;
}
