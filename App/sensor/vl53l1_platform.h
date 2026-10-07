#ifndef __VL53L1_PLATFORM_H
#define __VL53L1_PLATFORM_H
#include "vl53l1_ll_def.h"
#include "vl53l1_platform_log.h"

#define VL53L1_IPP_API
#include "vl53l1_platform_ipp_imports.h"
#include "vl53l1_platform_user_data.h"
#include "vl53l1_api.h"
#include <string.h>
#include <time.h>
#include <math.h>

//************ user define function and variable start ************
//#include "vl53l1x.h"
#include <micro_seconds_control.h>
#include "main.h"
#include "i2c.h"
extern volatile uint16_t Error_code;
#define DOF_I2C_SPEED 				400	//400kHz High speed
#define DISTANCE_THRESHOLD 			500 //500 mm
#define VL53L1X_I2C_HANDLE			hi2c2
extern volatile uint8_t TOF_interrupt;
extern VL53L1_Dev_t RR_TOF, RL_TOF, FR_TOF, FL_TOF;

#define FRONT_RIGHT_TOF_CHANNEL		0
#define FRONT_LEFT_TOF_CHANNEL		1
#define REAR_RIGHT_TOF_CHANNEL		2
#define REAR_LEFT_TOF_CHANNEL		3
#define FRONT_RIGHT_TOF_FLAG 		(1 << 0)
#define FRONT_LEFT_TOF_FLAG 		(1 << 1)
#define REAR_RIGHT_TOF_FLAG 		(1 << 2)
#define REAR_LEFT_TOF_FLAG 			(1 << 3)
VL53L1_Error VL53L1_Init_All();
VL53L1_Error Get_TOF_Distance();
//************ user define function and variable end ************

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
VL53L1_Error VL53L1_WrByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t       data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
VL53L1_Error VL53L1_WrWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t      data);
/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
VL53L1_Error VL53L1_WrDWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint32_t      data);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
VL53L1_Error VL53L1_RdByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
VL53L1_Error VL53L1_RdWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t     *pdata);
/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint32_t     *pdata);
/** @brief VL53L1_WaitMs() definition.\n
 * To be implemented by the developer
 */
VL53L1_Error VL53L1_WaitMs(
		VL53L1_Dev_t *pdev,
		int32_t       wait_ms);

VL53L1_Error VL53L1_WaitUs(
		VL53L1_Dev_t *pdev,
		int32_t wait_us);
/**
 * @brief Register "wait for value" polling routine
 *
 * Port of the V2WReg Script function  WaitValueMaskEx()
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @param[in]   timeout_ms    : timeout in [ms]
 * @param[in]   index         : uint16_t register index value
 * @param[in]   value         : value to wait for
 * @param[in]   mask          : mask to be applied before comparison with value
 * @param[in]   poll_delay_ms : polling delay been each read transaction in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitValueMaskEx(
		VL53L1_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms);

/*
 * @brief Gets current system tick count in [ms]
 *
 * @return  time_ms : current time in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

/*VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptime_ms);*/
VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms);
#endif
