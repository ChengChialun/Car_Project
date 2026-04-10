#include "vl53l1/platform/inc/vl53l1_platform.h"
#include "main.h"
#include "i2c.h"

VL53L1_Error VL53L1_Write(uint16_t dev, uint16_t reg, uint8_t *data, uint32_t count)
{
	return HAL_I2C_Mem_Write(&hi2c1,
                            dev,
                            reg,
                            I2C_MEMADD_SIZE_16BIT,
                            data,
                            count,
                            100);
}

VL53L1_Error VL53L1_GetTickCount(VL53L1_DEV Dev, uint32_t *ptick_count_ms) {
    (void)Dev;
    *ptick_count_ms = HAL_GetTick();
    return VL53L1_ERROR_NONE;
}
