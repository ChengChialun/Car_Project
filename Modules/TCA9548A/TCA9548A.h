#ifndef __TCA9548A_H
#define __TCA9548A_H
#include "main.h"

#define TCA_ADDR (0x70 << 1)

HAL_StatusTypeDef TCA_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel);
HAL_StatusTypeDef TCA_DisableAll(I2C_HandleTypeDef *hi2c);
#endif
