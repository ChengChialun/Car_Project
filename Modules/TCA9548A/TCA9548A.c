#include "TCA9548A.h"

HAL_StatusTypeDef TCA_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel)
{
    if (channel > 7) return HAL_ERROR;

    uint8_t data = (1 << channel);

    return HAL_I2C_Master_Transmit(
        hi2c,
        TCA_ADDR,
        &data,
        1,
        100
    );
}

HAL_StatusTypeDef TCA_DisableAll(I2C_HandleTypeDef *hi2c)
{
    uint8_t data = 0x00;

    return HAL_I2C_Master_Transmit(
        hi2c,
        TCA_ADDR,
        &data,
        1,
        100
    );
}
