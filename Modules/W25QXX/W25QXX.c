#include "W25QXX.h"

uint8_t W25QXX_ReadStatus(void)
{
    uint8_t cmd = CMD_READ_STATUS;
    uint8_t status;

    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

    return status;
}

void W25QXX_WaitBusy(void)
{
    while (W25QXX_ReadStatus() & 0x01);
}

void W25QXX_WriteEnable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

void W25QXX_EraseSector(uint32_t addr)
{
    uint8_t cmd[4];

    W25QXX_WriteEnable();

    cmd[0] = CMD_SECTOR_ERASE_4KB;
    cmd[1] = addr >> 16;
    cmd[2] = addr >> 8;
    cmd[3] = addr;

    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

    W25QXX_WaitBusy();
}

//page write
void W25QXX_Write(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint32_t remain;
    uint32_t offset;

    while (len > 0)
    {
        offset = addr % 256;
        remain = 256 - offset;

        if (len < remain) remain = len;

        W25QXX_WriteEnable();

        uint8_t cmd[4];
        cmd[0] = CMD_PAGE_PROGRAM;
        cmd[1] = addr >> 16;
        cmd[2] = addr >> 8;
        cmd[3] = addr;

        HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
        HAL_SPI_Transmit(&hspi1, data, remain, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

        W25QXX_WaitBusy();

        addr += remain;
        data += remain;
        len -= remain;
    }
}

void W25QXX_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];

    cmd[0] = CMD_READ_DATA;
    cmd[1] = addr >> 16;
    cmd[2] = addr >> 8;
    cmd[3] = addr;

    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

void JEDEC_Read(uint8_t *buf, uint32_t len)
{
	uint8_t cmd = CMD_READ_JEDEC_ID;
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

