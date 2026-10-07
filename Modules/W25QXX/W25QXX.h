#ifndef __W25QXX_H
#define __W25QXX_H
#include "main.h"
#include "spi.h"

#define CMD_WRITE_ENABLE     		0x06
#define CMD_READ_STATUS      		0x05
#define CMD_PAGE_PROGRAM     		0x02
#define CMD_READ_DATA        		0x03
#define CMD_SECTOR_ERASE_4KB     	0x20
#define CMD_BLOCK_ERASE_64KB     	0xD8
#define CMD_READ_JEDEC_ID			0x9F

uint8_t W25QXX_ReadStatus(void);
void W25QXX_WaitBusy(void);
void W25QXX_WriteEnable(void);
void W25QXX_EraseSector(uint32_t addr);
void W25QXX_Write(uint32_t addr, uint8_t *data, uint32_t len);
void W25QXX_Read(uint32_t addr, uint8_t *buf, uint32_t len);
void JEDEC_Read(uint8_t *buf, uint32_t len);
#endif
