#include <storage_control.h>

/**
 * @brief Flash Memory Mapping Configuration
 * * Overview:
 * - Total number of sectors depends on the specific STM32 model; this is a baseline configuration.
 * - Sector Size: Based on the STM32F405, the initial sectors are typically 16KB.
 * * Memory Layout:
 * -------------------------------------------------------------------------
 * [Range]          [Usage]         [Description]
 * -------------------------------------------------------------------------
 * Sector 0 ~ 1    | EEPROM (Cfg)  | Stores system parameters, PID settings, and calibration values.
 * |               | Uses a dual-backup (Ping-Pong) mechanism.
 * -------------------------------------------------------------------------
 * Sector 2 ~ 200  | LOG (Data)    | Stores vehicle trajectory and sensor snapshots.
 * |               | Managed via a Circular (Ring) Buffer.
 * -------------------------------------------------------------------------
 * Sector 201 ~    | Reserved      | Reserved for future expansion, such as OTA updates or new features.
 * -------------------------------------------------------------------------
 */
//#define CONFIG_ADDR   			0x000000  	// external flash start
#define FLASH_MAGIC			  		0x43464731  // "CFG1"
#define STATE_EMPTY   				0xFFFFFFFF
#define STATE_WRITING 				0xAAAAAAAA
#define STATE_VALID   				0x88888888
#define SECTOR_SIZE   				4096
#define CONFIG_MAX_RECORDS 			(SECTOR_SIZE / sizeof(Config_t))

//config
#define SECTOR_A_ADDR 				0x000000	//sector 0
#define SECTOR_B_ADDR 				0x001000	//sector 1
//uint32_t lastA = 0, lastB = 0;
//Config_t tmp;
//uint8_t active_sector = 0; // 0=A, 1=B
//uint32_t current_index = 		0;
uint32_t cur_count;

//EEPROM_Data_t current_data;
typedef enum{
	None,
	sector_A,
	sector_B,
}config_active_sector;
config_active_sector config_active;

//log
#define LOG_START_SECTOR 	2
#define LOG_END_SECTOR   	200
#define LOG_SIZE 			sizeof(Log_t)
uint32_t log_sector = LOG_START_SECTOR;
uint32_t log_offset = 0;

extern CRC_HandleTypeDef hcrc;

int8_t Varify_Conf(uint32_t addr);
uint32_t Calculate_Config_Checksum(Config_t *conf);
uint32_t Calculate_Log_Checksum(Log_t *log);
int16_t replace_slot_index_A, replace_slot_index_B;
Config_t* FLASH_Config_Read();
int32_t Find_Latest_In_Sector(uint32_t addr_base, Config_t *out, int16_t *replase_slot);
void Load_Default(Config_t *data);
extern uint64_t time_offset_us;
int32_t counterA, counterB;
Config_t dataA, dataB;
extern Config_Record_t last_conf_record;
extern Config_t cur_conf;
/*
uint8_t Simulate_EEPROM_ReadLatest(EEPROM_Data_t *out)
{
    EEPROM_Data_t tmp;
    uint32_t addr = CONFIG_ADDR;
    uint32_t max_counter = 0;
    uint8_t found = 0;

    for (int i = 0; i < MAX_RECORDS; i++)   // 假設最多100筆
    {
        W25QXX_Read(addr, (uint8_t*)&tmp, sizeof(tmp));

        if (tmp.magic != CONFIG_MAGIC)
            break;

        if (tmp.counter >= max_counter)
        {
            max_counter = tmp.counter;
            *out = tmp;
            found = 1;
        }

        addr += sizeof(tmp);
    }

    return found;
}

void Simulate_EEPROM_Write(EEPROM_Data_t *data)
{
    EEPROM_Data_t tmp;
    uint32_t addr = CONFIG_ADDR;

    // find last record
    for (int i = 0; i < MAX_RECORDS; i++)
    {
        W25QXX_Read(addr, (uint8_t*)&tmp, sizeof(tmp));

        if (tmp.magic != CONFIG_MAGIC)
            break;

        addr += sizeof(tmp);
    }

    // 設定資料
    data->magic = CONFIG_MAGIC;
    data->counter++;

    // 如果滿了 → erase
    if (addr >= CONFIG_ADDR + SECTOR_SIZE)
    {
        W25QXX_EraseSector(CONFIG_ADDR);
        addr = CONFIG_ADDR;
        data->counter = 0;
    }

    W25QXX_Write(addr, (uint8_t*)data, sizeof(Config_t));
}


void EEPROM_Init(void)
{
    EEPROM_Data_t tmp;
    uint32_t addr = EEPROM_ADDR;

    current_index = 0;

    for (int i = 0; i < MAX_RECORDS; i++)
    {
        W25QXX_Read(addr, (uint8_t*)&tmp, sizeof(tmp));

        if (tmp.magic != EEPROM_MAGIC)
            break;

        current_data = tmp;
        current_index++;

        addr += sizeof(tmp);
    }
}

Config_t* FLASH_Config_Read()
{
    if (config_active == sector_A)//read sector A
    {
        W25QXX_Read(SECTOR_A_ADDR, (uint8_t*)&cur_conf, sizeof(cur_conf));
    }
    else if(config_active == sector_B)//read sector B
    {
        W25QXX_Read(SECTOR_B_ADDR, (uint8_t*)&cur_conf, sizeof(cur_conf));
    }
    else
    {
    	Load_Default(&cur_conf);
    }

    return &cur_conf;
}*/

void Get_Cur_Conf(Config_t* conf)
{
	if(config_active == sector_A)
		*conf = dataA;
	else if(config_active == sector_B)
		*conf = dataB;
}

void Config_Parser(Config_t *conf, Config_Record_t *data ,int8_t rw){
	uint32_t ver = MAKE_VERSION(VER_MAJOR, VER_MINOR, VER_PATCH);
	if( __READ == rw )//read -> return Config_Record_t *data
	{
		switch(conf->version)
		{
			case VERSION_1_0_0:
				 if(conf->length <= PAYLOAD_SIZE && conf->length <= sizeof(*data))
				 {
				     memcpy(data, conf->payload, conf->length);
				 }
				 else
				 {
				     memset(data, 0, sizeof(*data));
				 }
			break;
			default:
				memset(data, 0, sizeof(*data));
			break;
		}
	}
	else//write -> update Config_t *conf
	{
		conf->version = ver;
		if(sizeof(*data) <= PAYLOAD_SIZE)
		{
			memcpy(conf->payload, data, sizeof(*data));
			conf->length = sizeof(*data);
		}
	}
}

int32_t Find_Latest_In_Sector(uint32_t addr_base, Config_t *out, int16_t *replase_slot)
{
    Config_t temp;
    int i;
    uint32_t addr_tmp;
    int32_t max_counter = -1;

    for (i = 0; i < CONFIG_MAX_RECORDS; i++)
    {
    	addr_tmp = addr_base + i * sizeof(Config_t);

        W25QXX_Read(addr_tmp, (uint8_t*)&temp, sizeof(temp));

        if ( (temp.state == STATE_EMPTY) )
        {
			if( -1 == *replase_slot)
				*replase_slot = i;
			break;
        }
		if ( (temp.magic != FLASH_MAGIC) ||
			 (temp.state != STATE_VALID) ||
			 (Calculate_Config_Checksum(&temp) != temp.checksum)
			)
		{
			//if( -1 == *replase_slot)
			//	*replase_slot = i;
			continue;
		}
        //if (temp.counter >= max_counter)
        //{
			//find out last valid record in current sector
            max_counter = temp.counter;
            *out = temp;
        //}
    }

    if(i == CONFIG_MAX_RECORDS)	// sector full
    	*replase_slot = -1;

    return max_counter;
}

Config_Record_t* FLASH_Config_Init(void)
{
	replace_slot_index_A = -1;
	replace_slot_index_B = -1;
	counterA = Find_Latest_In_Sector(SECTOR_A_ADDR, &dataA, &replace_slot_index_A);
	counterB = Find_Latest_In_Sector(SECTOR_B_ADDR, &dataB, &replace_slot_index_B);

    if (counterA >= 0 && counterB >= 0)
    {
    	config_active = (dataA.counter > dataB.counter) ? sector_A : sector_B;
    }
    else if (counterA >= 0)
    {
    	config_active = sector_A;
    }
    else if (counterB >= 0)
    {
    	config_active = sector_B;
    }
    else
    {
    	config_active = None;
    	Load_Default(&cur_conf);
    }

    if(config_active == sector_A)
    	Config_Parser(&dataA, &last_conf_record, __READ);
    else if (config_active == sector_B)
    	Config_Parser(&dataB, &last_conf_record, __READ);

    return &last_conf_record;
}

void FLASH_Config_Write(Config_Record_t *data)
{
    uint32_t addr = SECTOR_A_ADDR;
    Config_t new_data;

    switch(config_active)
    {
    	case sector_A:
    		if( replace_slot_index_A  == -1)//sector A full
    		{
    			addr = SECTOR_B_ADDR;
    			W25QXX_EraseSector(addr);
    			W25QXX_Write(addr, (uint8_t*)&dataA, sizeof(Config_t)); //write last data in new sector
    			addr += sizeof(Config_t);
    			replace_slot_index_B = 1;
    			counterB = counterA + 1;
    		}
    		else
    			addr = SECTOR_A_ADDR + replace_slot_index_A * sizeof(Config_t);
    		new_data.counter = ++counterA;
    	break;
    	case sector_B:
    		if( replace_slot_index_B  == -1)
    		{
    			addr = SECTOR_A_ADDR;
    			W25QXX_EraseSector(addr);
    			W25QXX_Write(addr, (uint8_t*)&dataB, sizeof(Config_t)); //write last data in new sector
    			addr += sizeof(Config_t);
    			replace_slot_index_A = 1;
    			counterA = counterB + 1;
    		}
    		else
    			addr = SECTOR_B_ADDR + replace_slot_index_B * sizeof(Config_t);
    		new_data.counter = ++counterB;
    	break;
    	case None:
    		new_data.counter = 1;
    		addr = SECTOR_A_ADDR + sizeof(Config_t);
    	break;
    }

    new_data.magic = FLASH_MAGIC;
    new_data.version = MAKE_VERSION(VER_MAJOR, VER_MINOR, VER_PATCH);
    Config_Parser(&new_data, data, __WRITE);
    new_data.checksum = Calculate_Config_Checksum(&new_data);
    new_data.state = STATE_WRITING;
    W25QXX_Write(addr, (uint8_t*)&new_data, sizeof(Config_t));
    if( addr >= SECTOR_B_ADDR )
    {
    	replace_slot_index_B++;
    	if(replace_slot_index_B >= CONFIG_MAX_RECORDS) //sector B is full
    		replace_slot_index_B = -1;
    }
    else
    {
    	replace_slot_index_A++;
    	if(replace_slot_index_A >= CONFIG_MAX_RECORDS)
    		replace_slot_index_A = -1;
    }

    if( 0 == Varify_Conf(addr))//write successful
    {
    	//update new conf
    	if(addr >= SECTOR_B_ADDR)
    	{
    		dataB = new_data;
    		config_active = sector_B;
    	}
    	else
    	{
    		dataA = new_data;
    		config_active = sector_A;
    	}
    }
}

void Load_Default(Config_t *data)
{
	data->magic = FLASH_MAGIC;
	data->version = MAKE_VERSION(VER_MAJOR, VER_MINOR, VER_PATCH);
	data->counter = 1;

	last_conf_record.imu_offset[0] = 0; //accel
	last_conf_record.imu_offset[1] = 0; //gyro
	last_conf_record.imu_offset[2] = 0;	//Magnetometer
	last_conf_record.pid_kp = 0;
	last_conf_record.pid_ki = 0;
	last_conf_record.pid_kd = 0;
	memcpy(data->payload, &last_conf_record, sizeof(last_conf_record));

	data->length = sizeof(last_conf_record);
	data->state = STATE_WRITING;
	data->checksum = Calculate_Config_Checksum( data );

	W25QXX_EraseSector(SECTOR_A_ADDR);
	W25QXX_Write(SECTOR_A_ADDR, (uint8_t*)data, sizeof(Config_t));
	Varify_Conf(SECTOR_A_ADDR);
}

int8_t Varify_Conf(uint32_t addr)
{
	Config_t conf_varify;
	W25QXX_Read(addr, (uint8_t*)&conf_varify, sizeof(conf_varify));

	if( (conf_varify.magic == FLASH_MAGIC) && (conf_varify.state == STATE_WRITING) &&
	    conf_varify.checksum == Calculate_Config_Checksum(&conf_varify))
	{
		conf_varify.state = STATE_VALID;
		W25QXX_Write(addr + offsetof(Config_t, state), (uint8_t*)&conf_varify.state, sizeof(uint32_t));
		return 0;
	}

	return 1;	//error
}

void Log_Init(void)
{
    Log_t tmp;

    log_sector = LOG_START_SECTOR;
    log_offset = 0;

    for (uint32_t s = LOG_START_SECTOR; s <= LOG_END_SECTOR; s++)
    {
        for (uint32_t i = 0; i < SECTOR_SIZE; i += LOG_SIZE)
        {
            W25QXX_Read(s * SECTOR_SIZE + i, (uint8_t*)&tmp, sizeof(tmp));

            if (tmp.magic != FLASH_MAGIC)
            {
                log_sector = s;
                log_offset = i;
                return;
            }
        }
    }

    if(tmp.magic == FLASH_MAGIC && tmp.checksum == Calculate_Log_Checksum(&tmp))
    	time_offset_us = tmp.log_data.tick;
}

void Log_Write(Log_t *log)
{
    log->magic = FLASH_MAGIC;
    log->log_data.tick = Get_SystemTime_Us();

    uint32_t addr = log_sector * SECTOR_SIZE + log_offset;

    W25QXX_Write(addr, (uint8_t*)log, sizeof(Log_t));

    log_offset += sizeof(Log_t);

    // change sector
    if (log_offset + sizeof(Log_t) > SECTOR_SIZE)
    {
        log_offset = 0;
        log_sector++;

        if (log_sector > LOG_END_SECTOR)
        {
            log_sector = LOG_START_SECTOR;
        }

        W25QXX_EraseSector(log_sector * SECTOR_SIZE);
    }
}

void Log_ReadAll(void)
{
    Log_t tmp;

    for (uint32_t s = LOG_START_SECTOR; s <= LOG_END_SECTOR; s++)
    {
        for (uint32_t i = 0; i < SECTOR_SIZE; i += LOG_SIZE)
        {
            W25QXX_Read(s * SECTOR_SIZE + i, (uint8_t*)&tmp, sizeof(tmp));

            if (tmp.magic != FLASH_MAGIC)
                return;

            // do something：
            // UART print / graph / analysis
        }
    }
}

// Calculate Checksum
uint32_t Calculate_Log_Checksum(Log_t *log) {
	__HAL_CRC_DR_RESET(&hcrc);
	return HAL_CRC_Calculate(&hcrc, (uint32_t *)log, (sizeof(Log_t) - sizeof(log->checksum)) / 4);
}

uint32_t Calculate_Config_Checksum(Config_t *conf) {
	__HAL_CRC_DR_RESET(&hcrc);
	return HAL_CRC_Calculate(&hcrc, (uint32_t *)conf, (sizeof(Config_t) - sizeof(conf->state) - sizeof(conf->checksum)) / 4);
}
