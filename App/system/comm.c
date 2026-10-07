#include <comm.h>

uint8_t comm_raw[FRAME_SIZE];
uint16_t serialize(uint8_t* raw, Comm_Frame_t *cFrame);

uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;	//x^16 + x^15 + x^2 + 1 and LSB-first so 1000 0000 0000 0101 -> 0101 0000 0000 0001 = 0xA001
            else
                crc >>= 1;
        }
    }
    return crc;
}

//Consistent Overhead Byte Stuffing
uint16_t cobs_encode(const uint8_t *input, uint16_t length, uint8_t *output)
{
	uint16_t read_index = 0;
	uint16_t write_index = 1;
	uint16_t code_index = 0;
	uint8_t data_length = 1;	//section data

    while (read_index < length) {
        if (input[read_index] == 0) {	//section end
            output[code_index] = data_length;	//refill length
            data_length = 1;					//reset
            code_index = write_index++;			//move to next section index
        } else {
            output[write_index++] = input[read_index];
            data_length++;
            if (data_length == 0xFF) {		//section max is 255 so need new section
                output[code_index] = data_length;
                data_length = 1;
                code_index = write_index++;
            }
        }
        read_index++;
    }

    output[code_index] = data_length;
    return write_index;
}

uint16_t cobs_decode(const uint8_t *input, uint16_t length, uint8_t *output)
{
    uint16_t read_index = 0;
    uint16_t write_index = 0;

    while (read_index < length) {
        uint8_t data_length = input[read_index];
        if (read_index + data_length > length && data_length != 1)
            return 0; // error

        read_index++;

        for (uint8_t i = 1; i < data_length; i++)
            output[write_index++] = input[read_index++];

        if (data_length != 0xFF && read_index < length)
            output[write_index++] = 0;
    }

    return write_index;
}

void send_packet(Comm_Frame_t *cFrame)
{
	uint8_t encoded[FRAME_SIZE];

	uint16_t idx = serialize(comm_raw, cFrame);

	// CRC
    uint16_t crc = crc16(comm_raw, idx);
    comm_raw[idx++] = crc & 0xFF;
    comm_raw[idx++] = (crc >> 8);

    // COBS encode
    uint16_t enc_len = cobs_encode(comm_raw, idx, encoded);

    // Framing (COBS standard)
    uint8_t zero = 0x00;
    HAL_UART_Transmit(&COMM_UART_HANDLE, &zero, 1, 10);
    HAL_UART_Transmit(&COMM_UART_HANDLE, encoded, enc_len, 100);
    HAL_UART_Transmit(&COMM_UART_HANDLE, &zero, 1, 10);
}

uint16_t serialize(uint8_t* raw, Comm_Frame_t *cFrame)
{
	memset(raw, 0, FRAME_SIZE);

	uint16_t idx = 0;
	raw[idx++] = cFrame->topic;
	raw[idx++] = cFrame->type;
	raw[idx++] = cFrame->len;
	raw[idx++] = cFrame->seq;
	memcpy(&raw[idx], cFrame->payload, cFrame->len);
	idx += cFrame->len;

	return idx;
}
