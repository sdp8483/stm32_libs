/*
 * eeprom_24LC02B.c
 *
 *  Created on: Mar 6, 2020
 *      Author: samper
 */

#include "eeprom_24LC02B.h"

uint8_t EEPROM_24LC028_read_uint8(uint8_t data_addr) {
	uint8_t buffer;

	/* Make sure the memory address is not outside of the device address range
	 * 	This should never happen but lets add it just because
	 */
	if (data_addr > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, &buffer, 1, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return buffer;
} // end EEPROM_24LC028_read_uint8()

int8_t EEPROM_24LC028_read_int8(uint8_t data_addr) {
	uint8_t buffer;

	/* Make sure the memory address is not outside of the device address range
	 * 	This should never happen but lets add it just because
	 */
	if (data_addr > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, &buffer, 1, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return (int8_t) buffer;
} // end EEPROM_24LC028_read_int8()

uint16_t EEPROM_24LC028_read_uint16(uint8_t data_addr) {
	uint8_t buffer[2];

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 1) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, buffer, 2, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return (uint16_t) ((buffer[1] << 8) + buffer[0]);
} // end EEPROM_24LC028_read_uint16()

int16_t EEPROM_24LC028_read_int16(uint8_t data_addr) {
	uint8_t buffer[2];

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 1) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, buffer, 2, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return (int16_t) ((buffer[1] << 8) + buffer[0]);
} // end EEPROM_24LC028_read_int16()

uint32_t EEPROM_24LC028_read_uint32(uint8_t data_addr) {
	uint8_t buffer[4];

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 3) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, buffer, 4, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return (uint32_t) (((buffer[3] << 24) + (buffer[2] << 16) + (buffer[1] << 8)) + buffer[0]);
} // end EEPROM_24LC028_read_uint32()

int32_t EEPROM_24LC028_read_int32(uint8_t data_addr) {
	uint8_t buffer[4];

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 3) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, buffer, 4, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return (int32_t) (((buffer[3] << 24) + (buffer[2] << 16) + (buffer[1] << 8)) + buffer[0]);
} // end EEPROM_24LC028_read_int32()

float EEPROM_24LC028_read_float(uint8_t data_addr) {
	union Data {
		uint8_t buffer[4];
		float val;
	} data;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 3) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, data.buffer, 4, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return data.val;
} // end EEPROM_24LC028_read_float()

double EEPROM_24LC028_read_double(uint8_t data_addr) {
	union Data {
		uint8_t buffer[8];
		double val;
	} data;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 7) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, data.buffer, 8, 10) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}

	return data.val;
} // end EEPROM_24LC028_read_double()

uint8_t EEPROM_24LC028_write_uint8(uint8_t data_addr, uint8_t *pData) {
	uint8_t buffer;

	/* Make sure the memory address is not outside of the device address range
	 * 	This should never happen but lets add it just because
	 */
	if ((data_addr) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_uint8(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, pData, 1, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_uint8()

uint8_t EEPROM_24LC028_write_int8(uint8_t data_addr, int8_t *pData) {
	int8_t buffer;

	/* Make sure the memory address is not outside of the device address range
	 * 	This should never happen but lets add it just because
	 */
	if ((data_addr) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_int8(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, (uint8_t*) pData, 1, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_int8()

uint8_t EEPROM_24LC028_write_uint16(uint8_t data_addr, uint16_t *pData) {
	uint16_t buffer;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 1) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_uint16(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, (uint8_t*) pData, 2, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_uint16()

uint8_t EEPROM_24LC028_write_int16(uint8_t data_addr, int16_t *pData) {
	int16_t buffer;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 1) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_int16(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, (uint8_t*) pData, 2, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_int16()

uint8_t EEPROM_24LC028_write_uint32(uint8_t data_addr, uint32_t *pData) {
	uint32_t buffer;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 3) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_uint32(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, (uint8_t*) pData, 4, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_uint32()

uint8_t EEPROM_24LC028_write_int32(uint8_t data_addr, int32_t *pData) {
	int32_t buffer;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 3) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_int32(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, (uint8_t*) pData, 4, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_int32()

uint8_t EEPROM_24LC028_write_float(uint8_t data_addr, float *pData) {
	float buffer;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 3) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_float(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, (uint8_t*) pData, 4, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_float()

uint8_t EEPROM_24LC028_write_double(uint8_t data_addr, double *pData) {
	double buffer;

	/* Make sure the memory address is not outside of the device address range */
	if ((data_addr + 7) > EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();

		return EEPROM_24LC028_ERROR;
	}

	/* First read the address so we make sure we are not writing the same data */
	buffer = EEPROM_24LC028_read_double(data_addr);

	/* Check if data on the EEPROM is the same as what we want to write */
	if (buffer != *pData) {
		while (HAL_I2C_Mem_Write(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, data_addr, 1, (uint8_t*) pData, 8, 10) != HAL_OK) {
			if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
				Error_Handler();
			}
		}

		return EEPROM_24LC028_OK;		// Data stored in EEPROM
	} else {
		return EEPROM_24LC028_NOUP;	// Data was already stored in EEPROM so no update
	}
} // end EEPROM_24LC028_write_double()

void EEPROM_24LC028_memdump(uint8_t *pData, uint8_t len) {
	/* Raise Error if data array to store eeprom dump is smaller then eeprom size*/
	if (len < EEPROM_24LC028_MAX_ADDR) {
		Error_Handler();
	}

	while (HAL_I2C_Mem_Read(&EEPROM_24LC02B_I2C_PORT, EEPROM_24LC028_DEVICE_ADDR, 0, 1, pData, EEPROM_24LC028_MAX_ADDR, 100) != HAL_OK) {
		if (HAL_I2C_GetError(&EEPROM_24LC02B_I2C_PORT) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}
}

