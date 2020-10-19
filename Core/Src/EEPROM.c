/*
 * EEPROM.c
 *
 *  Created on: Oct 19, 2020
 *      Author: nicoc
 */

#include "main.h"
#include "EEPROM.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t readEEPROM(uint8_t pos)
{
	// aca leo
//	HAL_I2C_Mem_Read(hi2c1, EEPROM_ADDRESS_READ, vuela, 1, Rec_Data, 6, i2c_timeout);
}


uint8_t writeEEPROM(uint8_t pos)
{
	// aca escribo
}
