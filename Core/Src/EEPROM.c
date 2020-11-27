/*
 * eeprom.c
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 */


/*
 * EEPROM.c
 *
 *  Created on: Oct 19, 2020
 *      Author: nicoc
 */

#include "main.h"
#include "eeprom.h"

#define	CANT_RESERVADO		3	// 3 alarmas reservadas
#define TIMEOUT				10
#define OK					0
#define	NO_OK				1

extern I2C_HandleTypeDef hi2c1;

extern float customAlarms[CANT_ALARMAS];
uint8_t	customAlarmsInt[CANT_ALARMAS+1]={0};

extern float valuetoSave;

uint8_t readEEPROM( void )
{
	uint16_t eeprom_memadd = EEPROM_MEMADD;

	for(int i = CANT_RESERVADO; i < CANT_ALARMAS; i++){
		if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_READ, eeprom_memadd, I2C_MEMADD_SIZE_16BIT, &customAlarmsInt[i], 1, TIMEOUT) != HAL_OK)
			return EEPROM_ERR;
		eeprom_memadd += EEPROM_SHIFT_16BIT;
		HAL_Delay(5);
	}

	customAlarms[3] = customAlarmsInt[0] + ((float) customAlarmsInt[1] /100);
	customAlarms[4] = customAlarmsInt[2] + ((float) customAlarmsInt[3] /100);
	customAlarms[5] = customAlarmsInt[4] + ((float) customAlarmsInt[5] /100);
	customAlarms[6] = customAlarmsInt[6] + ((float) customAlarmsInt[7] /100);

	return EEPROM_OK;
}
//uint8_t readEEPROM( void )
//{
//	uint16_t eeprom_memadd = EEPROM_MEMADD;
//
//	for(int i = CANT_RESERVADO; i < CANT_ALARMAS; i++){
//		HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_READ, (eeprom_memadd>>8)&0xFF, I2C_MEMADD_SIZE_8BIT, &customAlarms[i], 1, TIMEOUT);
//		if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS_READ, eeprom_memadd&0xFF, I2C_MEMADD_SIZE_8BIT, &customAlarms[i], 1, TIMEOUT) != HAL_OK)
//			return EEPROM_ERR;
//		eeprom_memadd += EEPROM_SHIFT_16BIT;
//		HAL_Delay(5);
//	}
//	return EEPROM_OK;
//}

uint8_t writeEEPROM( void )
{
	uint16_t eeprom_memadd = EEPROM_MEMADD;

	customAlarmsInt[0] = customAlarms[3];
	customAlarmsInt[1] = (customAlarms[3] - (uint8_t) customAlarms[3]) * 100;
	customAlarmsInt[2] = customAlarms[4];
	customAlarmsInt[3] = (customAlarms[4] - (uint8_t) customAlarms[4]) * 100;
	customAlarmsInt[4] = customAlarms[5];
	customAlarmsInt[5] = (customAlarms[5] - (uint8_t) customAlarms[5]) * 100;
	customAlarmsInt[6] = customAlarms[6];
	customAlarmsInt[7] = (customAlarms[6] - (uint8_t) customAlarms[6]) * 100;

	for(int i = CANT_RESERVADO; i < CANT_ALARMAS; i++){
		if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS_WRITE, eeprom_memadd, I2C_MEMADD_SIZE_16BIT, &customAlarmsInt[i], 1, TIMEOUT) != HAL_OK)
			return EEPROM_ERR;
		eeprom_memadd += EEPROM_SHIFT_16BIT;
		HAL_Delay(5);
	}
	return EEPROM_OK;
}

//uint8_t writeEEPROM( void )
//{
//	uint16_t eeprom_memadd = EEPROM_MEMADD;
//
//	for(int i = CANT_RESERVADO; i < CANT_ALARMAS; i++){
//		HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS_WRITE, (eeprom_memadd>>8)&0xFF, I2C_MEMADD_SIZE_8BIT, &baka, 1, TIMEOUT);
//		if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS_WRITE, eeprom_memadd&0xFF, I2C_MEMADD_SIZE_8BIT, &customAlarms[i], 1, TIMEOUT) != HAL_OK)
//			return EEPROM_ERR;
//		eeprom_memadd += EEPROM_SHIFT_16BIT;
//		HAL_Delay(5);
//	}
//	return EEPROM_OK;
//}

void EEPROM_Init( void )
{
//	writeEEPROM();
//	customAlarms[3] = 0;
//	customAlarms[4] = 0;
//	customAlarms[5] = 0;
//	customAlarms[6] = 0;
//	HAL_Delay(1000);
	readEEPROM();
}

void EEPROM_refresh(uint8_t action)
{
	int del_ok = NO_OK;
	int refresh_ok = NO_OK;

	if(action == EEPROM_DELETE){
		for(int i = CANT_RESERVADO; i < CANT_ALARMAS; i++){
			if(customAlarms[i] == EEPROM_EMPTY && i > CANT_RESERVADO){
				del_ok = OK;
				customAlarms[i-1] = EEPROM_EMPTY;
				writeEEPROM();
			}
		}
		if(del_ok == NO_OK){
			customAlarms[CANT_ALARMAS-1] = EEPROM_EMPTY;
			writeEEPROM();
		}
	}
	else{
		for(int i = CANT_RESERVADO; i < CANT_ALARMAS; i++){
			if(customAlarms[i] == EEPROM_EMPTY){
				refresh_ok = OK;
				customAlarms[i] = valuetoSave;
				writeEEPROM();
				break;
			}
		}
		if(refresh_ok == NO_OK){
			// memoria llena - gg ez
		}
	}
}
