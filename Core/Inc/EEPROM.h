/*
 * eeprom.h
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_ADDRESS			0xA0

#define EEPROM_ADDRESS_READ		0xA1
#define EEPROM_ADDRESS_WRITE	0xA0

#define EEPROM_MEMADD			0x0100

#define EEPROM_ERR				0
#define EEPROM_OK				1

#define EEPROM_SHIFT_16BIT		0x02

#define EEPROM_EMPTY			250		// valor que nunca se mide
#define EEPROM_DELETE			2
#define EEPROM_SAVE				3

#define CANT_ALARMAS			7


uint8_t readEEPROM( void );
uint8_t writeEEPROM( void );
void EEPROM_Init( void );
void EEPROM_refresh(uint8_t action);

#endif /* INC_EEPROM_H_ */
