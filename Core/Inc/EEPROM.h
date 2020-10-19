/*
 * EEPROM.h
 *
 *  Created on: Oct 19, 2020
 *      Author: nicoc
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_ADDRESS_READ		0xA1
#define EEPROM_ADDRESS_WRITE	0xA0

uint8_t readEEPROM(uint8_t pos);
uint8_t writeEEPROM(uint8_t pos);

#endif /* INC_EEPROM_H_ */
