/*
 * password.c
 *
 *  Created on: May 29, 2026
 *      Author: ahmed mahdy
 */
#include <stdint.h>
#include <stdlib.h>
#include "DataTypes.h"
#include "password.h"
#include "eeprom.h"

void vWritePassword(uint8_t *u8passwd){
	udtEEprom_Write(EEPROM_ADDRESS, PASSWORD_START_BYTE, u8passwd, (255-PASSWORD_START_BYTE));
}
uint32_t u32ReadPassword(void){
	uint8_t u8passwd[4] = "";
	udtEEprom_Read(EEPROM_ADDRESS, PASSWORD_START_BYTE, u8passwd, (255-PASSWORD_START_BYTE));
	return atoi(u8passwd);
}
