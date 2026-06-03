/*
 * password.h
 *
 *  Created on: May 29, 2026
 *      Author: ahmed mahdy
 */

#ifndef INC_PASSWORD_H_
#define INC_PASSWORD_H_

#define PASSWORD_START_BYTE (uint8_t)(251)
#define DEFAULT_PASSWORD 	(uint32_t)(1234)

void vWritePassword(uint8_t *u8passwd);
uint32_t u32ReadPassword(void);

#endif /* INC_PASSWORD_H_ */
