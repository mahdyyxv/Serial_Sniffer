/*
 * rtc.h
 *
 *  Created on: May 30, 2026
 *      Author: ahmed mahdy
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

extern RTC_HandleTypeDef hrtc;

bool bRTC_SetDateTime(RTC_TimeTypeDef *Time, RTC_DateTypeDef *Date);

void vRTC_GetDateTime(RTC_TimeTypeDef *Time, RTC_DateTypeDef *Date);
uint8_t u8RTC_GetDateTimeString(uint8_t *Buf);


#endif /* INC_RTC_H_ */
