/*
 * rtc.c
 *
 *  Created on: May 30, 2026
 *      Author: ahmed mahdy
 */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <memory.h>
#include <stdbool.h>
#include "rtc.h"
#include "DataTypes.h"


const static uint8_t weekDays[7][4] = {
		{"Mon\0"},
		{"Tue\0"},
		{"Wed\0"},
		{"Thu\0"},
		{"Fri\0"},
		{"Sat\0"},
		{"Sun\0"}
};


bool bRTC_SetDateTime(RTC_TimeTypeDef *Time, RTC_DateTypeDef *Date){
	bool DateSuccess = true, TimeSucces = true;
	if(Time == NULL)
	{

	}else{
		HAL_StatusTypeDef status = HAL_RTC_SetTime(&hrtc, Time, RTC_FORMAT_BIN);
		if(status != HAL_OK){
			TimeSucces = false;
		}
	}
	if(Date == NULL){

	}else{
		HAL_StatusTypeDef status = HAL_RTC_SetDate(&hrtc, Date, RTC_FORMAT_BIN);
		if(status != HAL_OK){
			DateSuccess = false;
		}
	}
	return (DateSuccess && TimeSucces);
}

void vRTC_GetDateTime(RTC_TimeTypeDef *Time, RTC_DateTypeDef *Date){
	if(Time == NULL)
	{

	}else{
		HAL_RTC_GetTime(&hrtc, Time, RTC_FORMAT_BIN);
	}
	if(Date == NULL){

	}else{
		HAL_RTC_GetDate(&hrtc, Date, RTC_FORMAT_BIN);
	}
}
uint8_t u8RTC_GetDateTimeString(uint8_t *Buf){
	uint8_t length = 0;
	RTC_DateTypeDef Date;
	RTC_TimeTypeDef Time;

	if( Buf != NULL ){
		vRTC_GetDateTime(&Time, &Date);
		Time.SubSeconds = (((Time.SecondFraction - Time.SubSeconds) * 1000U) / (Time.SecondFraction + 1U));
		if(Date.Year < 10){
			length = snprintf(Buf, 50, "%d:%d:%d.%d - %s %d/%d/200%d",
					Time.Hours, Time.Minutes, Time.Seconds, Time.SubSeconds,
					&weekDays[Date.WeekDay][0], Date.Date, Date.Month, Date.Year
			);
		}else{
			length = snprintf(Buf, 50, "%d:%d:%d.%d - %s %d/%d/20%d",
								Time.Hours, Time.Minutes, Time.Seconds, Time.SubSeconds,
								&weekDays[Date.WeekDay-1][0], Date.Date, Date.Month, Date.Year
						);
		}
//		memcpy(Buf+length, "\r\n",2);
//		length += 2;
	}else{
		length = 0;
	}
	return length;
}
