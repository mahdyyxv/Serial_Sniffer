/*
 * mainHelper.c
 *
 *  Created on: May 29, 2026
 *      Author: ahmed mahdy
 */
#include <stdbool.h>
#include "mainHelper.h"
#include "main.h"
#include "string.h"
#include "usb_device.h"
#include "usb_host.h"
#include "DataTypes.h"
#include "eeprom.h"
#include "password.h"
#include "rtc.h"
#include "parse_usb.h"
#include "sdc.h"
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
static void vTimer2_Interrupt_Handler(void);
bool TimerTrigged = false;

uint8_t SerialBuffer[256];

void vMainInitFunc(void){
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_UART_MspInit(&huart2);
}

void vMainLoopFunc(void){
    while(1){
        MX_USB_HOST_Process();
        vTimer2_Interrupt_Handler();
        vSDC_Engine();
        vUsbEngine();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM2){
        TimerTrigged = true;
    }
}

static void vTimer2_Interrupt_Handler(void){
    static uint16_t cnt = 0;
    if(!TimerTrigged){
        return;
    }else{
        cnt++;
        vSDC_TimerTick();
        uint8_t date[50];
        const uint8_t buff[] = {1,2,3,4,5,6};
        if(cnt == 1000){
			uint8_t len = u8RTC_GetDateTimeString(date);
			bSDC_Write(date, len, buff, sizeof(buff));
            HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
            cnt = 0;
        }
        TimerTrigged = false;
    }
}

