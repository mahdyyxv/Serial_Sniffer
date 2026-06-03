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
#include "config.h"
#include "rs485_parser.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

static void vTimer2_Interrupt_Handler(void);
static void vLogRs485Frame(void);

bool TimerTrigged = false;

/* DMA receive buffer — shared with HAL_UART_MspInit and rs485_parser.c.
   Size must match DMA_BUF_SIZE in rs485_parser.c (= RS485_MAX_FRAME_SIZE). */
uint8_t SerialBuffer[512];

void vMainInitFunc(void){
    HAL_TIM_Base_Start_IT(&htim2);
    vConfig_Init();
    vRs485Parser_Init(&psConfig_Get()->rs485);
}

void vMainLoopFunc(void){
    while(1){
        MX_USB_HOST_Process();
        vTimer2_Interrupt_Handler();
        vSDC_Engine();
        vUsbEngine();
        if(bRs485Parser_FrameReady()){
            vLogRs485Frame();
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM2){
        TimerTrigged = true;
    }
}

static void vTimer2_Interrupt_Handler(void){
    if(!TimerTrigged){ return; }
    TimerTrigged = false;
    vSDC_TimerTick();
    vRs485Parser_TimerTick();
}

/* Retrieve the waiting RS485 frame, format it, and queue it for SD write. */
static void vLogRs485Frame(void){
    sRs485Frame  frame;
    uint8_t      date[50];
    uint8_t      payload[SDC_DATA_MAX_SIZE];

    vRs485Parser_GetFrame(&frame);
    uint8_t  dateLen    = u8RTC_GetDateTimeString(date);
    uint16_t payloadLen = u16Rs485Parser_FormatLog(&frame, payload, (uint16_t)sizeof(payload));

    bSDC_Write(date, dateLen, payload, payloadLen);
}

