/*
 * parse_usb.c
 *
 *  Created on: May 30, 2026
 *      Author: ahmed mahdy
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "parse_usb.h"
#include "rtc.h"
#include "password.h"
#include "config.h"



/*
 * Config payload layout (10 bytes):
 *   [0] baud_rate   (eBaudRate enum)
 *   [1] stop_bits   (eStopBits enum)
 *   [2] parity      (eParity enum)
 *   [3] protocol    (eRs485Protocol enum)
 *   [4] log_dir     (eLogDirection enum)
 *   [5..9] params.bytes[0..4]
 */
#define CFG_PAYLOAD_SIZE (10u)

/********************************* User Defined Types *********************************/
typedef enum{
	IDLE = 0,
	PARSING,
	RESPONDING,
	PROCESS_ERROR,
}eUsbState;

typedef enum{
	NO_REQUEST = 0,
	GET_DATE_TIME,
	SET_DATE_TIME,
	GET_PASSWORD,
	SET_PASSWORD,
	GET_CONFIG,
	SET_CONFIG,
	NO_OF_REQUESTS,
}eRequests;

typedef bool (*sRequestsHandlers)(void);

/********************************* static Functions Definitions *********************************/
static bool bParse();
static bool bUsbResponce(void);
static bool ProcessGetDateTime(void);
static bool ProcessSetDateTime(void);
static bool ProcessGetPassword(void);
static bool ProcessSetPassword(void);
static bool ProcessGetConfig(void);
static bool ProcessSetConfig(void);
static uint8_t Checksum8_XOR(const uint8_t *data, size_t len);
static bool VerifyChecksum(const uint8_t *frame, size_t len);
/********************************* Variables Definitions *********************************/
const static sRequestsHandlers RquestsFunctions[NO_OF_REQUESTS] = {
		NULL,
		ProcessGetDateTime,
		ProcessSetDateTime,
		ProcessGetPassword,
		ProcessSetPassword,
		ProcessGetConfig,
		ProcessSetConfig,
};

static 			uint8_t 	*ptrFrameDataStart 	          = NULL;
static 			eUsbState 	state 			              = IDLE;
static const 	uint8_t 	preamble[4] 	              = {0xAA, 0xBB, 0xCC, 0xDD};
static 			eRequests	request			              = NO_REQUEST;
static 			uint8_t 	*framePtr		              = NULL;
static 			uint16_t	u16FrameLenght	              = 0;
static 			uint8_t		u8TxFrame[APP_TX_DATA_SIZE]   = {0};

/********************************* Start of implementation *********************************/

/**
 * @brief  Validates the received frame: checks minimum length, verifies checksum,
 *         matches the 4-byte preamble, and extracts the request code and payload pointer.
 * @retval true  - frame is valid; `request` and `ptrFrameDataStart` are set.
 * @retval false - frame is too short, checksum mismatch, or preamble mismatch.
 */
static bool bParse(){
	bool correct = true;
	uint8_t i = 0;
	if(u16FrameLenght >= 5 && VerifyChecksum(framePtr, u16FrameLenght)) /*5 = 4bytes preamble + request (some requests don't have data)*/
	{
		for(i = 0; i < 4; i++){
			if(preamble[i] != framePtr[i]){
				correct = false;
			}
		}
		if(correct){
			request 		= framePtr[i++];  /* post-increment: i advances past the request byte, leaving i at payload start */
			ptrFrameDataStart 	= &framePtr[i];
		}
	}
	else{
		correct = false;
	}
	return correct;
}

/**
 * @brief  Dispatches the current request to its handler via the lookup table.
 * @retval true  - handler executed successfully.
 * @retval false - no handler registered for the current request code.
 */
static bool bUsbResponce(void){
	bool returnVal = false;
	if((request < NO_OF_REQUESTS) && (RquestsFunctions[request] != NULL)){
		returnVal = RquestsFunctions[request]();
	}
	return returnVal;
}

/**
 * @brief  Registers an incoming USB CDC frame for processing and triggers the state machine.
 * @param  ptrFrame     Pointer to the received raw frame buffer.
 * @param  FrameLength  Number of bytes in the frame.
 */
void vFrameParseStart(uint8_t *ptrFrame, uint16_t FrameLength){
	framePtr = ptrFrame;
	u16FrameLenght = FrameLength;
	state = PARSING;
}

/**
 * @brief  USB request processing state machine. Must be called repeatedly from the main loop.
 *         Transitions: IDLE -> PARSING -> RESPONDING -> IDLE, with PROCESS_ERROR on any failure.
 */
void vUsbEngine(void){
	switch(state){
		case PARSING:
			if(bParse()){
				state = RESPONDING;
			}else{
				state = PROCESS_ERROR;
			}
			break;
		case RESPONDING:
			if(bUsbResponce()){
				state = IDLE;
			}else{
				state = PROCESS_ERROR;
			}
			break;
		case PROCESS_ERROR:
			/**
			 * Maybe we will have some recovery mechanism
			 * So we will let it now as it is
			 * but maybe in the future we will split it
			 * */
			state = IDLE;
			break;
		case IDLE:
			break;
		default   :
			state = IDLE;
			break;
	}
}

/**
 * @brief  Handles GET_DATE_TIME request. Reads current RTC date/time and transmits it over USB CDC.
 * @retval true always.
 */
static bool ProcessGetDateTime(void){
	int length = u8RTC_GetDateTimeString(u8TxFrame);
	CDC_Transmit_FS(u8TxFrame, length);

	return true;
}

/**
 * @brief  Handles SET_DATE_TIME request. Parses HH:MM:SS DD:MM:YY:WD from the frame payload,
 *         applies it to the RTC, and responds with "ok" over USB CDC.
 * @retval true  - RTC updated successfully.
 * @retval false - RTC HAL write failed.
 */
static bool ProcessSetDateTime(void){
	RTC_DateTypeDef Date;
	RTC_TimeTypeDef Time;
	bool status = false;
	uint8_t cnt = 0;
	Time.Hours 			= ptrFrameDataStart[cnt++];
	Time.Minutes 		= ptrFrameDataStart[cnt++];
	Time.Seconds		= ptrFrameDataStart[cnt++];
	Time.SubSeconds		= 0;
	Time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	Time.StoreOperation = RTC_STOREOPERATION_RESET;
	
	Date.Date       	= ptrFrameDataStart[cnt++];
	Date.Month			= ptrFrameDataStart[cnt++];
	Date.Year			= ptrFrameDataStart[cnt++];
	Date.WeekDay		= ptrFrameDataStart[cnt++];
	status = bRTC_SetDateTime(&Time, &Date);
	memcpy(u8TxFrame, "ok", 2);
	CDC_Transmit_FS(u8TxFrame, 2);

	return status;
}

static bool ProcessGetPassword(void){
	itoa(u32ReadPassword(), u8TxFrame, 10);
	CDC_Transmit_FS(u8TxFrame, 4);
	return true;
}
static bool ProcessSetPassword(void){
	vWritePassword(ptrFrameDataStart);
	memcpy(u8TxFrame, "ok", 2);
	CDC_Transmit_FS(u8TxFrame, 2);

	return true;
}


static bool ProcessGetConfig(void){
	const sDeviceConfig *cfg = psConfig_Get();
	uint8_t cnt = 0u;
	u8TxFrame[cnt++] = (uint8_t)cfg->rs485.baud_rate;
	u8TxFrame[cnt++] = (uint8_t)cfg->rs485.stop_bits;
	u8TxFrame[cnt++] = (uint8_t)cfg->rs485.parity;
	u8TxFrame[cnt++] = (uint8_t)cfg->rs485.protocol;
	u8TxFrame[cnt++] = (uint8_t)cfg->rs485.log_dir;
	u8TxFrame[cnt++] = cfg->rs485.params.bytes[0u];
	u8TxFrame[cnt++] = cfg->rs485.params.bytes[1u];
	u8TxFrame[cnt++] = cfg->rs485.params.bytes[2u];
	u8TxFrame[cnt++] = cfg->rs485.params.bytes[3u];
	u8TxFrame[cnt++] = cfg->rs485.params.bytes[4u];
	CDC_Transmit_FS(u8TxFrame, CFG_PAYLOAD_SIZE);
	return true;
}

static bool ProcessSetConfig(void){
	sRs485Config rs485;
	uint8_t cnt = 0u;
	rs485.baud_rate        = (eBaudRate)     ptrFrameDataStart[cnt++];
	rs485.stop_bits        = (eStopBits)     ptrFrameDataStart[cnt++];
	rs485.parity           = (eParity)       ptrFrameDataStart[cnt++];
	rs485.protocol         = (eRs485Protocol)ptrFrameDataStart[cnt++];
	rs485.log_dir          = (eLogDirection) ptrFrameDataStart[cnt++];
	rs485.params.bytes[0u] = ptrFrameDataStart[cnt++];
	rs485.params.bytes[1u] = ptrFrameDataStart[cnt++];
	rs485.params.bytes[2u] = ptrFrameDataStart[cnt++];
	rs485.params.bytes[3u] = ptrFrameDataStart[cnt++];
	rs485.params.bytes[4u] = ptrFrameDataStart[cnt++];
	vConfig_SetRs485(&rs485);
	memcpy(u8TxFrame, "ok", 2);
	CDC_Transmit_FS(u8TxFrame, 2);
	return true;
}

/**
 * @brief  Computes an 8-bit XOR checksum over a byte buffer.
 * @param  data  Pointer to the data buffer.
 * @param  len   Number of bytes to include.
 * @retval XOR of all bytes in [data, data+len).
 */
static uint8_t Checksum8_XOR(const uint8_t *data, size_t len){
    uint8_t checksum = 0;

    for (size_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }

    return checksum;
}

/**
 * @brief  Verifies the XOR checksum of a complete frame (including the trailing checksum byte).
 * @note   Sender appends XOR of all preceding bytes as the checksum byte, so XORing the entire
 *         frame (payload + checksum) always yields 0 for a valid, uncorrupted frame.
 * @param  frame  Pointer to the full frame buffer.
 * @param  len    Total frame length in bytes, including the checksum byte.
 * @retval true  - checksum valid.
 * @retval false - frame corrupted.
 */
static bool VerifyChecksum(const uint8_t *frame, size_t len){
    uint8_t xor = 0;

    for (size_t i = 0; i < len; i++)
    {
        xor ^= frame[i];
    }

    return (xor == 0);
}
