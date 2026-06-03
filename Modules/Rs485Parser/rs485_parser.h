/*
 * rs485_parser.h
 *
 *  Created on: Jun 3, 2026
 *      Author: ahmed mahdy
 *
 * Protocol detection and frame extraction layer for the RS485 bus.
 * Supports: Modbus RTU, Modbus ASCII, DNP3, IEC 60870-5-101, IEC 60870-5-103,
 *           BACnet MS/TP, DMX512, Raw/Custom.
 *
 * Typical call sites
 * ------------------
 *  Init:        vMainInitFunc()   → vRs485Parser_Init(psConfig_Get()->rs485)
 *  RX ISR:      HAL_UART_RxCpltCallback  (handled internally in rs485_parser.c)
 *  Timer (1ms): HAL_TIM_PeriodElapsedCallback → vRs485Parser_TimerTick()
 *  Main loop:   if (bRs485Parser_FrameReady()) { ... vRs485Parser_GetFrame() ... }
 */

#ifndef MODULES_RS485PARSER_RS485_PARSER_H_
#define MODULES_RS485PARSER_RS485_PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Maximum frame size across all supported protocols.
   BACnet MS/TP data payload can be up to 501 bytes:
   8-byte header + 501-byte data + 2-byte CRC = 511 bytes total.      */
#define RS485_MAX_FRAME_SIZE  512U

/* Direction of the frame on the bus as decoded by the active protocol. */
typedef enum {
    FRAME_DIR_MASTER  = 0, /* master (primary station) → slave           */
    FRAME_DIR_SLAVE,       /* slave (secondary station) → master         */
    FRAME_DIR_UNKNOWN,     /* direction could not be determined           */
} eFrameDir;

/* One complete decoded frame. */
typedef struct {
    uint8_t   data[RS485_MAX_FRAME_SIZE];
    uint16_t  len;
    eFrameDir dir;
} sRs485Frame;

/*
 * Re-initialise UART2 for the protocol/baud/parity in *cfg, arm DMA
 * receive into SerialBuffer[], and enable the UART IDLE interrupt.
 * Call once from vMainInitFunc().  Safe to call again at runtime if
 * the user changes the protocol.
 */
void vRs485Parser_Init(const sRs485Config *cfg);

/*
 * Feed one received byte into the active protocol state machine.
 * You rarely need to call this directly — bytes are fed automatically
 * from the DMA drain inside vRs485Parser_IdleDetected() and the DMA
 * complete callback.
 */
void vRs485Parser_RxByte(uint8_t byte);

/*
 * Called from USART2_IRQHandler when the UART IDLE flag fires.
 * Drains all bytes that DMA has written to SerialBuffer since the last
 * call, then (for Modbus RTU) immediately processes the accumulated
 * frame — replacing the software 3.5-character timer.
 */
void vRs485Parser_IdleDetected(void);

/*
 * Advance the 50 ms idle timer for the Raw/Custom protocol only.
 * Call from the 1 ms TIM2 period-elapsed callback.
 * (Modbus RTU frame detection is now handled by the hardware IDLE
 * interrupt, so it no longer needs this tick.)
 */
void vRs485Parser_TimerTick(void);

/* Returns true when a complete, validated frame is waiting. */
bool bRs485Parser_FrameReady(void);

/*
 * Copy the waiting frame into *out and clear the ready flag.
 * Call only when bRs485Parser_FrameReady() returns true.
 */
void vRs485Parser_GetFrame(sRs485Frame *out);

/*
 * Format *frame as a human-readable log string suitable for the SD card.
 * Writes at most maxLen bytes into out[].  No null terminator is added.
 * Returns the number of bytes written.
 *
 * Output format:  "PROTO,DIR,XX XX XX ..."
 * Example:        "MBRTU,M,01 03 00 00 00 0A C5 CD"
 *                 "DNP3,S,05 64 14 44 03 00 01 00 9A 74 C0 C3 01 3C ..."
 * DIR codes: M = master, S = slave, ? = unknown
 */
uint16_t u16Rs485Parser_FormatLog(const sRs485Frame *frame,
                                   uint8_t *out, uint16_t maxLen);

#endif /* MODULES_RS485PARSER_RS485_PARSER_H_ */
