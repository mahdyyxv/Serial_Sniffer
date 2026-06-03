/*
 * sdc.h
 *
 *  Created on: Jun 1, 2026
 *      Author: ahmed mahdy
 */
#ifndef MODULES_SDCARD_SDC_H_
#define MODULES_SDCARD_SDC_H_

#include <stdint.h>
#include <stdbool.h>
#include "fatfs.h"

/* ── write queue dimensions ─────────────────────────────────────────────── */
#define SDC_WRITE_QUEUE_SIZE   10U

/* Date string buffer — matches snprintf(Buf, 50, ...) in rtc.c */
#define SDC_DATE_MAX_SIZE      50U

/* Data payload buffer.  uint16_t dataLen allows values 0–256.
   256 bytes covers a fully-formatted Modbus/DNP3/IEC101 log line.     */
#define SDC_DATA_MAX_SIZE      256U

/* ── read chunk size ────────────────────────────────────────────────────── */
#define SDC_READ_CHUNK_SIZE    1024U

/* ── log file written by bSDC_Write ────────────────────────────────────── */
#define SDC_LOG_FILENAME       "log.txt"

typedef enum {
    SDC_READ_IDLE = 0,
    SDC_READ_PENDING,
    SDC_READ_OK,
    SDC_READ_EOF,
    SDC_READ_FAIL,
} eSDCReadStatus;

/* Called from timer interrupt every tick — card-detect debounce & timekeeping */
void           vSDC_TimerTick(void);

/* Called from main loop — drives the state machine (non-blocking) */
void           vSDC_Engine(void);

/* True only when the card is mounted and the state machine is idle */
bool           bSDC_IsReady(void);

/* ── write API ───────────────────────────────────────────────────────────
   Enqueue one log record.  dateLen and bufLen are both uint16_t so a
   256-byte payload (SDC_DATA_MAX_SIZE) can be passed without truncation.
   Lengths are clamped to SDC_DATE_MAX_SIZE / SDC_DATA_MAX_SIZE.
   Returns false when the queue is full — caller must retry later.       */
bool           bSDC_Write(const uint8_t *date, uint8_t  dateLen,
                          const uint8_t *buf,  uint16_t bufLen);

/* Number of records currently waiting in the write queue */
uint8_t        u8SDC_GetQueueCount(void);

/* True if the last file-write operation failed */
bool           bSDC_HasWriteError(void);

/* ── read API ────────────────────────────────────────────────────────────
   Request up to SDC_READ_CHUNK_SIZE bytes from 'filename' at byte 'offset'.
   Returns false when the module is busy or not ready.                   */
bool           bSDC_Read(const uint8_t *filename, uint8_t filenameLen,
                         uint32_t offset);
eSDCReadStatus eSDC_GetReadStatus(void);

/* Copy the last-read chunk into out[].  Returns the number of bytes copied. */
uint16_t       u16SDC_GetReadData(uint8_t *out);

#endif /* MODULES_SDCARD_SDC_H_ */
