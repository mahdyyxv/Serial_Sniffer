/*
 * sdc.c
 *
 *  Created on: Jun 1, 2026
 *      Author: ahmed mahdy
 */
#include "sdc.h"
#include "fatfs_platform.h"
#include <string.h>   /* memcpy */

/* ── tunables ───────────────────────────────────────────────────────────── */
#define DETECT_DEBOUNCE_TICKS  10U    /* timer ticks before a card-detect edge is trusted */
#define INIT_RETRY_TICKS       2000U  /* timer ticks between mount retries (~2.4 s)        */

/* ── state machine ──────────────────────────────────────────────────────── */
typedef enum {
    SDC_INIT = 0,
    SDC_READY,
    SDC_WRITING,
    SDC_READING,
    SDC_CARD_REMOVED,
    SDC_ERROR_INIT,
    SDC_ERROR_WRITING,
    SDC_ERROR_READING,
} eSDCState;

/* ── write queue ────────────────────────────────────────────────────────── */
typedef struct {
    uint8_t date[SDC_DATE_MAX_SIZE];
    uint8_t dateLen;
    uint8_t data[SDC_DATA_MAX_SIZE];
    uint8_t dataLen;
} sSDCRecord;

static sSDCRecord s_queue[SDC_WRITE_QUEUE_SIZE];
static uint8_t    s_qHead      = 0U;   /* index of the oldest record (next to write) */
static uint8_t    s_qTail      = 0U;   /* index of the next free slot                */
static uint8_t    s_qCount     = 0U;   /* number of records currently in the queue   */
static bool       s_writeError = false;

/* ── read descriptor ────────────────────────────────────────────────────── */
typedef struct {
    uint8_t        filename[32];          /* null-terminated; passed to FATFS as char* */
    uint32_t       offset;
    bool           pending;
    uint8_t        chunk[SDC_READ_CHUNK_SIZE];
    uint16_t       size;
    eSDCReadStatus status;
} sReadReq;

static sReadReq s_read = {0};

/* ── module state ───────────────────────────────────────────────────────── */
static eSDCState s_state = SDC_INIT;

/* card-detect — set inside timer tick, consumed inside engine */
static volatile bool     s_cardInserted = false;
static volatile bool     s_cardRemoved  = false;
static uint8_t           s_debounce     = 0U;
static bool              s_lastDetected = false;

/* retry timing (counts timer ticks) */
static volatile uint32_t s_timerTicks   = 0U;
static uint32_t          s_retryAt      = 0U;

/* ── private helpers ────────────────────────────────────────────────────── */

static FRESULT prvMount(void)
{
    return f_mount(&SDFatFS, SDPath, 1);
}

static void prvUnmount(void)
{
    f_mount(NULL, SDPath, 0);
}

static FRESULT prvOpenAppend(FIL *fp, const char *path)
{
    FRESULT fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK) {
            f_close(fp);
        }
    }
    return fr;
}

static void prvClearQueue(void)
{
    s_qHead  = 0U;
    s_qTail  = 0U;
    s_qCount = 0U;
}

/* Write the oldest queued record to the log file then advance the head.
   Uses f_write with explicit lengths — no format strings, no char* casts. */
static void prvDoWrite(void)
{
    static const uint8_t SEP = (uint8_t)',';
    static const uint8_t NL  = (uint8_t)'\n';
    FIL  fil;
    UINT bw;
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    if (prvOpenAppend(&fil, SDC_LOG_FILENAME) != FR_OK) {
        s_writeError = true;
        s_state      = SDC_ERROR_WRITING;
        return;
    }

    f_write(&fil, s_queue[s_qHead].date, s_queue[s_qHead].dateLen, &bw);
    f_write(&fil, &SEP, 1U, &bw);
    f_write(&fil, s_queue[s_qHead].data, s_queue[s_qHead].dataLen, &bw);
    f_write(&fil, &NL,  1U, &bw);
    f_close(&fil);

    s_qHead  = (uint8_t)((s_qHead + 1U) % SDC_WRITE_QUEUE_SIZE);
    s_qCount--;

    s_writeError = false;
    s_state      = SDC_READY;
    HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
}

static void prvDoRead(void)
{
    FIL    fil;
    UINT   bytesRead = 0U;
    FRESULT fr;

    /* FATFS expects a null-terminated char* path; filename was null-terminated on copy */
    fr = f_open(&fil, (const char *)s_read.filename, FA_READ);
    if (fr != FR_OK) {
        s_read.status = SDC_READ_FAIL;
        s_state       = SDC_ERROR_READING;
        return;
    }

    fr = f_lseek(&fil, s_read.offset);
    if (fr != FR_OK) {
        f_close(&fil);
        s_read.status = SDC_READ_FAIL;
        s_state       = SDC_ERROR_READING;
        return;
    }

    fr = f_read(&fil, s_read.chunk, SDC_READ_CHUNK_SIZE, &bytesRead);
    f_close(&fil);

    if (fr != FR_OK) {
        s_read.status = SDC_READ_FAIL;
        s_state       = SDC_ERROR_READING;
        return;
    }

    s_read.size    = (uint16_t)bytesRead;
    s_read.pending = false;
    /* fewer bytes than requested means we hit the end of the file */
    s_read.status  = (bytesRead < SDC_READ_CHUNK_SIZE) ? SDC_READ_EOF : SDC_READ_OK;
    s_state        = SDC_READY;
}

/* ── public API ─────────────────────────────────────────────────────────── */

void vSDC_TimerTick(void)
{
    s_timerTicks++;

    bool detected = (BSP_PlatformIsDetected() == SD_PRESENT);

    if (detected != s_lastDetected) {
        s_debounce++;
        if (s_debounce >= DETECT_DEBOUNCE_TICKS) {
            s_lastDetected = detected;
            s_debounce     = 0U;
            if (detected) {
                s_cardInserted = true;
            } else {
                s_cardRemoved  = true;
            }
        }
    } else {
        s_debounce = 0U;
    }
}

bool bSDC_IsReady(void)
{
    return (s_state == SDC_READY);
}

bool bSDC_Write(const uint8_t *date, uint8_t dateLen,
                const uint8_t *buf,  uint8_t bufLen)
{
    if (s_qCount >= SDC_WRITE_QUEUE_SIZE) {
        return false;   /* queue full — caller must retry later */
    }

    /* clamp lengths to buffer maximums */
    if (dateLen > (uint8_t)(SDC_DATE_MAX_SIZE - 1U)) {
        dateLen = (uint8_t)(SDC_DATE_MAX_SIZE - 1U);
    }
    if (bufLen > (uint8_t)(SDC_DATA_MAX_SIZE - 1U)) {
        bufLen = (uint8_t)(SDC_DATA_MAX_SIZE - 1U);
    }

    memcpy(s_queue[s_qTail].date, date, dateLen);
    s_queue[s_qTail].dateLen = dateLen;

    memcpy(s_queue[s_qTail].data, buf, bufLen);
    s_queue[s_qTail].dataLen = bufLen;

    s_qTail  = (uint8_t)((s_qTail + 1U) % SDC_WRITE_QUEUE_SIZE);
    s_qCount++;

    return true;
}

uint8_t u8SDC_GetQueueCount(void)
{
    return s_qCount;
}

bool bSDC_HasWriteError(void)
{
    return s_writeError;
}

bool bSDC_Read(const uint8_t *filename, uint8_t filenameLen, uint32_t offset)
{
    if (s_state != SDC_READY || s_read.pending) {
        return false;
    }

    /* clamp and copy; keep a null terminator for the FATFS f_open call */
    if (filenameLen > (uint8_t)(sizeof(s_read.filename) - 1U)) {
        filenameLen = (uint8_t)(sizeof(s_read.filename) - 1U);
    }
    memcpy(s_read.filename, filename, filenameLen);
    s_read.filename[filenameLen] = 0U;

    s_read.offset  = offset;
    s_read.pending = true;
    s_read.status  = SDC_READ_PENDING;
    s_state        = SDC_READING;
    return true;
}

eSDCReadStatus eSDC_GetReadStatus(void)
{
    return s_read.status;
}

uint16_t u16SDC_GetReadData(uint8_t *out)
{
    if (out == NULL || s_read.size == 0U) {
        return 0U;
    }
    memcpy(out, s_read.chunk, s_read.size);
    return s_read.size;
}

void vSDC_Engine(void)
{
    /* card removal takes priority over every other state */
    if (s_cardRemoved && s_state != SDC_CARD_REMOVED) {
        s_cardRemoved  = false;
        s_cardInserted = false;
        prvClearQueue();
        s_read.pending = false;
        prvUnmount();
        s_state = SDC_CARD_REMOVED;
        return;
    }

    switch (s_state)
    {
        case SDC_INIT:
            if (prvMount() == FR_OK) {
                s_state = SDC_READY;
                HAL_Delay(100);
            } else {
                s_retryAt = s_timerTicks + INIT_RETRY_TICKS;
                s_state   = SDC_ERROR_INIT;
            }
            break;

        case SDC_READY:
            if (s_qCount > 0U) {
                s_state = SDC_WRITING;
            }
            break;

        case SDC_WRITING:
            prvDoWrite();
            break;

        case SDC_READING:
            prvDoRead();
            break;

        case SDC_CARD_REMOVED:
            if (s_cardInserted) {
                s_cardInserted = false;
                s_state        = SDC_INIT;
            }
            break;

        case SDC_ERROR_INIT:
            if (s_timerTicks >= s_retryAt) {
                s_state = SDC_INIT;
            }
            break;

        case SDC_ERROR_WRITING:
            /* one engine tick here lets the caller observe bSDC_HasWriteError(),
               then the failed record is discarded and the queue continues */
            s_qHead  = (uint8_t)((s_qHead + 1U) % SDC_WRITE_QUEUE_SIZE);
            s_qCount--;
            s_state  = SDC_READY;
            break;

        case SDC_ERROR_READING:
            s_read.pending = false;
            s_state        = SDC_READY;
            break;

        default:
            s_state = SDC_INIT;
            break;
    }
}
