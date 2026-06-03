/*
 * config.h
 *
 *  Created on: May 30, 2026
 *      Author: ahmed mahdy
 */

#ifndef MODULES_CONFIGHANDLER_CONFIG_H_
#define MODULES_CONFIGHANDLER_CONFIG_H_

#include <stdint.h>
#include "DataTypes.h"

/* ── Physical layer enums ────────────────────────────────────────────── */

typedef enum {
    ENUM_BAUD_4800   = 0,
    ENUM_BAUD_9600,
    ENUM_BAUD_19200,
    ENUM_BAUD_38400,
    ENUM_BAUD_57600,
    ENUM_BAUD_115200,
    ENUM_BAUD_COUNT
} eBaudRate;

typedef enum {
    ENUM_ONE_STOP_BIT = 0,
    ENUM_TWO_STOP_BITS
} eStopBits;

typedef enum {
    ENUM_NO_PARITY = 0,
    ENUM_EVEN_PARITY,
    ENUM_ODD_PARITY
} eParity;

/* ── RS485 protocol selection ────────────────────────────────────────── */

typedef enum {
    PROTO_MODBUS_RTU   = 0, /* Timeout-delimited, CRC-16                  */
    PROTO_MODBUS_ASCII,     /* Colon-start, CRLF-end, LRC                 */
    PROTO_DNP3,             /* 0x05 0x64 sync, block-CRC, DIR bit         */
    PROTO_IEC101,           /* FT1.2 framing, IEC 60870-5-101             */
    PROTO_IEC103,           /* FT1.2 framing, IEC 60870-5-103             */
    PROTO_BACNET_MSTP,      /* 0x55 0xFF preamble, CRC-8 header           */
    PROTO_DMX512,           /* Break + 513 slots at 250 kbaud             */
    PROTO_RAW,              /* User-configurable delimiters / fixed length */
    PROTO_COUNT
} eRs485Protocol;

/* Which frame directions to write to the SD card log */
typedef enum {
    LOG_DIR_ALL    = 0, /* master frames and slave frames             */
    LOG_DIR_MASTER,     /* only frames identified as master→slave     */
    LOG_DIR_SLAVE,      /* only frames identified as slave→master     */
} eLogDirection;

/* ── Protocol-specific parameter blocks (5 bytes each) ──────────────── */
/* Kept to 5 bytes so the full RS485 config fits in one 8-byte EEPROM
   page write after the fixed fields (baud, stop, parity, proto, dir). */

typedef struct {
    uint8_t slave_filter; /* 0 = log all addresses; 1–247 = only this slave */
    uint8_t _pad[4];
} sModbusParams;

typedef struct {
    uint8_t master_addr_lo;   /* link-layer address of the master (lo byte) */
    uint8_t master_addr_hi;
    uint8_t station_addr_lo;  /* outstation to match; 0xFFFF = all          */
    uint8_t station_addr_hi;
    uint8_t _pad;
} sDnp3Params;

typedef struct {
    uint8_t link_addr_size; /* 1 or 2 bytes for the A field               */
    uint8_t link_addr;      /* secondary link address to match; 0 = all   */
    uint8_t _pad[3];
} sIec101Params;             /* also used for IEC 103 (identical framing)  */

typedef struct {
    uint8_t max_master; /* highest MAC address treated as a master (0–127)*/
    uint8_t _pad[4];
} sBacnetParams;

typedef struct {
    uint8_t use_start_byte; /* 0 = no start delimiter, 1 = use start_byte */
    uint8_t start_byte;
    uint8_t use_end_byte;   /* 0 = no end delimiter,   1 = use end_byte   */
    uint8_t end_byte;
    uint8_t fixed_len;      /* 0 = use 50 ms timeout, else fixed N bytes  */
} sRawParams;

/* DMX512 has no user-configurable parameters; always 250 kbaud, 2 stop  */

typedef union {
    sModbusParams  modbus;
    sDnp3Params    dnp3;
    sIec101Params  iec101;  /* covers IEC 103 too */
    sBacnetParams  bacnet;
    sRawParams     raw;
    uint8_t        bytes[5]; /* raw byte access for EEPROM serialisation   */
} uProtoParams;

/* ── Per-bus configuration structs ──────────────────────────────────── */

typedef struct {
    eBaudRate      baud_rate; /* ignored for DMX512 (always 250 kbaud)     */
    eStopBits      stop_bits; /* ignored for DMX512 (always 2 stop bits)   */
    eParity        parity;    /* ignored for DMX512 (always none)           */
    eRs485Protocol protocol;
    eLogDirection  log_dir;
    uProtoParams   params;
} sRs485Config;

/* RS232 / UART1 channel (no direction control pin, no protocol decode)  */
typedef struct {
    eBaudRate baud_rate;
    eStopBits stop_bits;
    eParity   parity;
    uint8_t   _reserved[2];
} sRs232Config;

/* ── Top-level device configuration ─────────────────────────────────── */

typedef struct {
    sRs485Config rs485;
    /* future: sCan1Config can1; sCan2Config can2; sRs232Config uart1; */
} sDeviceConfig;

/* ── EEPROM layout ───────────────────────────────────────────────────── */
/* Password module occupies bytes 0xFB–0xFF (PASSWORD_START_BYTE = 251). */
/* Config uses bytes 0x00–0x0C (13 bytes, safe zone).                    */

#define CFG_EEPROM_MAGIC_0      0xADu
#define CFG_EEPROM_MAGIC_1      0xDEu
#define CFG_EEPROM_VERSION      0x01u

#define CFG_EEPROM_ADDR_MAGIC0  0x00u
#define CFG_EEPROM_ADDR_MAGIC1  0x01u
#define CFG_EEPROM_ADDR_VERSION 0x02u
/* RS485 block: baud(1) stop(1) parity(1) protocol(1) log_dir(1) params(5) = 10 bytes */
#define CFG_EEPROM_ADDR_RS485   0x03u

/* ── Public API ──────────────────────────────────────────────────────── */

/* Load config from EEPROM; fall back to defaults on first boot. */
void                  vConfig_Init(void);

/* Persist the full config to EEPROM. */
void                  vConfig_Save(const sDeviceConfig *cfg);

/* Return a pointer to the current (RAM) config. */
const sDeviceConfig  *psConfig_Get(void);

/* Update only the RS485 section and persist it. */
void                  vConfig_SetRs485(const sRs485Config *rs485);

#endif /* MODULES_CONFIGHANDLER_CONFIG_H_ */
