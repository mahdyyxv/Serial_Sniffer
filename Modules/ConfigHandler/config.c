/*
 * config.c
 *
 *  Created on: May 30, 2026
 *      Author: ahmed mahdy
 */

#include "config.h"
#include "eeprom.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ── Module state ─────────────────────────────────────────────────── */

static sDeviceConfig s_cfg;

/* ── Factory defaults ─────────────────────────────────────────────── */

static const sDeviceConfig DEFAULT_CONFIG = {
    .rs485 = {
        .baud_rate = ENUM_BAUD_9600,
        .stop_bits = ENUM_ONE_STOP_BIT,
        .parity    = ENUM_NO_PARITY,
        .protocol  = PROTO_MODBUS_RTU,
        .log_dir   = LOG_DIR_ALL,
        .params    = { .modbus = { .slave_filter = 0u, ._pad = {0u,0u,0u,0u} } }
    }
};

/* ── Private helpers ──────────────────────────────────────────────── */

/* udtEEprom_Write passes 'size' as MemAddSize (a known bug in eeprom.c).
   Calling with size=1 makes MemAddSize=1 = I2C_MEMADD_SIZE_8BIT, which
   is correct. Use single-byte calls throughout to stay safe.           */

static void prvWriteByte(uint8_t addr, uint8_t val)
{
    udtEEprom_Write(EEPROM_ADDRESS, addr, &val, 1u);
    HAL_Delay(10u); /* EEPROM write cycle: typical 5 ms, allow 10 ms */
}

static uint8_t prvReadByte(uint8_t addr)
{
    uint8_t val = 0u;
    udtEEprom_Read(EEPROM_ADDRESS, addr, &val, 1u);
    return val;
}

static void prvSaveRs485(const sRs485Config *r)
{
    uint8_t base = CFG_EEPROM_ADDR_RS485;
    prvWriteByte((uint8_t)(base + 0u), (uint8_t)r->baud_rate);
    prvWriteByte((uint8_t)(base + 1u), (uint8_t)r->stop_bits);
    prvWriteByte((uint8_t)(base + 2u), (uint8_t)r->parity);
    prvWriteByte((uint8_t)(base + 3u), (uint8_t)r->protocol);
    prvWriteByte((uint8_t)(base + 4u), (uint8_t)r->log_dir);
    for (uint8_t i = 0u; i < 5u; i++) {
        prvWriteByte((uint8_t)(base + 5u + i), r->params.bytes[i]);
    }
}

static void prvLoadRs485(sRs485Config *r)
{
    uint8_t base = CFG_EEPROM_ADDR_RS485;
    r->baud_rate = (eBaudRate)     prvReadByte((uint8_t)(base + 0u));
    r->stop_bits = (eStopBits)     prvReadByte((uint8_t)(base + 1u));
    r->parity    = (eParity)       prvReadByte((uint8_t)(base + 2u));
    r->protocol  = (eRs485Protocol)prvReadByte((uint8_t)(base + 3u));
    r->log_dir   = (eLogDirection) prvReadByte((uint8_t)(base + 4u));
    for (uint8_t i = 0u; i < 5u; i++) {
        r->params.bytes[i] = prvReadByte((uint8_t)(base + 5u + i));
    }

    /* Clamp all fields to valid enum ranges to survive EEPROM corruption */
    if (r->baud_rate >= ENUM_BAUD_COUNT)   r->baud_rate = ENUM_BAUD_9600;
    if (r->stop_bits > ENUM_TWO_STOP_BITS) r->stop_bits = ENUM_ONE_STOP_BIT;
    if (r->parity    > ENUM_ODD_PARITY)    r->parity    = ENUM_NO_PARITY;
    if (r->protocol  >= PROTO_COUNT)        r->protocol  = PROTO_MODBUS_RTU;
    if (r->log_dir   > LOG_DIR_SLAVE)       r->log_dir   = LOG_DIR_ALL;
}

/* ── Public API ───────────────────────────────────────────────────── */

void vConfig_Init(void)
{
    uint8_t magic0 = prvReadByte(CFG_EEPROM_ADDR_MAGIC0);
    uint8_t magic1 = prvReadByte(CFG_EEPROM_ADDR_MAGIC1);

    if ((magic0 == CFG_EEPROM_MAGIC_0) && (magic1 == CFG_EEPROM_MAGIC_1)) {
        prvLoadRs485(&s_cfg.rs485);
    } else {
        /* First boot or blank EEPROM: write magic + version + defaults */
        s_cfg = DEFAULT_CONFIG;
        prvWriteByte(CFG_EEPROM_ADDR_MAGIC0,  CFG_EEPROM_MAGIC_0);
        prvWriteByte(CFG_EEPROM_ADDR_MAGIC1,  CFG_EEPROM_MAGIC_1);
        prvWriteByte(CFG_EEPROM_ADDR_VERSION, CFG_EEPROM_VERSION);
        prvSaveRs485(&s_cfg.rs485);
    }
}

void vConfig_Save(const sDeviceConfig *cfg)
{
    s_cfg = *cfg;
    prvSaveRs485(&s_cfg.rs485);
}

const sDeviceConfig *psConfig_Get(void)
{
    return &s_cfg;
}

void vConfig_SetRs485(const sRs485Config *rs485)
{
    s_cfg.rs485 = *rs485;
    prvSaveRs485(&s_cfg.rs485);
}
