/*
 * rs485_parser.c
 *
 *  Created on: Jun 3, 2026
 *      Author: ahmed mahdy
 *
 * RS485 multi-protocol frame detector.
 *
 * ── Architecture ────────────────────────────────────────────────────
 *  Each supported protocol has its own static state struct and a pair
 *  of functions: handle_PROTO(byte) and (where needed) timeout_PROTO().
 *
 *  DMA (DMA1_Stream5 Ch4) fills SerialBuffer[] continuously.
 *  UART IDLE interrupt fires when the bus goes quiet between frames.
 *    → vRs485Parser_IdleDetected() drains new DMA bytes into the parser
 *      and, for Modbus RTU, immediately processes the frame — hardware
 *      IDLE replaces the software 3.5-character-gap timer.
 *  HAL_UART_RxCpltCallback fires when SerialBuffer[] fills up (rare);
 *    it drains remaining bytes and restarts DMA.
 *  HAL_UART_ErrorCallback detects the DMX512 BREAK framing error.
 *  vRs485Parser_TimerTick (1 ms) is kept only for the Raw idle timeout.
 *
 * ── Direction detection per protocol ────────────────────────────────
 *  Modbus RTU/ASCII  conversation tracking (request ↔ response pairing)
 *  DNP3              Control byte bit 7 (DIR): 1 = master→outstation
 *  IEC 101/103       Control byte bit 6 (PRM): 1 = primary (master)
 *  BACnet MS/TP      Source MAC ≤ max_master → master, else slave
 *  DMX512            Always MASTER (unidirectional controller→fixtures)
 *  Raw               Always UNKNOWN
 */

#include "rs485_parser.h"
#include "main.h"
#include <string.h>

/* ── External peripheral handles ───────────────────────────────────── */
extern UART_HandleTypeDef  huart2;
extern DMA_HandleTypeDef   hdma_usart2_rx;

/* DMA receive buffer — defined in mainHelper.c, shared with MSP.
   Sized to RS485_MAX_FRAME_SIZE so the largest possible frame (511-byte
   BACnet MS/TP) always fits inside one circular wrap.                  */
#define DMA_BUF_SIZE  512U
extern uint8_t SerialBuffer[DMA_BUF_SIZE];

/* ══════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ══════════════════════════════════════════════════════════════════════ */

/* DMA read-head: index of the next byte to consume from SerialBuffer[] */
static uint16_t s_dma_head;

/* HAL baud rate values, indexed by eBaudRate */
static const uint32_t BAUD_TABLE[ENUM_BAUD_COUNT] = {
    4800u, 9600u, 19200u, 38400u, 57600u, 115200u
};

/* HAL parity values, indexed by eParity */
static const uint32_t PARITY_HAL[] = {
    UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD
};

/* HAL stop-bit values, indexed by eStopBits */
static const uint32_t STOP_HAL[] = {
    UART_STOPBITS_1, UART_STOPBITS_2
};

/* Short protocol name strings used in the log output */
static const char * const PROTO_TAG[PROTO_COUNT] = {
    "MBRTU", "MBASCII", "DNP3", "IEC101", "IEC103", "BACNET", "DMX512", "RAW"
};

/* ══════════════════════════════════════════════════════════════════════
 * SHARED RX BUFFER & OUTPUT FRAME
 * ══════════════════════════════════════════════════════════════════════ */

static uint8_t    s_rx_buf[RS485_MAX_FRAME_SIZE]; /* accumulation buffer */
static uint16_t   s_rx_len;                        /* bytes currently buffered */

static sRs485Frame s_frame;
static bool        s_frame_ready;

/* Active configuration */
static eRs485Protocol s_protocol;
static eLogDirection  s_log_dir;

/* ══════════════════════════════════════════════════════════════════════
 * CRC / CHECKSUM HELPERS
 * ══════════════════════════════════════════════════════════════════════ */

/* CRC-16/IBM (Modbus): poly 0x8005 reflected = 0xA001, init 0xFFFF     */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)*data++;
        for (uint8_t i = 0u; i < 8u; i++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
        }
    }
    return crc;
}

/* CRC-16/DNP: poly 0x3D65 reflected = 0xA6BC, init 0x0000, XOR 0xFFFF */
static uint16_t crc16_dnp3(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000u;
    while (len--) {
        crc ^= (uint16_t)*data++;
        for (uint8_t i = 0u; i < 8u; i++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xA6BCu) : (crc >> 1);
        }
    }
    return (uint16_t)(~crc);
}

/* CRC-8 for BACnet MS/TP header: poly 0x07, init 0xFF, MSB-first.
   Stored byte = ~CRC, so valid when: crc8_bacnet(data,5) ^ 0xFF == stored */
static uint8_t crc8_bacnet_hdr(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    while (len--) {
        uint8_t b = *data++;
        for (uint8_t i = 0u; i < 8u; i++) {
            crc = ((crc ^ b) & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                                       : (uint8_t)(crc << 1);
            b = (uint8_t)(b << 1);
        }
    }
    return crc;
}

/* Convert a single ASCII hex nibble to its numeric value */
static uint8_t hex_nibble(uint8_t c)
{
    if (c >= (uint8_t)'0' && c <= (uint8_t)'9') { return (uint8_t)(c - '0'); }
    if (c >= (uint8_t)'A' && c <= (uint8_t)'F') { return (uint8_t)(c - 'A' + 10u); }
    if (c >= (uint8_t)'a' && c <= (uint8_t)'f') { return (uint8_t)(c - 'a' + 10u); }
    return 0u;
}

static uint8_t ascii_pair_to_byte(uint8_t hi, uint8_t lo)
{
    return (uint8_t)((hex_nibble(hi) << 4) | hex_nibble(lo));
}

/* ══════════════════════════════════════════════════════════════════════
 * FRAME EMISSION HELPERS
 * ══════════════════════════════════════════════════════════════════════ */

static bool should_log(eFrameDir dir)
{
    switch (s_log_dir) {
        case LOG_DIR_MASTER: return (dir == FRAME_DIR_MASTER);
        case LOG_DIR_SLAVE:  return (dir == FRAME_DIR_SLAVE);
        default:             return true;
    }
}

static void emit_frame(eFrameDir dir)
{
    if (!should_log(dir)) { return; }
    uint16_t n = (s_rx_len <= RS485_MAX_FRAME_SIZE) ? s_rx_len : RS485_MAX_FRAME_SIZE;
    memcpy(s_frame.data, s_rx_buf, n);
    s_frame.len   = n;
    s_frame.dir   = dir;
    s_frame_ready = true;
}

static void reset_rx(void)
{
    s_rx_len = 0u;
}

/* Append one byte; return false (and reset) on overflow */
static bool rx_append(uint8_t byte)
{
    if (s_rx_len >= RS485_MAX_FRAME_SIZE) { reset_rx(); return false; }
    s_rx_buf[s_rx_len++] = byte;
    return true;
}

/* ══════════════════════════════════════════════════════════════════════
 * PROTOCOL: MODBUS RTU
 * ══════════════════════════════════════════════════════════════════════
 * Frame boundary: 3.5-character silence on the line.
 * Direction:      conversation tracking — master sends to slave address,
 *                 next frame from same address (before a new request) is
 *                 the slave response.
 * ══════════════════════════════════════════════════════════════════════ */

static struct {
    uint8_t  last_addr;
    uint8_t  last_fc;
    bool     waiting_response;
} s_mbrtu;

static void mbrtu_process(void)
{
    if (s_rx_len < 4u) { reset_rx(); return; }

    /* CRC-16 stored little-endian at the end of the frame */
    uint16_t crc_calc = crc16_modbus(s_rx_buf, s_rx_len - 2u);
    uint16_t crc_recv = (uint16_t)((uint16_t)s_rx_buf[s_rx_len - 1u] << 8)
                      | s_rx_buf[s_rx_len - 2u];
    if (crc_calc != crc_recv) { reset_rx(); return; }

    uint8_t   addr = s_rx_buf[0];
    uint8_t   fc   = s_rx_buf[1];
    eFrameDir dir  = FRAME_DIR_UNKNOWN;

    if (addr == 0x00u) {
        /* Broadcast: always from master, no response expected */
        dir = FRAME_DIR_MASTER;
        s_mbrtu.waiting_response = false;
    } else if (s_mbrtu.waiting_response && (addr == s_mbrtu.last_addr)) {
        dir = FRAME_DIR_SLAVE;
        s_mbrtu.waiting_response = false;
    } else {
        dir = FRAME_DIR_MASTER;
        s_mbrtu.last_addr        = addr;
        s_mbrtu.last_fc          = fc & 0x7Fu;
        s_mbrtu.waiting_response = true;
    }

    emit_frame(dir);
    reset_rx();
}

static void handle_mbrtu(uint8_t byte)
{
    rx_append(byte);
    /* Frame boundary is detected by hardware IDLE interrupt, not a timer.
       mbrtu_process() is called from vRs485Parser_IdleDetected().        */
}

/* ══════════════════════════════════════════════════════════════════════
 * PROTOCOL: MODBUS ASCII
 * ══════════════════════════════════════════════════════════════════════
 * Frame boundary: ':' start, "\r\n" end.
 * Direction:      same conversation-tracking as RTU, based on binary FC.
 * ══════════════════════════════════════════════════════════════════════ */

static struct {
    bool    in_frame;
    uint8_t last_addr;
    bool    waiting_response;
} s_mbascii;

static void mbascii_process(void)
{
    /* Buffer: ':' [A1 A2] [F1 F2] [...data hex...] [L1 L2] '\r' '\n'
       Minimum total length: 1 + 2 + 2 + 2 + 2 = 9 bytes               */
    if ((s_rx_len < 9u) || (s_rx_buf[0] != (uint8_t)':')) {
        reset_rx(); return;
    }

    /* Number of binary bytes: all ASCII hex pairs between ':' and "\r\n" */
    uint8_t n_pairs = (uint8_t)((s_rx_len - 3u) / 2u); /* remove :, \r, \n */
    if (n_pairs < 3u) { reset_rx(); return; }            /* need addr+fc+lrc  */

    /* LRC validation: 8-bit sum of all binary bytes (addr…lrc) == 0x00  */
    uint8_t sum = 0u;
    for (uint8_t i = 0u; i < n_pairs; i++) {
        sum = (uint8_t)(sum + ascii_pair_to_byte(s_rx_buf[1u + i * 2u],
                                                  s_rx_buf[2u + i * 2u]));
    }
    if (sum != 0u) { reset_rx(); return; }

    uint8_t addr = ascii_pair_to_byte(s_rx_buf[1], s_rx_buf[2]);
    eFrameDir dir = FRAME_DIR_UNKNOWN;

    if (addr == 0x00u) {
        dir = FRAME_DIR_MASTER;
        s_mbascii.waiting_response = false;
    } else if (s_mbascii.waiting_response && (addr == s_mbascii.last_addr)) {
        dir = FRAME_DIR_SLAVE;
        s_mbascii.waiting_response = false;
    } else {
        dir = FRAME_DIR_MASTER;
        s_mbascii.last_addr        = addr;
        s_mbascii.waiting_response = true;
    }

    emit_frame(dir);
    reset_rx();
}

static void handle_mbascii(uint8_t byte)
{
    if (byte == (uint8_t)':') {
        reset_rx();
        s_mbascii.in_frame = true;
        rx_append(byte);
        return;
    }
    if (!s_mbascii.in_frame) { return; }
    rx_append(byte);
    if (byte == (uint8_t)'\n') {
        s_mbascii.in_frame = false;
        mbascii_process();
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * PROTOCOL: DNP3
 * ══════════════════════════════════════════════════════════════════════
 * Frame structure:
 *   [0x05][0x64][LEN][CTRL][DST_L][DST_H][SRC_L][SRC_H][CRC_L][CRC_H]
 *   followed by ceil((LEN-5)/16) data blocks of (16 data + 2 CRC) bytes,
 *   with the last block being ((LEN-5)%16) data + 2 CRC bytes.
 *
 * Direction: CTRL byte bit 7 (DIR): 1 = master→outstation.
 * ══════════════════════════════════════════════════════════════════════ */

typedef enum {
    DNP3_IDLE = 0,
    DNP3_SYNC1,
    DNP3_IN_HEADER,   /* collecting 8 bytes after 0x05 0x64 */
    DNP3_IN_BLOCKS,
} eDnp3State;

static struct {
    eDnp3State state;
    uint8_t    header_pos;    /* bytes received in current header phase   */
    uint8_t    frame_len;     /* DNP3 Length field value                  */
    uint8_t    ctrl;          /* Control byte (for direction)             */
    uint16_t   n_blocks;      /* number of complete 16-byte data blocks   */
    uint8_t    last_blk_sz;   /* bytes in the final partial block (0=none)*/
    uint16_t   blks_done;     /* fully received blocks so far             */
    uint8_t    blk_data_pos;  /* data bytes received in current block     */
    uint8_t    blk_crc_pos;   /* CRC bytes received for current block     */
} s_dnp3;

static void handle_dnp3(uint8_t byte)
{
    if (!rx_append(byte)) { s_dnp3.state = DNP3_IDLE; return; }

    switch (s_dnp3.state) {
        /* ── sync ─────────────────────────────────────────────────── */
        case DNP3_IDLE:
            if (byte == 0x05u) {
                s_rx_len = 1u; s_rx_buf[0] = byte;
                s_dnp3.state = DNP3_SYNC1;
            } else {
                s_rx_len = 0u;
            }
            break;

        case DNP3_SYNC1:
            if (byte == 0x64u) {
                s_dnp3.header_pos = 0u;
                s_dnp3.state      = DNP3_IN_HEADER;
            } else if (byte == 0x05u) {
                /* Possible new sync — keep only this byte */
                s_rx_len = 1u; s_rx_buf[0] = byte;
            } else {
                reset_rx(); s_dnp3.state = DNP3_IDLE;
            }
            break;

        /* ── 8-byte header (LEN CTRL DST DST SRC SRC CRC CRC) ─────── */
        case DNP3_IN_HEADER:
            s_dnp3.header_pos++;
            if (s_dnp3.header_pos == 1u) { s_dnp3.frame_len = byte; } /* LEN  */
            if (s_dnp3.header_pos == 2u) { s_dnp3.ctrl      = byte; } /* CTRL */

            if (s_dnp3.header_pos == 8u) {
                /* s_rx_buf: [0x05][0x64][LEN][CTRL][DST_L][DST_H][SRC_L][SRC_H][CRC_L][CRC_H]
                   CRC covers bytes [2..7] (LEN through SRC_H).                                  */
                uint16_t hcrc_calc = crc16_dnp3(&s_rx_buf[2], 6u);
                uint16_t hcrc_recv = (uint16_t)((uint16_t)s_rx_buf[9] << 8) | s_rx_buf[8];
                if (hcrc_calc != hcrc_recv) {
                    reset_rx(); s_dnp3.state = DNP3_IDLE; break;
                }

                uint8_t user_data = (s_dnp3.frame_len >= 5u)
                                  ? (uint8_t)(s_dnp3.frame_len - 5u) : 0u;
                s_dnp3.n_blocks    = user_data / 16u;
                s_dnp3.last_blk_sz = user_data % 16u;
                s_dnp3.blks_done   = 0u;
                s_dnp3.blk_data_pos = 0u;
                s_dnp3.blk_crc_pos  = 0u;

                if (user_data == 0u) {
                    eFrameDir dir = (s_dnp3.ctrl & 0x80u) ? FRAME_DIR_MASTER : FRAME_DIR_SLAVE;
                    emit_frame(dir);
                    reset_rx(); s_dnp3.state = DNP3_IDLE;
                } else {
                    s_dnp3.state = DNP3_IN_BLOCKS;
                }
            }
            break;

        /* ── data blocks (16 data + 2 CRC each; last block may be < 16) */
        case DNP3_IN_BLOCKS: {
            bool is_last_block = (s_dnp3.blks_done == s_dnp3.n_blocks);
            uint8_t blk_data_sz = is_last_block ? s_dnp3.last_blk_sz : 16u;

            if (s_dnp3.blk_crc_pos == 0u) {
                /* Still in data portion of this block */
                s_dnp3.blk_data_pos++;
                if (s_dnp3.blk_data_pos >= blk_data_sz) {
                    s_dnp3.blk_crc_pos  = 1u; /* first CRC byte next */
                    s_dnp3.blk_data_pos = 0u;
                }
            } else {
                s_dnp3.blk_crc_pos++;
                if (s_dnp3.blk_crc_pos > 2u) {
                    /* Block (data + 2 CRC bytes) fully received */
                    s_dnp3.blks_done++;
                    s_dnp3.blk_crc_pos  = 0u;
                    s_dnp3.blk_data_pos = 0u;

                    bool all_done = (s_dnp3.last_blk_sz > 0u)
                                  ? (s_dnp3.blks_done == s_dnp3.n_blocks + 1u)
                                  : (s_dnp3.blks_done == s_dnp3.n_blocks);
                    if (all_done) {
                        eFrameDir dir = (s_dnp3.ctrl & 0x80u)
                                      ? FRAME_DIR_MASTER : FRAME_DIR_SLAVE;
                        emit_frame(dir);
                        reset_rx(); s_dnp3.state = DNP3_IDLE;
                    }
                }
            }
            break;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * PROTOCOL: IEC 60870-5-101 and IEC 60870-5-103  (both use FT1.2)
 * ══════════════════════════════════════════════════════════════════════
 * Three frame types:
 *   Single char  0xE5                        → always slave (ACK)
 *   Fixed length 0x10 C A CS 0x16            → 5 bytes total
 *   Variable len 0x68 L L 0x68 <L bytes> CS 0x16
 *
 * Direction: Control byte C bit 6 (PRM bit in IEC 60870-5-1 notation):
 *   PRM=1 (C & 0x40) → primary station (master)
 *   PRM=0            → secondary station (slave)
 * ══════════════════════════════════════════════════════════════════════ */

typedef enum {
    FT12_IDLE = 0,
    FT12_FIXED,       /* collecting C + A + CS + 0x16 */
    FT12_VAR_L1,      /* waiting for first L byte       */
    FT12_VAR_L2,      /* waiting for second L byte      */
    FT12_VAR_START2,  /* waiting for repeated 0x68      */
    FT12_VAR_DATA,    /* collecting L + CS + 0x16       */
} eFt12State;

static struct {
    eFt12State state;
    uint8_t    L;              /* variable frame user-data length       */
    uint16_t   bytes_left;     /* bytes to receive to complete frame    */
    uint8_t    ctrl;           /* C field captured for direction decode */
    bool       ctrl_captured;
} s_ft12;

static void handle_ft12(uint8_t byte)
{
    if (!rx_append(byte)) { s_ft12.state = FT12_IDLE; return; }

    switch (s_ft12.state) {
        case FT12_IDLE:
            /* Every time we get here, restart the buffer from this byte */
            s_rx_len    = 1u;
            s_rx_buf[0] = byte;

            if (byte == 0xE5u) {
                /* Single-character ACK — always from secondary (slave) */
                emit_frame(FRAME_DIR_SLAVE);
                reset_rx();
            } else if (byte == 0x10u) {
                s_ft12.bytes_left    = 4u; /* C + A + CS + 0x16 */
                s_ft12.ctrl_captured = false;
                s_ft12.state         = FT12_FIXED;
            } else if (byte == 0x68u) {
                s_ft12.state = FT12_VAR_L1;
            } else {
                reset_rx(); /* not a valid start byte */
            }
            break;

        case FT12_FIXED:
            if (!s_ft12.ctrl_captured) {
                s_ft12.ctrl         = byte;
                s_ft12.ctrl_captured = true;
            }
            s_ft12.bytes_left--;
            if (s_ft12.bytes_left == 0u) {
                if (byte != 0x16u) { reset_rx(); s_ft12.state = FT12_IDLE; break; }
                eFrameDir dir = (s_ft12.ctrl & 0x40u) ? FRAME_DIR_MASTER : FRAME_DIR_SLAVE;
                emit_frame(dir);
                reset_rx(); s_ft12.state = FT12_IDLE;
            }
            break;

        case FT12_VAR_L1:
            s_ft12.L     = byte;
            s_ft12.state = FT12_VAR_L2;
            break;

        case FT12_VAR_L2:
            if (byte != s_ft12.L) { reset_rx(); s_ft12.state = FT12_IDLE; break; }
            s_ft12.state = FT12_VAR_START2;
            break;

        case FT12_VAR_START2:
            if (byte != 0x68u) { reset_rx(); s_ft12.state = FT12_IDLE; break; }
            /* After the second 0x68 we expect: L user-data bytes + CS + 0x16 */
            s_ft12.bytes_left    = (uint16_t)(s_ft12.L) + 2u;
            s_ft12.ctrl_captured = false;
            s_ft12.state         = FT12_VAR_DATA;
            break;

        case FT12_VAR_DATA:
            if (!s_ft12.ctrl_captured) {
                s_ft12.ctrl         = byte; /* first byte = C field */
                s_ft12.ctrl_captured = true;
            }
            s_ft12.bytes_left--;
            if (s_ft12.bytes_left == 0u) {
                if (byte != 0x16u) { reset_rx(); s_ft12.state = FT12_IDLE; break; }
                eFrameDir dir = (s_ft12.ctrl & 0x40u) ? FRAME_DIR_MASTER : FRAME_DIR_SLAVE;
                emit_frame(dir);
                reset_rx(); s_ft12.state = FT12_IDLE;
            }
            break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * PROTOCOL: BACnet MS/TP
 * ══════════════════════════════════════════════════════════════════════
 * Frame:
 *   [0x55][0xFF][Type][Dst][Src][Len_H][Len_L][Hdr_CRC]
 *   [<Len data bytes>][Data_CRC_L][Data_CRC_H]   (if Len > 0)
 *
 * Direction: Source MAC ≤ max_master → master, else → slave.
 * ══════════════════════════════════════════════════════════════════════ */

typedef enum {
    BACNET_IDLE = 0,
    BACNET_PREAMBLE,  /* got 0x55, waiting for 0xFF */
    BACNET_HEADER,    /* collecting 6 header bytes after the preamble */
    BACNET_DATA,
} eBacnetState;

static struct {
    eBacnetState state;
    uint8_t      header_pos;
    uint8_t      src_mac;
    uint16_t     data_len;
    uint16_t     bytes_left;
    uint8_t      max_master;
} s_bacnet;

static void handle_bacnet(uint8_t byte)
{
    if (!rx_append(byte)) { s_bacnet.state = BACNET_IDLE; return; }

    switch (s_bacnet.state) {
        case BACNET_IDLE:
            if (byte == 0x55u) {
                s_rx_len = 1u; s_rx_buf[0] = byte;
                s_bacnet.state = BACNET_PREAMBLE;
            } else {
                s_rx_len = 0u;
            }
            break;

        case BACNET_PREAMBLE:
            if (byte == 0xFFu) {
                s_bacnet.header_pos = 0u;
                s_bacnet.state      = BACNET_HEADER;
            } else if (byte == 0x55u) {
                s_rx_len = 1u; s_rx_buf[0] = byte; /* new preamble start */
            } else {
                reset_rx(); s_bacnet.state = BACNET_IDLE;
            }
            break;

        case BACNET_HEADER:
            /* Bytes after 0x55 0xFF: Type(1) Dst(1) Src(1) Len_H(1) Len_L(1) CRC(1) */
            s_bacnet.header_pos++;
            switch (s_bacnet.header_pos) {
                /* header_pos 1: frame Type  → s_rx_buf[2] */
                case 2u: /* Dst */ break;
                case 3u: s_bacnet.src_mac  = byte;  break;          /* Src */
                case 4u: s_bacnet.data_len = (uint16_t)((uint16_t)byte << 8); break; /* Len_H */
                case 5u: s_bacnet.data_len |= (uint16_t)byte;       break;            /* Len_L */
                case 6u: {
                    /* CRC byte: validate over s_rx_buf[2..6] (Type,Dst,Src,Len_H,Len_L) */
                    uint8_t crc_calc = crc8_bacnet_hdr(&s_rx_buf[2], 5u);
                    if ((crc_calc ^ 0xFFu) != byte) {
                        reset_rx(); s_bacnet.state = BACNET_IDLE; break;
                    }
                    eFrameDir dir = (s_bacnet.src_mac <= s_bacnet.max_master)
                                  ? FRAME_DIR_MASTER : FRAME_DIR_SLAVE;
                    if (s_bacnet.data_len == 0u) {
                        emit_frame(dir);
                        reset_rx(); s_bacnet.state = BACNET_IDLE;
                    } else {
                        /* data_len payload bytes + 2 data CRC bytes */
                        s_bacnet.bytes_left = s_bacnet.data_len + 2u;
                        s_bacnet.state      = BACNET_DATA;
                    }
                    break;
                }
                default: break;
            }
            break;

        case BACNET_DATA:
            s_bacnet.bytes_left--;
            if (s_bacnet.bytes_left == 0u) {
                eFrameDir dir = (s_bacnet.src_mac <= s_bacnet.max_master)
                              ? FRAME_DIR_MASTER : FRAME_DIR_SLAVE;
                emit_frame(dir);
                reset_rx(); s_bacnet.state = BACNET_IDLE;
            }
            break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * PROTOCOL: DMX512
 * ══════════════════════════════════════════════════════════════════════
 * Frame boundary: BREAK (line low > 88 µs) detected as a UART framing
 * error by HAL_UART_ErrorCallback, which sets s_dmx_break.
 * Frame: start code (1 byte) + up to 512 slot bytes = 513 bytes max.
 * Direction: always MASTER (controller → fixtures).
 * UART settings for DMX512: 250 000 baud, 8N2 (overrides config fields).
 * ══════════════════════════════════════════════════════════════════════ */

static volatile bool s_dmx_break;

static struct {
    bool in_frame;
} s_dmx;

static void handle_dmx(uint8_t byte)
{
    if (s_dmx_break) {
        s_dmx_break    = false;
        s_dmx.in_frame = true;
        reset_rx();
        rx_append(byte); /* start code is first byte after break */
        return;
    }
    if (!s_dmx.in_frame) { return; }
    if (!rx_append(byte)) {
        /* Buffer full (513 bytes): emit current frame */
        emit_frame(FRAME_DIR_MASTER);
        reset_rx(); s_dmx.in_frame = false;
        return;
    }
    /* Standard DMX512: 1 start code + 512 slots = 513 bytes total */
    if (s_rx_len >= 513u) {
        emit_frame(FRAME_DIR_MASTER);
        reset_rx(); s_dmx.in_frame = false;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * PROTOCOL: RAW / CUSTOM
 * ══════════════════════════════════════════════════════════════════════
 * Three framing modes (from sRawParams):
 *   1. End-byte delimiter  (use_end_byte  = 1)
 *   2. Fixed frame length  (fixed_len     > 0)
 *   3. 50 ms idle timeout  (neither of the above)
 * A start-byte delimiter can optionally gate frame collection.
 * ══════════════════════════════════════════════════════════════════════ */

#define RAW_IDLE_TIMEOUT_MS  50u

static struct {
    sRawParams params;
    uint16_t   timeout_cnt;
    bool       in_frame;
} s_raw;

static void handle_raw(uint8_t byte)
{
    /* If start byte is configured, wait for it before collecting */
    if (s_raw.params.use_start_byte && !s_raw.in_frame) {
        if (byte == s_raw.params.start_byte) {
            reset_rx();
            s_raw.in_frame = true;
            s_raw.timeout_cnt = 0u;
        }
        return;
    }
    s_raw.in_frame = true;

    if (!rx_append(byte)) {
        emit_frame(FRAME_DIR_UNKNOWN);
        reset_rx(); s_raw.in_frame = false; return;
    }
    s_raw.timeout_cnt = 0u;

    if (s_raw.params.use_end_byte && (byte == s_raw.params.end_byte)) {
        emit_frame(FRAME_DIR_UNKNOWN);
        reset_rx(); s_raw.in_frame = false; return;
    }
    if ((s_raw.params.fixed_len > 0u) && (s_rx_len >= (uint16_t)s_raw.params.fixed_len)) {
        emit_frame(FRAME_DIR_UNKNOWN);
        reset_rx(); s_raw.in_frame = false;
    }
}

static void raw_timer_tick(void)
{
    if (!s_raw.in_frame || s_rx_len == 0u) { return; }
    /* Timeout framing is only active when no other delimiter is configured */
    if (s_raw.params.use_end_byte || s_raw.params.fixed_len) { return; }
    if (++s_raw.timeout_cnt >= RAW_IDLE_TIMEOUT_MS) {
        emit_frame(FRAME_DIR_UNKNOWN);
        reset_rx(); s_raw.in_frame = false; s_raw.timeout_cnt = 0u;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * UART2 REINITIALISATION
 * ══════════════════════════════════════════════════════════════════════ */

static void prvReinitUart2(const sRs485Config *cfg)
{
    /* Stop any active DMA transfer before reconfiguring */
    HAL_UART_DMAStop(&huart2);
    HAL_UART_DeInit(&huart2);   /* calls HAL_UART_MspDeInit → HAL_DMA_DeInit */

    bool is_dmx = (cfg->protocol == PROTO_DMX512);

    uint32_t baud   = is_dmx ? 250000u : BAUD_TABLE[cfg->baud_rate];
    uint32_t parity = is_dmx ? UART_PARITY_NONE : PARITY_HAL[cfg->parity];
    uint32_t stop   = is_dmx ? UART_STOPBITS_2  : STOP_HAL[cfg->stop_bits];
    /* STM32 UART needs 9-bit word length when hardware parity is enabled */
    uint32_t wlen   = (parity != UART_PARITY_NONE) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;

    huart2.Init.BaudRate     = baud;
    huart2.Init.WordLength   = wlen;
    huart2.Init.StopBits     = stop;
    huart2.Init.Parity       = parity;
    huart2.Init.Mode         = UART_MODE_RX;  /* passive sniffer: receive only */
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    /* HAL_UART_Init → HAL_UART_MspInit → re-inits DMA in NORMAL mode and
       starts HAL_UART_Receive_DMA(huart, SerialBuffer, 256).
       We immediately override DMA mode to CIRCULAR with the full 512-byte
       buffer so DMA never stops mid-frame.  This survives CubeMX
       regeneration without touching the MSP file.                     */
    HAL_UART_Init(&huart2);

    HAL_UART_DMAStop(&huart2);                   /* stop the MSP-started transfer */
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;     /* never stops, wraps automatically */
    HAL_DMA_Init(&hdma_usart2_rx);
    HAL_UART_Receive_DMA(&huart2, SerialBuffer, DMA_BUF_SIZE);

    /* Enable UART IDLE line interrupt so vRs485Parser_IdleDetected()
       fires at the natural end of each bus frame.
       Priority 0 matches DMA1_Stream5 — no mutual preemption.        */
    HAL_NVIC_SetPriority(USART2_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/* ══════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════ */

void vRs485Parser_Init(const sRs485Config *cfg)
{
    s_protocol    = cfg->protocol;
    s_log_dir     = cfg->log_dir;
    s_frame_ready = false;
    s_dmx_break   = false;
    s_dma_head    = 0u;
    reset_rx();

    /* Zero all per-protocol state */
    memset(&s_mbrtu,   0, sizeof(s_mbrtu));
    memset(&s_mbascii, 0, sizeof(s_mbascii));
    memset(&s_dnp3,    0, sizeof(s_dnp3));
    memset(&s_ft12,    0, sizeof(s_ft12));
    memset(&s_bacnet,  0, sizeof(s_bacnet));
    memset(&s_dmx,     0, sizeof(s_dmx));
    memset(&s_raw,     0, sizeof(s_raw));

    /* Protocol-specific one-time setup */
    switch (cfg->protocol) {
        case PROTO_BACNET_MSTP:
            s_bacnet.max_master = cfg->params.bacnet.max_master;
            break;
        case PROTO_RAW:
            s_raw.params = cfg->params.raw;
            break;
        default:
            break;
    }

    /* Reconfigure UART2 baud/parity, restart DMA, enable IDLE interrupt.
       HAL_UART_MspInit (called inside) already starts DMA receive into
       SerialBuffer[DMA_BUF_SIZE], so no extra Receive_DMA call needed.  */
    prvReinitUart2(cfg);
}

void vRs485Parser_RxByte(uint8_t byte)
{
    switch (s_protocol) {
        case PROTO_MODBUS_RTU:   handle_mbrtu(byte);   break;
        case PROTO_MODBUS_ASCII: handle_mbascii(byte); break;
        case PROTO_DNP3:         handle_dnp3(byte);    break;
        case PROTO_IEC101:       /* fall-through */
        case PROTO_IEC103:       handle_ft12(byte);    break;
        case PROTO_BACNET_MSTP:  handle_bacnet(byte);  break;
        case PROTO_DMX512:       handle_dmx(byte);     break;
        case PROTO_RAW:          handle_raw(byte);     break;
        default:                                       break;
    }
}

/* ── DMA drain helper ─────────────────────────────────────────────── */

/* Feed every new byte that DMA has written since s_dma_head into the
   active protocol state machine.  DMA_CIRCULAR wraps the write position,
   so we use != with modulo instead of a simple < comparison.           */
static void prvDrainDMA(void)
{
    uint16_t write_pos = (uint16_t)(DMA_BUF_SIZE
                       - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx))
                       % DMA_BUF_SIZE;
    while (s_dma_head != write_pos) {
        vRs485Parser_RxByte(SerialBuffer[s_dma_head]);
        s_dma_head = (uint16_t)((s_dma_head + 1u) % DMA_BUF_SIZE);
    }
}

/* ── Public API (continued) ────────────────────────────────────────── */

void vRs485Parser_IdleDetected(void)
{
    /* Drain all bytes DMA has written since the last drain */
    prvDrainDMA();

    /* For Modbus RTU, the UART IDLE condition IS the inter-frame gap.
       Process the accumulated bytes as a complete frame right now,
       which eliminates the need for a software 3.5-char timer.        */
    if (s_protocol == PROTO_MODBUS_RTU && s_rx_len > 0u) {
        mbrtu_process();
    }
}

void vRs485Parser_TimerTick(void)
{
    /* Modbus RTU uses hardware IDLE, not this timer.
       Only Raw/Custom needs the 50 ms idle timeout.  */
    if (s_protocol == PROTO_RAW) {
        raw_timer_tick();
    }
}

bool bRs485Parser_FrameReady(void)
{
    return s_frame_ready;
}

void vRs485Parser_GetFrame(sRs485Frame *out)
{
    *out          = s_frame;
    s_frame_ready = false;
}

uint16_t u16Rs485Parser_FormatLog(const sRs485Frame *frame,
                                   uint8_t *out, uint16_t maxLen)
{
    static const char * const DIR_TAG[] = {"M", "S", "?"};
    static const uint8_t      HEX[]     = "0123456789ABCDEF";
    uint16_t pos = 0u;

    /* "PROTO," */
    const char *tag = PROTO_TAG[s_protocol];
    for (const char *p = tag; *p && pos < maxLen; p++) { out[pos++] = (uint8_t)*p; }
    if (pos < maxLen) { out[pos++] = (uint8_t)','; }

    /* "DIR," */
    const char *dir = DIR_TAG[frame->dir];
    for (const char *p = dir; *p && pos < maxLen; p++) { out[pos++] = (uint8_t)*p; }
    if (pos < maxLen) { out[pos++] = (uint8_t)','; }

    /* "XX XX XX ..." — as many bytes as fit */
    for (uint16_t i = 0u; i < frame->len; i++) {
        if ((pos + 2u) > maxLen) { break; }
        if (i > 0u && pos < maxLen) { out[pos++] = (uint8_t)' '; }
        if ((pos + 2u) > maxLen) { break; }
        out[pos++] = HEX[(frame->data[i] >> 4) & 0x0Fu];
        out[pos++] = HEX[ frame->data[i]       & 0x0Fu];
    }

    return pos;
}

/* ══════════════════════════════════════════════════════════════════════
 * HAL UART CALLBACKS  (override weak symbols)
 * ══════════════════════════════════════════════════════════════════════ */

/* Safety drain at the halfway point of the circular buffer.
   In normal operation IDLE has already drained these bytes; this is a
   fallback for protocols that never go idle (e.g., continuous DMX).   */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        prvDrainDMA();
    }
}

/* Safety drain when DMA wraps back to position 0.
   DMA_CIRCULAR restarts automatically — no HAL_UART_Receive_DMA call
   needed here.                                                         */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        prvDrainDMA();
    }
}

/* Called by UART ISR on framing / parity / overrun errors.
   For DMX512 a framing error means a BREAK signal was detected.       */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if ((s_protocol == PROTO_DMX512) &&
            (huart->ErrorCode & HAL_UART_ERROR_FE)) {
            s_dmx_break = true;
        }
        /* DMA continues running after a UART error — no restart needed */
    }
}
