/*
 * ble_handler.c
 *
 *  Created on: May 25, 2026
 *      Author: Bruno Vicente
 */

#include "ble_handler.h"
#include "rn4871.h"
#include "hardware_abstraction.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;
extern RTC_HandleTypeDef  hrtc;
extern struct car t24;

/* ── DMA RX circular buffer ─────────────────────────────────────────────── */

static uint8_t  dma_rx[BLE_RX_BUF_SIZE];
static uint16_t rx_head = 0;        // our read position in the circular buffer

static uint8_t  ble_tx_buf[BLE_TX_BUF_SIZE];

/* ── State ───────────────────────────────────────────────────────────────── */

static BLE_STATE_MACHINE_t ble_state     = BLE_IDLE;
static CTS_Result          ble_result    = CTS_RESULT_PENDING;
static uint32_t            state_tick    = 0;
static uint8_t             time_synced   = 0;

/* ── Log flush state ─────────────────────────────────────────────────────── */

static uint8_t  logs_paused   = 0;
static uint8_t  flushing      = 0;
static uint8_t  flush_index   = 0;

#define MAX_EVENTS  20      // must match your EEPROM layout

/* ── Internal helpers ────────────────────────────────────────────────────── */

static void ble_send(const char *s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);
}

static void go_to(BLE_STATE_MACHINE_t s) {
    ble_state  = s;
    state_tick = millis();
}

static uint8_t timed_out(uint32_t limit_ms) {
    return (millis() - state_tick) > limit_ms;
}

/* Peek at unread DMA bytes without consuming them.
   Returns 1 if needle is found, and advances rx_head past it. */
static uint8_t rx_contains(const char *needle) {
    uint16_t write_head = BLE_RX_BUF_SIZE -
        __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    char tmp[BLE_RX_BUF_SIZE + 1];
    uint16_t n = 0, i = rx_head;

    while (i != write_head) {
        tmp[n++] = dma_rx[i];
        i = (i + 1) % BLE_RX_BUF_SIZE;
    }
    tmp[n] = '\0';

    if (strstr(tmp, needle)) {
        rx_head = write_head;
        return 1;
    }
    return 0;
}

/* Copy all unread DMA bytes into dst and advance rx_head. */
static uint16_t rx_read(char *dst, uint16_t max) {
    uint16_t write_head = BLE_RX_BUF_SIZE -
        __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    uint16_t n = 0, i = rx_head;
    while (i != write_head && n < max - 1) {
        dst[n++] = dma_rx[i];
        i = (i + 1) % BLE_RX_BUF_SIZE;
    }
    dst[n] = '\0';
    rx_head = write_head;
    return n;
}

/* Fall through to EXIT_CMD on any failure — bridge always starts */
static void abort_to_exit(CTS_Result r) {
    ble_result = r;
    go_to(BLE_EXIT_CMD);
}

/* ── CTS byte parser ─────────────────────────────────────────────────────── */

/* RN4871 CHR response format: "CHR,<handle>,<len>,<hexbytes>\r\n"
   CTS payload (0x2A2B), 10 bytes:
     [0-1] year (LE uint16), [2] month, [3] day,
     [4] hour, [5] min, [6] sec, [7] day-of-week, [8-9] fractions  */
static uint8_t parse_cts_and_set_rtc(const char *raw) {
    /* Find hex payload — third comma onwards */
    uint8_t commas = 0;
    const char *p = raw;
    while (*p && commas < 3) {
        if (*p++ == ',') commas++;
    }
    if (commas < 3 || strlen(p) < 14) return 0;

    uint8_t b[7];
    for (int i = 0; i < 7; i++) {
        char hex[3] = { p[i*2], p[i*2+1], '\0' };
        b[i] = (uint8_t)strtol(hex, NULL, 16);
    }

    uint16_t year  = b[0] | ((uint16_t)b[1] << 8);
    uint8_t  month = b[2];
    uint8_t  day   = b[3];
    uint8_t  hour  = b[4];
    uint8_t  min   = b[5];
    uint8_t  sec   = b[6];

    /* Sanity check before touching RTC */
    if (year < 2024 || year > 2099) return 0;
    if (month < 1   || month > 12)  return 0;
    if (day   < 1   || day   > 31)  return 0;
    if (hour  > 23  || min   > 59 || sec > 59) return 0;

    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};
    t.Hours   = hour;
    t.Minutes = min;
    t.Seconds = sec;
    d.Year    = (uint8_t)(year - 2000);
    d.Month   = month;
    d.Date    = day;

    HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);
    return 1;
}

/* ── EEPROM log flush helpers ────────────────────────────────────────────── */

/* Format one EEPROM event record as human-readable text and TX over UART2.
   Called once per main loop tick while flushing — never blocks. */
static void flush_one_record(uint8_t index) {
    /* TODO: read EventRecord from EEPROM at:
       address = LOG_BASE + index * RECORD_SIZE
       e.g. eeprom_read(LOG_BASE + index * RECORD_SIZE, &rec, sizeof(rec)); */

    /* TODO: validate record:
       if (rec.magic != 0xA5 || !crc_valid(&rec)) → skip, send "EMPTY\r\n" */

    /* TODO: format record to ble_tx_buf with snprintf, example layout:
       BOOT:%u UPTIME:%lu [TS:%lu]\r\n
       EVENT:%s SEV:%u\r\n
       PC:0x%08lX LR:0x%08lX\r\n
       CFSR:0x%08lX HFSR:0x%08lX\r\n
       STATE:%u SDC:0x%02X\r\n
       CRC:0x%04X\r\n
       ---\r\n  */

    /* TODO: transmit formatted record:
       int len = snprintf(ble_tx_buf, sizeof(ble_tx_buf), ...);
       HAL_UART_Transmit(&huart2, ble_tx_buf, len, 500);  */

    /* Placeholder so code compiles now */
    int len = snprintf((char*)ble_tx_buf, sizeof(ble_tx_buf),
                       "SLOT:%u (EEPROM not yet implemented)\r\n---\r\n",
                       index);
    HAL_UART_Transmit(&huart2, ble_tx_buf, len, 500);
}

/* ── Periodic telemetry TX ───────────────────────────────────────────────── */

static void send_telemetry_log(void) {
    int len = snprintf((char*)ble_tx_buf, sizeof(ble_tx_buf),
        "TEMP:%.2f REAR_P:%.2f FRONT_P:%.2f STATE:%u MISSION:%u\r\n",
        t24.chip_temp,
        t24.Rear_Pressure.Pneumatic,
        t24.Front_Pressure.Pneumatic,
        (uint8_t)t24.Autonomous_State,
        (uint8_t)t24.Current_Mission);

    HAL_UART_Transmit_DMA(&huart2, ble_tx_buf, len);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void ble_handler_init(void) {
    /* Start DMA circular RX — runs forever, never needs restarting */
    HAL_UART_Receive_DMA(&huart2, dma_rx, BLE_RX_BUF_SIZE);
    go_to(BLE_ENTER_CMD);
}

void ble_handler(void) {
    char cmd_buf[64];
    int  len;

    switch (ble_state) {

    /* ── Boot sequence: CTS time sync ──────────────────────────────────── */

    case BLE_IDLE:
        /* Should not stay here — init moves us to ENTER_CMD */
        go_to(BLE_ENTER_CMD);
        break;

    case BLE_ENTER_CMD:
        /* $$$ has NO \r\n — this is intentional */
        HAL_UART_Transmit(&huart2, (uint8_t*)"$$$", 3, 100);
        go_to(BLE_CONNECT);     // reuse enum as WAIT_CMD state
        /* NOTE: we repurpose BLE_CONNECT momentarily as WAIT_CMD
           Add BLE_WAIT_CMD to your enum for cleaner code */
        break;

    case BLE_CONNECT:
        /* Waiting for "CMD>" then sending connect command */
        if (rx_contains("CMD>")) {
            len = rn4871_connect(cmd_buf, sizeof(cmd_buf),
                                 BLE_MAC, BLE_MAC_ADDR_TYPE);
            if (len > 0) ble_send(cmd_buf);
            go_to(BLE_READ_CURRENT_TIME);   // repurposed as WAIT_CONNECTED
        } else if (timed_out(TIMEOUT_CMD_MS)) {
            abort_to_exit(CTS_RESULT_TIMEOUT);
        }
        break;

    case BLE_READ_CURRENT_TIME:
        /* Waiting for "CONNECTED" then reading CTS characteristic */
        if (rx_contains("CONNECTED")) {
            HAL_Delay(300);     // let connection settle
            /* 0x000B is the typical CTS handle — verify with your phone */
            /* TODO: optionally do LS command first to discover handle */
            len = rn4871_read_remote_char(cmd_buf, sizeof(cmd_buf), 0x000B);
            if (len > 0) ble_send(cmd_buf);
            go_to(BLE_PROCESS_DATE);    // repurposed as WAIT_CTS
        } else if (timed_out(TIMEOUT_CONNECT_MS)) {
            abort_to_exit(CTS_RESULT_NO_DEVICE);
        }
        break;

    case BLE_PROCESS_DATE:
        /* Waiting for CHR response, then parsing */
        if (rx_contains("CHR")) {
            char raw[BLE_RX_BUF_SIZE];
            rx_read(raw, sizeof(raw));
            if (parse_cts_and_set_rtc(raw)) {
                time_synced = 1;
                ble_result  = CTS_RESULT_OK;
            } else {
                ble_result  = CTS_RESULT_PARSE_ERROR;
            }
            go_to(BLE_EXIT_CMD);
        } else if (rx_contains("ERR") || rx_contains("DISCONNECT")) {
            abort_to_exit(CTS_RESULT_PARSE_ERROR);
        } else if (timed_out(TIMEOUT_CTS_MS)) {
            abort_to_exit(CTS_RESULT_TIMEOUT);
        }
        break;

    case BLE_EXIT_CMD:
        /* Disconnect and return to transparent bridge mode */
        len = rn4871_disconnect(cmd_buf, sizeof(cmd_buf));
        if (len > 0) ble_send(cmd_buf);
        HAL_Delay(200);
        len = rn4871_cmd_exit_mode(cmd_buf, sizeof(cmd_buf));
        if (len > 0) ble_send(cmd_buf);
        go_to(BLE_BRIDGE);
        break;

    /* ── Normal operation: bridge mode ─────────────────────────────────── */

    case BLE_BRIDGE: {

        /* 1. Check for incoming commands from phone */
        if (rx_contains("stop\r\n")) {
            logs_paused = 1;
        } else if (rx_contains("start\r\n")) {
            logs_paused = 0;
            flushing    = 0;
        } else if (rx_contains("flush\r\n")) {
            logs_paused  = 1;
            flushing     = 1;
            flush_index  = 0;
            /* Send header so phone knows dump is starting */
            ble_send("=== EVENT LOG ===\r\n");
        }

        /* 2. If flushing: send one record per tick, never block */
        if (flushing) {
            if (flush_index < MAX_EVENTS) {
                flush_one_record(flush_index);
                flush_index++;
            } else {
                /* Flush complete */
                ble_send("=== END LOG ===\r\n");
                flushing    = 0;
                logs_paused = 0;
            }
            break;   // skip telemetry this tick while flushing
        }

        /* 3. Periodic telemetry — only when not paused */
        if (!logs_paused) {
            static uint32_t log_tick = 0;
            if (millis() - log_tick > 1000) {
                send_telemetry_log();
                log_tick = millis();
            }
        }

        break;
    }

    default:
        go_to(BLE_BRIDGE);
        break;
    }
}

uint8_t ble_time_synced(void) {
    return time_synced;
}
