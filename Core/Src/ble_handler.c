/*
 * ble_handler.c
 *
 *  Created on: May 25, 2026
 *      Author: Bruno Vicente
 */

#include "ble_handler.h"
#include "APP.h"
#include "hardware_abstraction.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;
extern struct car t24;
extern Main_state_machine_t Vehicle_state_machine;

/* ── DMA RX circular buffer ─────────────────────────────────────────────── */

static uint8_t  dma_rx[BLE_RX_BUF_SIZE];
static uint16_t rx_head = 0;        // our read position in the circular buffer

static uint8_t  ble_tx_buf[BLE_TX_BUF_SIZE];

/* ── Shared TX busy flag (non-static — referenced by main.c TIM2 ISR) ────── */
volatile uint8_t ble_tx_busy = 0;
static uint32_t  ble_tx_tick = 0;

/* ── State ───────────────────────────────────────────────────────────────── */

static BLE_STATE_MACHINE_t ble_state     = BLE_IDLE;
static uint32_t            state_tick    = 0;

/* ── Log flush state ─────────────────────────────────────────────────────── */

static uint8_t  logs_paused   = 0;
static uint8_t  flushing      = 0;
static uint8_t  flush_index   = 0;

#define MAX_EVENTS  20      // must match your EEPROM layout

/* ── Internal helpers ────────────────────────────────────────────────────── */

static void ble_send_dma(const char *s) {
    /* ── critical section: atomic check-and-set ── */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return;                     /* drop — DMA still busy with previous TX */
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    /* ── copy and start DMA ── */
    size_t len = strlen(s);
    if (len >= sizeof(ble_tx_buf)) {
        len = sizeof(ble_tx_buf) - 1;   /* clamp to fit buffer (keep room for NUL) */
    }
    memcpy(ble_tx_buf, s, len);
    ble_tx_buf[len] = '\0';
    if (HAL_UART_Transmit_DMA(&huart2, ble_tx_buf, len) != HAL_OK) {
        ble_tx_busy = 0;               /* rollback on failure */
    }
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

/* ── Enum-to-string helpers ──────────────────────────────────────────────── */

static const char* state_name(Main_state_machine_t s) {
    switch(s) {
        case Start: return "START";
        case IDLE:  return "IDLE";
        case AS_ON: return "AS_ON";
        case EMERGENCY: return "EMERGENCY";
        default: return "?";
    }
}

static const char* as_state_name(AS_STATE_t s) {
    switch(s) {
        case AS_STATE_OFF:       return "OFF";
        case AS_STATE_READY:     return "READY";
        case AS_STATE_DRIVING:   return "DRIVING";
        case AS_STATE_EMERGENCY: return "EMERGENCY";
        case AS_STATE_FINISHED:  return "FINISHED";
        default: return "?";
    }
}

static const char* mission_name(current_mission_t m) {
    switch(m) {
        case MANUAL:       return "MANUAL";
        case ACCELERATION: return "ACCELERATION";
        case SKIDPAD:      return "SKIDPAD";
        case TRACKDRIVE:   return "TRACKDRIVE";
        case EBS_TEST:     return "EBS_TEST";
        case INSPECTION:   return "INSPECTION";
        case AUTOCROSS:    return "AUTOCROSS";
        default: return "?";
    }
}

static const char* assi_state_name(uint8_t a) {
    switch(a) {
        case AS_STATE_OFF:       return "OFF";
        case AS_STATE_READY:     return "YELLOW";
        case AS_STATE_DRIVING:   return "YELLOW FLASH";
        case AS_STATE_EMERGENCY: return "BLUE FLASH";
        case AS_STATE_FINISHED:  return "BLUE";
        default: return "?";
    }
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

    /* ── critical section: atomic check-and-set ── */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return;                     /* drop this record, next tick will retry */
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    /* ── format and start DMA ── */
    int len = snprintf((char*)ble_tx_buf, sizeof(ble_tx_buf),
                       "SLOT:%u (EEPROM not yet implemented)\r\n---\r\n",
                       index);
    if (HAL_UART_Transmit_DMA(&huart2, ble_tx_buf, len) != HAL_OK) {
        ble_tx_busy = 0;               /* rollback on failure */
    }
}

/* ── Periodic telemetry TX ───────────────────────────────────────────────── */

static void send_telemetry_log(void) {
    /* ── critical section: atomic check-and-set (snprintf kept outside) ── */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return;                     /* skip this tick — next one in 1 s */
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    int len = snprintf((char*)ble_tx_buf, sizeof(ble_tx_buf),
        "\r\n"
        "====================================\r\n"
        "      ACU V3.0 TELEMETRY\r\n"
        "====================================\r\n"
        "State: %s | AS: %s | Mission: %s\r\n"
        "Front:  Pneu=%.1f bar  Hyd=%.1f bar\r\n"
        "Rear:   Pneu=%.1f bar  Hyd=%.1f bar\r\n"
        "Sol:    Front=%s  Rear=%s\r\n"
        "Speed:  %u km/h  |  RPM: %d\r\n"
        "Temp:   %.1f C\r\n"
        "SDC:    %s  |  Ignition: %s\r\n"
        "ASMS:   %s  |  ASSI: %s\r\n"
        "Emergency: %s  |  Res: %u\r\n"
        "====================================\r\n",
        state_name(Vehicle_state_machine),
        as_state_name(t24.Autonomous_State),
        mission_name(t24.Current_Mission),
        t24.Front_Pressure.Pneumatic,
        t24.Front_Pressure.Hydraulic,
        t24.Rear_Pressure.Pneumatic,
        t24.Rear_Pressure.Hydraulic,
        t24.front_solenoid ? "ON " : "OFF",
        t24.rear_solenoid  ? "ON " : "OFF",
        (unsigned int)t24.Speed.Speed, t24.rpm,
        (double)t24.chip_temp,
        t24.SDC_feedback ? "OK" : "OPEN",
        t24.Ignition_Status ? "ON" : "OFF",
        t24.ASMS ? "ON" : "OFF",
        assi_state_name(t24.ASSI_state),
        t24.Emergency ? "ACTIVE" : "NONE",
        (unsigned int)t24.Res
    );
    if (HAL_UART_Transmit_DMA(&huart1, ble_tx_buf, len) != HAL_OK) {
        ble_tx_busy = 0;               /* rollback on failure */
    }
}

/* ── Binary telemetry TX (called from TIM2 ISR context at 10 Hz) ─────────── */

HAL_StatusTypeDef ble_send_binary(const uint8_t *data, uint16_t len) {
    /* ── critical section: atomic check-and-set ── */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return HAL_BUSY;            /* drop — DMA still busy with previous TX */
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    /* ── start DMA (data lives in caller's scope, e.g. static pkt in TIM2 ISR) ── */
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, len) != HAL_OK) {
        ble_tx_busy = 0;               /* rollback on failure */
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void ble_handler_init(void) {
    /* Start DMA circular RX — runs forever, never needs restarting.
       NOTE: this takes over UART2 RX from the earlier idle-line setup in main.c.
       The idle-line DMA is intentionally overridden — circular RX is what
       ble_handler needs to parse multi-byte commands without per-byte interrupts. */
    HAL_UART_Receive_DMA(&huart1, dma_rx, BLE_RX_BUF_SIZE);
    go_to(BLE_WAIT_CONFIG);
}

void ble_handler(void) {
    switch (ble_state) {

    case BLE_WAIT_CONFIG:
        if (ble_module_config_is_done() == 5) {
            go_to(BLE_BRIDGE);
        }
        break;

    case BLE_BRIDGE: {

        /* 0. DMA timeout recovery — abort if TX stuck >500 ms */
        if (ble_tx_busy && (HAL_GetTick() - ble_tx_tick > 500)) {
            HAL_UART_AbortTransmit(&huart2);  /* abort TX only — preserves circular RX DMA */
            ble_tx_busy = 0;
        }

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
            ble_send_dma("=== EVENT LOG ===\r\n");
        }

        /* 2. If flushing: send one record per tick */
        if (flushing) {
            if (flush_index < MAX_EVENTS) {
                flush_one_record(flush_index);
                flush_index++;
            } else {
                ble_send_dma("=== END LOG ===\r\n");
                flushing    = 0;
                logs_paused = 0;
            }
            break;
        }

        /* 3. Periodic telemetry — every 1000ms when not paused */
        if (!logs_paused) {
            static uint32_t log_tick = 0;
            uint32_t time_temp = millis();
            if (time_temp - log_tick > 1000) {
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

/* ── UART TX complete callback ────────────────────────────────────────────── */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        ble_tx_busy = 0;
    }
}
