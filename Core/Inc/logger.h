/*
 * logger.h
 *
 *  Created on: Jun 1, 2026
 *      Author: bruno
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include <stdint.h>
#include "APP.h"  // for your enums
#include <stdbool.h>
#include "ee24.h"
#include "main.h"

#define LOG_MAGIC        0xDEADBEEF
#define LOG_HEADER_ADDR  0
#define LOG_DATA_ADDR    8
#define LOG_MAX_ENTRIES  19

#define RX_BUFFER_SIZE 32
extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t uart_log_dump;

typedef struct {
    uint32_t magic;   // 0xDEADBEEF
    uint8_t  head;    // oldest entry index
    uint8_t  tail;    // next write index
    uint8_t  count;   // valid entries (0-19)
    uint8_t  pad;     // alignment
} log_header_t;      // 8 bytes

typedef struct {
	uint32_t can_hw_state;
    uint32_t timestamp;                  // HAL_GetTick()
    uint32_t pc;                         // program counter
    uint32_t lr;                         // link register
    uint32_t psr;                        // program status register
    AS_STATE_t as_state;                 // autonomous state
    current_mission_t mission;           // current mission
    Emergency_cause_t emergency_cause;   // cause of emergency/fault
    startup_sequence_state_t startup_state;
    float front_pressure_pneumatic;
    float rear_pressure_pneumatic;
    float front_pressure_hydraulic;
    float rear_pressure_hydraulic;
    uint8_t asms;
    uint8_t sdc_feedback;
    uint8_t pad[2];                      // keep 4-byte aligned
} log_entry_t;                           // 52 bytes


bool eeprom_log_init(EE24_HandleTypeDef *ee);
bool eeprom_log_write(EE24_HandleTypeDef *ee, log_entry_t *entry);
bool eeprom_log_read(EE24_HandleTypeDef *ee, uint8_t index, log_entry_t *entry);
bool eeprom_log_clear(EE24_HandleTypeDef *ee);
bool eeprom_log_count(EE24_HandleTypeDef *ee, uint8_t *count);




#endif /* INC_LOGGER_H_ */
