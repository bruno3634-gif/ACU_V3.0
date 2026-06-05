/*
 * logger.c
 *
 *  Created on: Jun 1, 2026
 *      Author: bruno
 */

#include "logger.h"




bool eeprom_log_init(EE24_HandleTypeDef *ee) {
    log_header_t header = {0};

    if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
        return false;

    if (header.magic != LOG_MAGIC) {
        header.magic = LOG_MAGIC;
        header.head  = 0;
        header.tail  = 0;
        header.count = 0;
        return EE24_Write(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000);
    }

    return true;
}

bool eeprom_log_write(EE24_HandleTypeDef *ee, log_entry_t *entry) {
    log_header_t header = {0};

    if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
        return false;

    if (header.magic != LOG_MAGIC)
        return false;

    uint32_t addr = LOG_DATA_ADDR + (header.tail * sizeof(log_entry_t));
    if (!EE24_Write(ee, addr, (uint8_t*)entry, sizeof(log_entry_t), 1000))
        return false;

    header.tail = (header.tail + 1) % LOG_MAX_ENTRIES;

    if (header.count == LOG_MAX_ENTRIES)
        header.head = (header.head + 1) % LOG_MAX_ENTRIES;
    else
        header.count++;

    return EE24_Write(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000);
}

// index 0 = oldest, index count-1 = newest
bool eeprom_log_read(EE24_HandleTypeDef *ee, uint8_t index, log_entry_t *entry) {
    log_header_t header = {0};

    if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
        return false;

    if (header.magic != LOG_MAGIC)
        return false;

    if (index >= header.count)
        return false;

    uint8_t physical = (header.head + index) % LOG_MAX_ENTRIES;
    uint32_t addr = LOG_DATA_ADDR + (physical * sizeof(log_entry_t));

    return EE24_Read(ee, addr, (uint8_t*)entry, sizeof(log_entry_t), 1000);
}

// optional but useful
bool eeprom_log_clear(EE24_HandleTypeDef *ee) {
    log_header_t header = {0};
    header.magic = LOG_MAGIC;
    return EE24_Write(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000);
}

// get count without reading entries
bool eeprom_log_count(EE24_HandleTypeDef *ee, uint8_t *count) {
    log_header_t header = {0};

    if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
        return false;

    if (header.magic != LOG_MAGIC)
        return false;

    *count = header.count;
    return true;
}
