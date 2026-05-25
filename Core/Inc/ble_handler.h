/*
 * ble_handler.h
 *
 *  Created on: May 25, 2026
 *      Author: Bruno Vicente
 */

#ifndef INC_BLE_HANDLER_H_
#define INC_BLE_HANDLER_H_
#include "main.h"


#include "stm32f4xx_hal.h"
#include "main.h"

#define BLE_MAC             "AABBCCDDEEFF"   // TODO: load from EEPROM
#define BLE_MAC_ADDR_TYPE   0               // 0 = public, 1 = random

#define BLE_RX_BUF_SIZE     256
#define BLE_TX_BUF_SIZE     512

#define TIMEOUT_CMD_MS      1000
#define TIMEOUT_CONNECT_MS  8000
#define TIMEOUT_CTS_MS      3000

void ble_handler_init(void);
void ble_handler(void);

uint8_t ble_time_synced(void);

#endif /* INC_BLE_HANDLER_H_ */
