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

#define BLE_RX_BUF_SIZE     256
#define BLE_TX_BUF_SIZE     512

#define TIMEOUT_CMD_MS      1000

extern volatile uint8_t ble_tx_busy;

void ble_handler_init(void);
void ble_handler(void);
HAL_StatusTypeDef ble_send_binary(const uint8_t *data, uint16_t len);

#endif /* INC_BLE_HANDLER_H_ */
