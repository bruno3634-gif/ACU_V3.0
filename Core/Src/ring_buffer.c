/*
 * ring_buffer.c
 *
 *  Created on: Apr 22, 2026
 *      Author: TC-Desenvolvimento
 */

#include "ring_buffer.h"


void can_buffer_init(struct ring *ring_buffer) {
	ring_buffer->counter = 0;
	ring_buffer->head = 0;
	ring_buffer->tail = 0;
}

void can_buffer_push(struct ring *ring_buffer, CAN_TxHeaderTypeDef  tx_header,
		uint8_t data[8]) {
	if (ring_buffer->counter >= MAX_SIZE) {
		return;
	}
	ring_buffer->queue[ring_buffer->head].can_tx_header = tx_header;
	memcpy(ring_buffer->queue[ring_buffer->head].tx_data, data, 8);

	ring_buffer->head = (ring_buffer->head + 1) % MAX_SIZE;
	ring_buffer->counter++;
}

void can_rx_buffer_push(struct ring *ring_buffer, CAN_RxHeaderTypeDef  tx_header,
		uint8_t data[8]) {
	if (ring_buffer->counter >= MAX_SIZE) {
		return;
	}
	ring_buffer->queue[ring_buffer->head].arrival_time = HAL_GetTick();
	ring_buffer->queue[ring_buffer->head].can_rx_header = tx_header;
	memcpy(ring_buffer->queue[ring_buffer->head].tx_data, data, 8);

	ring_buffer->head = (ring_buffer->head + 1) % MAX_SIZE;
	ring_buffer->counter++;
}

void can_buffer_pop(struct ring *ring_buffer, uint8_t tx_or_rx,struct can_queue *can_rx) {
	if (ring_buffer->counter == 0) {
		return;
	}

	uint32_t mailbox;
	HAL_StatusTypeDef result = HAL_ERROR;

	if(tx_or_rx){
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
			result = HAL_CAN_AddTxMessage(&hcan1,
					&ring_buffer->queue[ring_buffer->tail].can_tx_header,
					ring_buffer->queue[ring_buffer->tail].tx_data, &mailbox);
		}
		if(result == HAL_OK){
			ring_buffer->tail = (ring_buffer->tail + 1) % MAX_SIZE;
			ring_buffer->counter--;
		}
	}else {
		memcpy(can_rx, &ring_buffer->queue[ring_buffer->tail], sizeof(ring_buffer->queue[ring_buffer->tail]));
        memset(&ring_buffer->queue[ring_buffer->tail], 0, sizeof(ring_buffer->queue[ring_buffer->tail]));
        ring_buffer->tail = (ring_buffer->tail + 1) % MAX_SIZE;
        ring_buffer->counter--;
    }



}
