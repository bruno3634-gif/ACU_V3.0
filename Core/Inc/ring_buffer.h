/*
 * ring_buffer.h
 *
 *  Created on: Apr 22, 2026
 *      Author: TC-Desenvolvimento
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include "main.h"
#include <stdint.h>
#include <string.h>


#define MAX_SIZE 64


struct ring{
	uint32_t head;
	uint32_t tail;
	uint32_t counter;
	struct can_queue tx_queue[MAX_SIZE];
};


extern CAN_HandleTypeDef hcan1;


void can_buffer_init(struct ring *ring_buffer);
void can_buffer_push(struct ring *ring_buffer,CAN_TxHeaderTypeDef  tx_header ,uint8_t data[8]);
void can_buffer_pop(struct ring *ring_buffer);


#endif /* INC_RING_BUFFER_H_ */
