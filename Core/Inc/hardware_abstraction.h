/*
 * hardware_abstraction.h
 *
 *  Created on: Mar 20, 2026
 *      Author: bruno
 */
#ifndef HW_ABS
#define HW_ABS

#include <stdint.h>
#include "main.h"

#if defined (__ARM_ARCH_7EM__)

#else
	#include <time.h>
#endif


extern struct car t24;

uint32_t millis();
void Peripheral_aquisition(uint8_t *assi_leds);
void Peripheral_actuation();

#endif
