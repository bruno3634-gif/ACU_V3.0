/*
 * hardware_abstraction.h
 *
 *  Created on: Mar 20, 2026
 *      Author: bruno
 */
#ifndef HW_ABS
#define HW_ABS

#include <stdint.h>

#if defined (__ARM_ARCH_7EM__)

#else
	#include <time.h>
#endif

uint32_t millis();

#endif
