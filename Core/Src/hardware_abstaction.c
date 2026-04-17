/*
 * hardware_abstaction.c
 *
 *  Created on: Mar 23, 2026
 *      Author: bruno
 */


#include "hardware_abstraction.h"


uint32_t millis() {

#if defined (__ARM_ARCH_7EM__)

	return HAL_GetTick();

#else
	struct timespec ts;
	    clock_gettime(CLOCK_MONOTONIC, &ts);
	    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif

}
