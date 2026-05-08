/*
 * EMA_Filter.h
 *
 *  Created on: May 4, 2026
 *      Author: bruno
 */

#ifndef INC_EMA_FILTER_H_
#define INC_EMA_FILTER_H_

#include "main.h"
#include "stdbool.h"



typedef struct{
    float alpha;
    float output;
    bool  initialized;
}ema_data_structure;

void ema_init(ema_data_structure *f, float alpha);
float ema_update(ema_data_structure *f, float input);


#endif /* INC_EMA_FILTER_H_ */
