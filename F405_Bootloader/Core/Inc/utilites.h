/*
 * utilites.h
 *
 *  Created on: 30 июн. 2021 г.
 *      Author: ivan
 */

#ifndef INC_UTILITES_H_
#define INC_UTILITES_H_

#include "main.h"

#define FILTER_LEN 8
#define CHANNELS 1

typedef struct _filterType
{
    int32_t filterData[FILTER_LEN];    // данные фильтра
    int32_t sum;                        // текущая сумма
    int16_t top;                        // указатель на текущую выборку
} filterType;

// определяем масcив данных фильтра
extern filterType currentFilter[CHANNELS];        // как внешний

int32_t filtering(int32_t input_data, filterType * flt);
uint8_t get_check_sum(uint8_t *source, uint8_t size);


#endif /* INC_UTILITES_H_ */
