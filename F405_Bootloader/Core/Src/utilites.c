/*
 * utilites.c
 *
 *  Created on: 30 июн. 2021 г.
 *      Author: ivan
 */
#include "utilites.h"

filterType currentFilter[CHANNELS];

int32_t filtering(int32_t input_data, filterType * flt)
{

    flt->sum -= flt->filterData[(int16_t)flt->top];        // отнять от суммы значение на которое указывает top
    flt->filterData[(int16_t)flt->top] = input_data;       // запомнить значение по top
    if(++flt->top > FILTER_LEN-1) flt->top = 0;             // увеличить указатель top, если он больше длины фильтра установить в начало
    return (int32_t)((flt->sum += input_data)/FILTER_LEN);  // к сумме прибавить новое значение и вернуть среднее значение

}

uint8_t get_check_sum(uint8_t *source, uint8_t size) {
	uint8_t i, uSum = 0;
	for (i = 0; i < size - 2; i++) {
		uSum = uSum + source[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}


