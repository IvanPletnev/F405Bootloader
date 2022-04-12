/*
 * crc32.c
 *
 *  Created on: Apr 5, 2022
 *      Author: ivan
 */
#include "main.h"
#include "crc32.h"

uint32_t reverse_32(uint32_t value) {
	uint32_t result;

	__ASM volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
	return (result);
}

uint32_t CRC_CalcCRC (CRC_HandleTypeDef *hcrc, uint32_t data) {
	uint32_t temp;

	hcrc->State = HAL_CRC_STATE_BUSY;
	hcrc->Instance->DR = data;
	temp = hcrc->Instance->DR;
	hcrc->State = HAL_CRC_STATE_READY;
	return temp;
}


uint32_t crc32_bytes(CRC_HandleTypeDef *hcrc, uint8_t *buf, int len) {
	uint32_t *p = (uint32_t*) buf;
	uint32_t crc, crc_reg;

	hcrc->State = HAL_CRC_STATE_BUSY;
	__HAL_CRC_DR_RESET(hcrc);

	if (len >= 4) {
		while (len >= 4) {
			hcrc->Instance->DR = reverse_32(*p++);
			crc_reg = hcrc->Instance->DR;
			len -= 4;
		}
	} else {
		crc = 0xFFFFFFFF;
		crc_reg = CRC_CalcCRC(hcrc, 0xEBABAB);
	}
	crc = reverse_32(crc_reg);
	if (len) {
		CRC_CalcCRC(hcrc, crc_reg);
		switch (len) {
		case 1:
			crc_reg = CRC_CalcCRC(hcrc,reverse_32((*p & 0xFF) ^ crc) >> 24);
			crc = (crc >> 8) ^ reverse_32(crc_reg);
			break;
		case 2:
			crc_reg = CRC_CalcCRC(hcrc, reverse_32((*p & 0xFFFF) ^ crc) >> 16);
			crc = (crc >> 16) ^ reverse_32(crc_reg);
			break;
		case 3:
			crc_reg = CRC_CalcCRC(hcrc, reverse_32((*p & 0xFFFFFF) ^ crc) >> 8);
			crc = (crc >> 24) ^ reverse_32(crc_reg);
			break;
		}
	}
	return ~crc;
}
