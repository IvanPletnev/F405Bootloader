/*
 * crc32.h
 *
 *  Created on: Apr 5, 2022
 *      Author: ivan
 */

#ifndef INC_CRC32_H_
#define INC_CRC32_H_

extern uint32_t crc32_bytes(CRC_HandleTypeDef *hcrc, uint8_t *buf, int len);

#endif /* INC_CRC32_H_ */
