/*
 * etx_ota_update.c
 *
 *  Created on: 26-Jul-2021
 *      Author: EmbeTronicX
 */

#include <stdio.h>
#include "etx_ota_update.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>
#include "crc32.h"
#include "utilites.h"


/* OTA State */
static ETX_OTA_STATE_ ota_state = ETX_OTA_STATE_IDLE;

/* Firmware Total Size that we are going to receive */
static uint32_t ota_fw_total_size;
/* Firmware image's CRC32 */
static uint32_t ota_fw_crc;
/* Firmware Size that we have received */
static uint32_t ota_fw_received_size;
/* Slot number to write the received firmware */
static uint8_t slot_num_to_write;
/* Configuration */
ETX_GNRL_CFG_ *cfg_flash = (ETX_GNRL_CFG_*) (CONFIG_FLASH_ADDR);

/* Hardware CRC handle */
extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern uint8_t rxBuffer[OTA_PACKET_MAX_SIZE];
extern uint8_t rxFlag;
extern uint16_t rxSize;


static ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len);
static void etx_ota_send_resp(uint8_t type);
static HAL_StatusTypeDef write_data_to_slot(uint8_t slot_num, uint8_t *data, uint16_t data_len, bool is_first_block);
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint32_t data_len);
static uint8_t get_available_slot_number(void);
static HAL_StatusTypeDef write_cfg_to_flash(ETX_GNRL_CFG_ *cfg);

/**
 * @brief Download the application from UART and flash it.
 * @param None
 * @retval ETX_OTA_EX_
 */
ETX_OTA_EX_ etx_ota_download_and_flash(void) {

	ETX_OTA_EX_ ret = ETX_OTA_EX_OK;
	uint32_t cal_data_crc = 0u;
	uint32_t rec_data_crc = 0u;
	uint16_t data_len;

	printf("Waiting for the OTA data...\r\n");

	/* Reset the variables */
	ota_fw_total_size = 0u;
	ota_fw_received_size = 0u;
	ota_fw_crc = 0u;
	ota_state = ETX_OTA_STATE_START;
	slot_num_to_write = 0xFFu;

	do {
		//clear the buffer
//		memset(rxBuffer, 0, ETX_OTA_PACKET_MAX_SIZE);
		while (!rxFlag);
		rxFlag = 0;
		if (rxSize != 0u) {
			data_len = *((uint16_t*)(rxBuffer+2));
			cal_data_crc = crc32_bytes(&hcrc, rxBuffer, data_len);
			rec_data_crc = *(uint32_t *) &rxBuffer[rxSize - 5];

			if( cal_data_crc != rec_data_crc )
			{
			  printf("Chunk's CRC mismatch [Cal CRC = 0x%08lX] [Rec CRC = 0x%08lX]\r\n", cal_data_crc, rec_data_crc );
			  ret = ETX_OTA_EX_ERR;
			} else {
				ret = etx_process_data(rxBuffer, rxSize);
			}

		} else {
			//didn't received data. break.
			printf("Zero Length\r\n");
			ret = ETX_OTA_EX_ERR;
		}

		//Send ACK or NACK
		if (ret != ETX_OTA_EX_OK) {
			printf("Sending NACK\r\n");
			etx_ota_send_resp(OTA_NACK);
			break;
		} else {
			//printf("Sending ACK\r\n");
			etx_ota_send_resp(OTA_ACK);
		}


	} while (ota_state != ETX_OTA_STATE_IDLE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, OTA_PACKET_MAX_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	return ret;
}

/**
 * @brief Process the received data from UART4.
 * @param buf buffer to store the received data
 * @param max_len maximum length to receive
 * @retval ETX_OTA_EX_
 */
static ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len) {

	ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

	do {
		if ((buf == NULL) || (len == 0u)) {
			break;
		}

		//Check we received OTA Abort command
		ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*) buf;
		if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD) {
			if (cmd->cmd == ETX_OTA_CMD_ABORT) {
				//received OTA Abort command. Stop the process
				break;
			}
		}

		switch (ota_state) {
		case ETX_OTA_STATE_IDLE: {
			printf("ETX_OTA_STATE_IDLE...\r\n");
			ret = ETX_OTA_EX_OK;
		}
			break;

		case ETX_OTA_STATE_START: {
			ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*) buf;

			if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD) {
				if (cmd->cmd == ETX_OTA_CMD_START) {
					printf("Received OTA START Command\r\n");
					ota_state = ETX_OTA_STATE_HEADER;
					ret = ETX_OTA_EX_OK;
				}
			}
		}
			break;

		case ETX_OTA_STATE_HEADER: {
			ETX_OTA_HEADER_ *header = (ETX_OTA_HEADER_*) buf;
			if (header->packet_type == ETX_OTA_PACKET_TYPE_HEADER) {
				ota_fw_total_size = header->meta_data.package_size;
				ota_fw_crc = header->meta_data.package_crc;
				printf("Received OTA Header. FW Size = %ld\r\n", ota_fw_total_size);

				//get the slot number
				slot_num_to_write = get_available_slot_number();
				if (slot_num_to_write != 0xFF) {
					ota_state = ETX_OTA_STATE_DATA;
					ret = ETX_OTA_EX_OK;
				}
			}
		}
			break;

		case ETX_OTA_STATE_DATA: {
			ETX_OTA_DATA_ *data = (ETX_OTA_DATA_*) buf;
			uint16_t data_len = data->data_len;
			HAL_StatusTypeDef ex;
			bool is_first_block = false;

			if (data->packet_type == ETX_OTA_PACKET_TYPE_DATA) {

				if (ota_fw_received_size == 0) {
					//This is the first block
					is_first_block = true;

					/* Read the configuration */
					ETX_GNRL_CFG_ cfg;
					memcpy(&cfg, cfg_flash, sizeof(ETX_GNRL_CFG_));

					/* Before writing the data, reset the available slot */
					cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 1u;

					/* write back the updated config */
					ret = write_cfg_to_flash(&cfg);
					if (ret != ETX_OTA_EX_OK) {
						break;
					}
				}

				/* write the chunk to the Flash (App location) */
				ex = write_data_to_slot(slot_num_to_write, buf + 4, data_len-4, is_first_block);
				printf ("Writing %d bytes to slot %d\r\n", data_len, slot_num_to_write);
				if (ex == HAL_OK) {
					printf("[%ld/%ld]\r\n", ota_fw_received_size / OTA_DATA_MAX_SIZE, ota_fw_total_size /OTA_DATA_MAX_SIZE);
					if (ota_fw_received_size >= ota_fw_total_size) {
						//received the full data. So, move to end
						ota_state = ETX_OTA_STATE_END;
					}
					ret = ETX_OTA_EX_OK;
				}
			}
		}
			break;

		case ETX_OTA_STATE_END: {

			ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*) buf;

			if (cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD) {
				if (cmd->cmd == ETX_OTA_CMD_END) {
					printf("Received OTA END Command\r\n");

					printf("Validating the received Binary...\r\n");

					uint32_t slot_addr;
					if (slot_num_to_write == 0u) {
						slot_addr = APP_SLOT0_FLASH_ADDR;
					} else {
						slot_addr = APP_SLOT1_FLASH_ADDR;
					}

					//Calculate and verify the CRC
					uint32_t cal_crc = crc32_bytes(&hcrc, (uint8_t*)slot_addr, ota_fw_total_size);
					if (cal_crc != ota_fw_crc) {
						printf("ERROR: FW CRC Mismatch\r\n");
						break;
					}
					printf("Done!!!\r\n");

					/* Read the configuration */
					ETX_GNRL_CFG_ cfg;
					memcpy(&cfg, cfg_flash, sizeof(ETX_GNRL_CFG_));

					//update the slot
					cfg.slot_table[slot_num_to_write].fw_crc = cal_crc;
					cfg.slot_table[slot_num_to_write].fw_size = ota_fw_total_size;
					cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 0u;
					cfg.slot_table[slot_num_to_write].should_we_run_this_fw = 1u;

					//reset other slots
					for (uint8_t i = 0; i < NO_OF_SLOTS; i++) {
						if (slot_num_to_write != i) {
							//update the slot as inactive
							cfg.slot_table[i].should_we_run_this_fw = 0u;
						}
					}

					//update the reboot reason
					cfg.reboot_cause = NORMAL_BOOT;

					/* write back the updated config */
					ret = write_cfg_to_flash(&cfg);
					if (ret == ETX_OTA_EX_OK) {
						ota_state = ETX_OTA_STATE_IDLE;
						ret = ETX_OTA_EX_OK;
					}
				}
			}
		}
			break;

		default: {
			/* Should not come here */
			ret = ETX_OTA_EX_ERR;
		}
			break;
		};
	} while ( false);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, OTA_PACKET_MAX_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	return ret;
}

/**
 * @brief Receive a one chunk of data.
 * @param buf buffer to store the received data
 * @param max_len maximum length to receive
 * @retval ETX_OTA_EX_
 */
uint16_t etx_receive_chunk(uint16_t size) {

	uint16_t bytesReceived = 0u;
	ETX_OTA_PACKET_TYPE_ packetType;

	packetType = (ETX_OTA_PACKET_TYPE_)rxBuffer[1];
	if (packetType > ETX_OTA_PACKET_TYPE_RESPONSE){
		puts ("Unknown packet type!\r\n");
		return 0;
	}
	if (size < 10){
		if (packetType != ETX_OTA_PACKET_TYPE_DATA){
			puts ("Wrong packet size!\r\n");
			return 0;
		}
	}

	bytesReceived = size;

	printf ("%d bytes received\r\n", bytesReceived);
	return bytesReceived;
}


/**
 * @brief Send the response.
 * @param type ACK or NACK
 * @retval none
 */
static void etx_ota_send_resp(uint8_t type) {

	ETX_OTA_RESP_ rsp = {

		.sof = OTA_SOF,
		.packet_type =ETX_OTA_PACKET_TYPE_RESPONSE,
		.data_len = 1u,
		.status = type,
		.eof = OTA_EOF
	};

	rsp.crc = crc32_bytes(&hcrc, (uint8_t*)&rsp+4,5);

	//send response
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&rsp, sizeof(ETX_OTA_RESP_));
}

/**
 * @brief Write data to the Slot
 * @param slot_num slot to be written
 * @param data data to be written
 * @param data_len data length
 * @is_first_block true - if this is first block, false - not first block
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef write_data_to_slot(uint8_t slot_num, uint8_t *data, uint16_t data_len, bool is_first_block) {

	HAL_StatusTypeDef ret;

	do {

		if (slot_num >= NO_OF_SLOTS) {
			ret = HAL_ERROR;
			break;
		}

		ret = HAL_FLASH_Unlock();
		if (ret != HAL_OK) {
			break;
		}

		//No need to erase every time. Erase only the first time.
		if (is_first_block) {
			printf("Erasing the Slot %d Flash memory...\r\n", slot_num);
			//Erase the Flash
			FLASH_EraseInitTypeDef EraseInitStruct;
			uint32_t SectorError;

			EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
			if (slot_num == 0) {
				EraseInitStruct.Sector = FLASH_SECTOR_5;
			} else {
				EraseInitStruct.Sector = FLASH_SECTOR_7;
			}
			EraseInitStruct.NbSectors = 2;                    //erase 2 sectors
			EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

			ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
			if (ret != HAL_OK) {
				printf("Flash Erase Error\r\n");
				break;
			}
		}

		uint32_t flash_addr;

		if (slot_num == 0) {
			flash_addr = APP_SLOT0_FLASH_ADDR;
		} else {
			flash_addr = APP_SLOT1_FLASH_ADDR;
		}

		for (int i = 0; i < data_len; i++) {
			ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE, (flash_addr + ota_fw_received_size), data[i]);
			if (ret == HAL_OK) {
				//update the data count
				ota_fw_received_size += 1;
			} else {
				printf("Flash Write Error\r\n");
				break;
			}
		}

		if (ret != HAL_OK) {
			break;
		}

		ret = HAL_FLASH_Lock();
		if (ret != HAL_OK) {
			break;
		}
	} while (false);

	return ret;
}

/**
 * @brief Return the available slot number
 * @param none
 * @retval slot number
 */
static uint8_t get_available_slot_number( void )
{
  uint8_t   slot_number = 0xFF;

  /* Read the configuration */
  ETX_GNRL_CFG_ cfg;
  memcpy( &cfg, cfg_flash, sizeof(ETX_GNRL_CFG_) );
  /*
   * Check the slot is valid or not. If it is valid,
   * then check the slot is active or not.
   *
   * If it is valid and not active, then use that slot.
   * If it is not valid, then use that slot.
   *
   */

   for( uint8_t i = 0; i < NO_OF_SLOTS; i++ )
   {
     if( ( cfg.slot_table[i].is_this_slot_not_valid != 0u ) || ( cfg.slot_table[i].is_this_slot_active == 0u ) )
     {
       slot_number = i;
       printf("Slot %d is available for OTA update\r\n", slot_number);
       break;
     }
   }

   return slot_number;
}

/**
 * @brief Write data to the Application's actual flash location.
 * @param data data to be written
 * @param data_len data length
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint32_t data_len) {
	HAL_StatusTypeDef ret;

	do {
		ret = HAL_FLASH_Unlock();
		if (ret != HAL_OK) {
			break;
		}

		//Check if the FLASH_FLAG_BSY.
		FLASH_WaitForLastOperation( HAL_MAX_DELAY);

		// clear all flags before you write it to flash
		__HAL_FLASH_CLEAR_FLAG(
				FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

		printf("Erasing the App Flash memory...\r\n");
		//Erase the Flash
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;

		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.Sector = FLASH_SECTOR_9;
		EraseInitStruct.NbSectors = 2;                    //erase 2 sectors(5,6)
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
		if (ret != HAL_OK) {
			printf("Flash erase Error\r\n");
			break;
		}

		for (uint32_t i = 0; i < data_len; i++) {
			ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE, (APP_FLASH_ADDR + i), data[i]);
			if (ret != HAL_OK) {
				printf("App Flash Write Error\r\n");
				break;
			}
		}

		if (ret != HAL_OK) {
			break;
		}

		ret = HAL_FLASH_Lock();
		if (ret != HAL_OK) {
			break;
		}

		//Check if the FLASH_FLAG_BSY.
		FLASH_WaitForLastOperation( HAL_MAX_DELAY);

	} while ( false);

	return ret;
}

/**
 * @brief Load the new app to the app's actual flash memory.
 * @param none
 * @retval none
 */
void load_new_app(void) {
	bool is_update_available = false;
	uint8_t slot_num;
	HAL_StatusTypeDef ret;

	/* Read the configuration */
	ETX_GNRL_CFG_ cfg;
	memcpy(&cfg, cfg_flash, sizeof(ETX_GNRL_CFG_));

	/*
	 * Check the slot whether it has a new application.
	 */

	for (uint8_t i = 0; i < NO_OF_SLOTS; i++) {
		if (cfg.slot_table[i].should_we_run_this_fw == 1u) {
			printf("New Application is available in the slot %d!!!\r\n", i);
			is_update_available = true;
			slot_num = i;

			//update the slot
			cfg.slot_table[i].is_this_slot_active = 1u;
			printf ("Slot %d active now\r\n", slot_num);
			cfg.slot_table[i].should_we_run_this_fw = 0u;

			break;
		}
	}

	if (is_update_available) {
		//make other slots inactive
		for (uint8_t i = 0; i < NO_OF_SLOTS; i++) {
			if (slot_num != i) {
				//update the slot as inactive
				cfg.slot_table[i].is_this_slot_active = 0u;
			}
		}

		uint32_t slot_addr;
		if (slot_num == 0u) {
			slot_addr = APP_SLOT0_FLASH_ADDR;
		} else {
			slot_addr = APP_SLOT1_FLASH_ADDR;
		}

		//Load the new app or firmware to app's flash address
		ret = write_data_to_flash_app((uint8_t*) slot_addr, cfg.slot_table[slot_num].fw_size);
		if (ret != HAL_OK) {
			printf("App Flash write Error\r\n");
		} else {
				/* write back the updated config */
			puts ("Write config to flash\r\n");
			ret = write_cfg_to_flash(&cfg);
			if (ret != HAL_OK) {
				printf("Config Flash write Error\r\n");
			}
		}
	} else {
		//Find the active slot in case the update is not available
		for (uint8_t i = 0; i < NO_OF_SLOTS; i++) {
			if (cfg.slot_table[i].is_this_slot_active == 1u) {
				slot_num = i;
				break;
			}
		}
	}

	//Verify the application is corrupted or not
	printf("Verifying the Application...");

	FLASH_WaitForLastOperation(HAL_MAX_DELAY);
	//Verify the application
	uint32_t cal_data_crc = crc32_bytes(&hcrc, (uint8_t*)APP_FLASH_ADDR, cfg.slot_table[slot_num].fw_size);
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	//Verify the CRC
	if (cal_data_crc != cfg.slot_table[slot_num].fw_crc) {
		printf("ERROR!!!\r\n");
		printf("Invalid Application. HALT!!!\r\n");
		while (1)
			;
	}
	printf("Done!!!\r\n");
}

/**
 * @brief Write the configuration to flash
 * @param cfg config structure
 * @retval none
 */
static HAL_StatusTypeDef write_cfg_to_flash(ETX_GNRL_CFG_ *cfg) {
	HAL_StatusTypeDef ret;

	do {
		if (cfg == NULL) {
			ret = HAL_ERROR;
			break;
		}

		ret = HAL_FLASH_Unlock();
		if (ret != HAL_OK) {
			break;
		}

		//Check if the FLASH_FLAG_BSY.
		FLASH_WaitForLastOperation( HAL_MAX_DELAY);

		//Erase the Flash
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;

		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.Sector = FLASH_SECTOR_3;
		EraseInitStruct.NbSectors = 1;                    //erase only sector 4
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		// clear all flags before you write it to flash
		__HAL_FLASH_CLEAR_FLAG(
				FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

		ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
		if (ret != HAL_OK) {
			break;
		}

		//write the configuration
		uint8_t *data = (uint8_t*) cfg;
		for (uint32_t i = 0u; i < sizeof(ETX_GNRL_CFG_); i++) {
			ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE, CONFIG_FLASH_ADDR + i, data[i]);
			if (ret != HAL_OK) {
				printf("Slot table Flash Write Error\r\n");
				break;
			}
		}

		//Check if the FLASH_FLAG_BSY.
		FLASH_WaitForLastOperation( HAL_MAX_DELAY);

		if (ret != HAL_OK) {
			break;
		}

		ret = HAL_FLASH_Lock();
		if (ret != HAL_OK) {
			break;
		}
	} while ( false);

	return ret;
}
