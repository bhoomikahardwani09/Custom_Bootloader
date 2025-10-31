/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#define FLASH_SEC2_BASE_ADDR     0x08008000U

//BOOTLOADER COMMAND//
#define BL_VERSION               0x11
#define	BL_HELP                  0x02
#define BL_CID                   0x03
#define	BL_RDP_STATUS            0x04
#define	BL_GO_TO_ADDR            0x05
#define	BL_FLASH_ERASE           0x06
#define	BL_MEM_WR                0x07
#define	BL_READ_SECTOR_STATUS    0x08
#define BL_EN_RW_PROTECT         0x09
#define BL_DI_RW_PROTECT         0x0A
#define BL_READ_OTP              0x0B

#define BL_ACK                   0xAE
#define BL_NACK                  0x7F

#define CRC_SUCCESS     0
#define CRC_FAILURE     1

#define ADDR_VALID      0x00
#define ADDR_INVALID    0x01

#define SECTOR_VALID    0x00
#define SECTOR_INVALID  0x01
//Start and end addresses of memories//
#define SRAM_SIZE     128*1024
#define SRAM_END      (SRAM_BASE + SRAM_SIZE)
#define FLASH_SIZE    512*1024
#define BKPSRAM_SIZE  4*1024
#define BKPSRAM_END   (BKPSRAM_BB_BASE + BKPSRAM_SIZE)
//BL function prototype
void bootloader_uart_read_data();
void bootloader_jump_to_userapp();

void bootloader_send_ACK(uint8_t cmd_code, uint8_t follow_len);
void bootloader_send_NACK(void);

uint8_t bootloader_verify_CRC(uint8_t *pData, uint32_t len, uint32_t crc_host);
void bootloader_uart_write_data(uint8_t *pbuffer, uint32_t len);

uint8_t get_bl_version(void);
void bl_handler_getver_cmd(uint8_t *bl_rcv_buffer);

void bl_handle_gethelp_cmd(uint8_t *pbuffer);

uint16_t get_cid_num(void);
void bl_handle_getcid_cmd(uint8_t *pbuffer);

uint8_t get_flash_RDP_level(void);
void bl_handle_getrdp_cmd(uint8_t *pbuffer);

uint8_t verify_addr(uint32_t go_addr);
void jump_to_addr(uint32_t go_addr);
void bl_handle_jumptoaddr_cmd(uint8_t* pbuffer);

uint8_t execute_flash_erase(uint8_t sector_num, uint8_t units);
void bl_handle_flash_erase_cmd(uint8_t* pbuffer);

uint8_t execute_mem_wr(uint8_t *pbuffer, uint32_t mem_addr, uint32_t len);
void bl_handle_mem_wr_cmd(uint8_t *pbuffer);

void bl_handle_en_rw_protect_cmd(uint8_t *pbuffer);

void bl_handle_dis_rw_protect_cmd(uint8_t *pbuffer);

void bl_handle_read_sector_status(uint8_t *pbuffer);
uint16_t read_OB_rw_protect_status(void);

uint8_t config_flash_rw_protect(uint8_t sector_details, uint8_t protection_mode, uint8_t DI);

uint8_t read_otp_block(uint16_t block_num);
void bl_handle_read_OTP_cmd(uint8_t *pbuffer);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
