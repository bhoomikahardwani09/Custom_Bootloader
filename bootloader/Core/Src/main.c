/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#define BL_DEBUG_MSG_EN
//prints formatted srings to console over UART
void printm(char* format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
//extract the argumnet list using VA apis
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char data[] = "Hello from bootloader..!!!\r\n";
#define bl_rcv_len  200
uint8_t bl_rcv_buffer[bl_rcv_len];

uint8_t supported_cmd[] = {
		BL_VERSION,
		BL_HELP,
		BL_CID,
		BL_RDP_STATUS,
		BL_GO_TO_ADDR,
		BL_FLASH_ERASE,
		BL_MEM_WR,
		BL_READ_SECTOR_STATUS,
		BL_EN_RW_PROTECT,
		BL_DI_RW_PROTECT,
		BL_READ_OTP};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
	  {
		  printm("BL_Debug msg : Button is pressed..going to BL mode\r\n");
		  bootloader_uart_read_data();
	  }
	  else
	  {
		  printm("BL_Debug msg : Button is not pressed..executing user application\r\n");
		  bootloader_jump_to_userapp();
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

//Code to jump to user application which is stored in FLASH_SEC2_BASE_ADDR//
void bootloader_jump_to_userapp(void)
{
	//function pointer to hold the address of the reset handler of the user app.//
	void (*app_reset_handler)(void);
	printm("BL_Debug msg : bootloader_jump_to_userappur\n");

	//(I)configure the MSP by reading the value from the base address of sector 2
	uint32_t msp_val = *(volatile uint32_t*)FLASH_SEC2_BASE_ADDR;
	printm("BL_Debug msg : %x\n", msp_val);

	__set_MSP(msp_val); //CMSIS function

	//(II) fetch the address of reset handler of the user application//
	uint32_t resethandler_addr = *(volatile uint32_t*)(FLASH_SEC2_BASE_ADDR + 4);
	app_reset_handler = (void*)resethandler_addr;
	printm("BL_Debug msg : Application reset handler address = %#x\n", app_reset_handler);

	//(III) Jump to reset handler of the user application
	app_reset_handler();
}

void bootloader_uart_read_data(void)
{
	uint8_t rx_len = 0;
	while(1)
	{
		memset(bl_rcv_buffer, 0, 200);
		HAL_UART_Receive(&huart2, bl_rcv_buffer, 1, HAL_MAX_DELAY);
		rx_len = bl_rcv_buffer[0];
		HAL_UART_Receive(&huart2, &bl_rcv_buffer[1], rx_len, HAL_MAX_DELAY);
		printm("Length byte: %d\r\n", bl_rcv_buffer[0]);
		printm("Command byte: 0x%x\r\n", bl_rcv_buffer[1]);


		switch(bl_rcv_buffer[1])
		{
			case BL_VERSION:
				bl_handler_getver_cmd(bl_rcv_buffer);
				break;

			case BL_HELP:
				bl_handle_gethelp_cmd(bl_rcv_buffer);
				break;

			case BL_CID:
				bl_handle_getcid_cmd(bl_rcv_buffer);
				break;

			case BL_RDP_STATUS:
				bl_handle_getrdp_cmd(bl_rcv_buffer);
				break;

			case BL_GO_TO_ADDR:
				bl_handle_jumptoaddr_cmd(bl_rcv_buffer);
				break;

			case BL_FLASH_ERASE:
				bl_handle_flash_erase_cmd(bl_rcv_buffer);
				break;

			case BL_MEM_WR:
				bl_handle_mem_wr_cmd(bl_rcv_buffer);
				break;

			case BL_READ_SECTOR_STATUS:
				bl_handle_read_sector_status(bl_rcv_buffer);
				break;

			case BL_EN_RW_PROTECT:
				bl_handle_en_rw_protect_cmd(bl_rcv_buffer);
				break;

			case BL_DI_RW_PROTECT:
				bl_handle_dis_rw_protect_cmd(bl_rcv_buffer);
				break;

			case BL_READ_OTP:
				bl_handle_read_OTP_cmd(bl_rcv_buffer);
				break;

			default:
				printm("Invalid command code from the host\n");
				break;
		}
	}


}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//////Implementation of command handle functions//////

//1. To handle to get version command of the bootloader//
void bl_handler_getver_cmd(uint8_t *bl_rcv_buffer)
{
	uint8_t bl_ver;

	//1. check the CRC
	printm("Bootloader command : bl_handler_getver_cmd\n");
	//total length of cmad packet
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*)(bl_rcv_buffer + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success!!\n");
		bootloader_send_ACK(bl_rcv_buffer[0], 1);
		bl_ver = get_bl_version();
		printm("BL_VER : %d %#x\n", bl_ver, bl_ver);
		bootloader_uart_write_data((uint8_t*)&bl_ver, 1);
	}
	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}

}

//2. Bootloader sends all the supported command codes//
void bl_handle_gethelp_cmd(uint8_t *pbuffer)
{
	printm("Bootloader command : bl_handle_gethelp_cmd\n");

	//total length of command packet
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*)(bl_rcv_buffer + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success!!\n");
		bootloader_send_ACK(bl_rcv_buffer[0], sizeof(supported_cmd));
		bootloader_uart_write_data(supported_cmd, sizeof(supported_cmd));
	}
	else
	{
		printm("Checksum failed..!!\n");
		bootloader_send_NACK();
	}
}

//3. To get the chip identification no. of the MCU//
void bl_handle_getcid_cmd(uint8_t *pbuffer)
{
	uint16_t bl_cid_num = 0;
	printm("Bootloader command : bl_handle_getcid_cmd\n");

	//total length of command packet
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract the CRC32 sent by the host//
	uint32_t crc_host = *( (uint32_t*)(&bl_rcv_buffer[0] + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success!!\n");
		bootloader_send_ACK(pbuffer[0], 2);
		bl_cid_num = get_cid_num();
		printm("MCU ID : %d %#x\n", bl_cid_num, bl_cid_num);
		bootloader_uart_write_data((uint8_t*)&bl_cid_num, 2);
	}
	else
	{
		printm("Checksum failed..!!\n");
		bootloader_send_NACK();
	}
}

//4. To handle the RDP level and read it//
void bl_handle_getrdp_cmd(uint8_t *pbuffer)
{
	uint8_t rdp_level = 0x00;
	printm("Bootloader command : bl_handle_getrdp_cmd\n");

	//Total length of command packet
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract the CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*)(&bl_rcv_buffer[0] + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success!!\n");
		bootloader_send_ACK(pbuffer[0], 2);
		rdp_level = get_flash_RDP_level();
		printm("MCU ID : %d %#x\n", rdp_level, rdp_level);
		bootloader_uart_write_data(&rdp_level, 1);
	}
	else
	{
		printm("Checksum failed..!!\n");
		bootloader_send_NACK();
	}
}

//5. This function jumps to the given address//
void bl_handle_jumptoaddr_cmd(uint8_t* pbuffer)
{
	uint32_t addr        = 0;
	uint8_t valid_addr   = ADDR_VALID;
	uint8_t invalid_addr = ADDR_INVALID;

	printm("Bootloader command : bl_handle_jumptoaddr_cmd\n");

	//Total length of command packet
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract the CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*)(&bl_rcv_buffer[0] + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len - 4, crc_host))
	{
		printm("Checksum success!!");
		bootloader_send_ACK(pbuffer[0], 1);

		//extract the address to jump
		addr = *((uint32_t*)&pbuffer[2]);
		printm("Jump to address : %#x\n", addr);

		if(verify_addr(addr) == ADDR_VALID)
		{
			bootloader_uart_write_data(&valid_addr, 1);

			addr += 1;  //making the T bit as 1, otherwise it will result in hardfault exception

			void (*jump_to_addr)(void) = (void*)addr;

			printm("Jumping to the given address...\n");
			jump_to_addr();
		}

		else
		{
			printm("Given address is invalid!\n");
			bootloader_uart_write_data(&invalid_addr, 1);
		}
	}

	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}
}

//6. This command is used to mass erase the flash sectors//
void bl_handle_flash_erase_cmd(uint8_t* pbuffer)
{
	uint8_t erase_status = 0x00;
	printm("Bootloader command : bl_handle_flash_erase_cmd\n");

	//Total length of command packet
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract the CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*)(&bl_rcv_buffer[0] + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len - 4, crc_host))
	{
		printm("Checksum success!!");
		bootloader_send_ACK(pbuffer[0], 1);
		printm("Initial sector : %d Number of sectors : %d\n", pbuffer[2], pbuffer[3]);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
		erase_status = execute_flash_erase(pbuffer[2], pbuffer[3]);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

		printm("Flash erase status : %#x\n", erase_status);
		bootloader_uart_write_data(&erase_status, 1);
	}
	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}

}

//7. This command handles to write in the flash region of the mcu//
void bl_handle_mem_wr_cmd(uint8_t *pbuffer)
{
	uint8_t val_addr = ADDR_VALID, status_wr = 0x00, len = pbuffer[0], checksum = 0;
	uint8_t payload_len = pbuffer[6];
	uint32_t mem_addr = *( (uint32_t*) (&pbuffer[2]) );
	checksum = pbuffer[len];
	printm("Bootloader command : bl_handle_mem_wr_cmd\n");

	//total length
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract the CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*)(&bl_rcv_buffer[0] + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len - 4, crc_host))
	{
		printm("Checksum success!!");
		bootloader_send_ACK(pbuffer[0], 1);
		printm("Memory write address : %#x\n", mem_addr);

		if(verify_addr(mem_addr) == ADDR_VALID)
		{
			printm("Write address is valid\n");
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
			status_wr = execute_mem_wr(&pbuffer[7], mem_addr, payload_len);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

			bootloader_uart_write_data(&status_wr, 1);
		}
		else
		{
			printm("Write address is invalid\n");
			status_wr = ADDR_INVALID;
			bootloader_uart_write_data(&status_wr, 1);
		}
	}

	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}

}

//8. This command enables the read/write protection
void bl_handle_en_rw_protect_cmd(uint8_t *pbuffer)
{
	uint8_t status = 0x00;
	printm("Bootloader command : bl_handle_en_rw_protect_cmd\n");

	//total length
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//extract the CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*)(&bl_rcv_buffer[0] + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success..!!\n");
		bootloader_send_ACK(pbuffer[0], 1);

		status = config_flash_rw_protect(pbuffer[2], pbuffer[3], 0);

		printm("Flash erase status : %#x\n", status);
		bootloader_uart_write_data(&status, 1);
	}
	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}
}

//9. This command disables the read/write protection
void bl_handle_dis_rw_protect_cmd(uint8_t *pbuffer)
{
	uint8_t status = 0x00;
	printm("Bootloader command : bl_handle_dis_rw_protect_cmd\n");

	//Total length
	uint32_t cmd_packet_len = pbuffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t crc_host = *((uint32_t*) (bl_rcv_buffer + cmd_packet_len - 4));

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success..!!\n");
		bootloader_send_ACK(pbuffer[0], 1);

		status = config_flash_rw_protect(0, 0, 1);

		printm("Flash erase status : %#x", status);
		bootloader_uart_write_data(&status, 1);
	}
	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}
}

//10. This command reads the status of all sectors
void bl_handle_read_sector_status(uint8_t *pbuffer)
{
	uint8_t status = 0x00;
	printm("Bootloader command : bl_handle_read_sector_status\n");

	//Total length
	uint32_t cmd_packet_len = bl_rcv_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*) (bl_rcv_buffer + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success..!!\n");
		bootloader_send_ACK(pbuffer[0], 2);
		status = read_OB_rw_protect_status();
		printm("nWRP Status : %#x\n", status);
		bootloader_uart_write_data((uint8_t*)&status, 2);
	}
	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}
}

//11. This command handles to read the OTP data blocks
void bl_handle_read_OTP_cmd(uint8_t *pbuffer)
{
	uint8_t status = 0x00;
	printm("Bootloader command : bl_handle_read_OTP_cmd\n");

	//Total length
	uint32_t cmd_packet_len = pbuffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t crc_host = *( (uint32_t*) (bl_rcv_buffer + cmd_packet_len - 4) );

	if(!bootloader_verify_CRC(&bl_rcv_buffer[0], cmd_packet_len, crc_host))
	{
		printm("Checksum success..!!\n");
		bootloader_send_ACK(pbuffer[0], 1);
		status = read_otp_block(pbuffer[2]);
		printm("OTP block data: %d", status);
		bootloader_uart_write_data(&status, 1);
	}
	else
	{
		printm("Checksum failed!!\n");
		bootloader_send_NACK();
	}
}
//This function sends ACK if CRC matches with the length//
void bootloader_send_ACK(uint8_t cmd_code, uint8_t follow_len)
{
	//We send 2 bytes here : (I) ACK (II) length to follow
	uint8_t ack_buffer[2];
	ack_buffer[0] = BL_ACK;
	ack_buffer[1] = follow_len;
	HAL_UART_Transmit(&huart2, ack_buffer, 2, HAL_MAX_DELAY);
	printm("ACK Done\n");
}

//This function sends NACK if CRC doesn't match with the length//
void bootloader_send_NACK(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(&huart2, &nack, 1, HAL_MAX_DELAY);
}

//This function writes data in UART2//
void bootloader_uart_write_data(uint8_t *pbuffer, uint32_t len)
{
	HAL_UART_Transmit(&huart2, pbuffer, len, HAL_MAX_DELAY);
}
//This function verify the CRC of the given buffer in pData
//pData : pointer to the data
//len : length for which CRC is to be calculated
//crc_host : 32 bit CRC value received by the host that is to be compared
uint8_t bootloader_verify_CRC(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t crc_val = 0xff;
	for(uint32_t i = 0; i < len; i++)
	{
		uint32_t idata = pData[i];
		crc_val = HAL_CRC_Accumulate(&hcrc, &idata, 1);

	}

	//Reset CRC calculation unit
	__HAL_CRC_DR_RESET(&hcrc);
	if(crc_val == crc_host)
	{
		return CRC_SUCCESS;
	}
	else return CRC_FAILURE;
}

//This function returns the bl_version macro//
uint8_t get_bl_version(void)
{
	return BL_VERSION;
}

//This function reads the chip identifier number//
uint16_t get_cid_num(void)
{
	uint16_t id;
	id = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return id;
}

//This function gets the RDP level from the flash memory//
uint8_t get_flash_RDP_level(void)
{
	uint8_t rdp_status = 0;

#if 0
	FLASH_OBProgramInitTypeDef ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t) ob_handle.RDPLevel;
#else
	volatile uint32_t *pOptionBytes = (uint32_t*) 0x1FFFC000;
	rdp_status = (uint8_t) (*pOptionBytes >> 8);  //we only want bits from 8 to 15
#endif
	return rdp_status;
}


//This function verifies the given address is valid or not
uint8_t verify_addr(uint32_t go_addr)
{
	if(go_addr >= SRAM_BASE && go_addr <= SRAM_END) return ADDR_VALID;
	else if(go_addr >= FLASH_BASE && go_addr <= FLASH_END) return ADDR_VALID;
	else if(go_addr >= BKPSRAM_BB_BASE && go_addr <= BKPSRAM_END) return ADDR_VALID;
	else return ADDR_INVALID;
}

void jump_to_addr(uint32_t go_addr)
{
	//function pointer to hold the address of the reset handler of the go_addr//
	void (*app_reset_handler)(void);
	uint32_t msp_val = *(volatile uint32_t*)go_addr;

	__set_MSP(msp_val);

	uint32_t resethandler_addr = *(volatile uint32_t*)(go_addr + 4);
	app_reset_handler = (void*)resethandler_addr;

	app_reset_handler();

}

uint8_t execute_flash_erase(uint8_t sector_num, uint8_t units)
{
	FLASH_EraseInitTypeDef flasherase_handle;
	uint32_t error;
	HAL_StatusTypeDef status;

	if(sector_num > 8) return SECTOR_INVALID;
	if(sector_num < 8 || sector_num == 0xff)
	{
		if(sector_num == (uint8_t) 0xff) flasherase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;

		else
		{
			if(units > 8 - sector_num) units = sector_num;

			flasherase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flasherase_handle.Sector    = sector_num;
			flasherase_handle.NbSectors = units;
		}
		flasherase_handle.Banks = FLASH_BANK_1;

		//Access to the flash registers//
		HAL_FLASH_Unlock();
		flasherase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t)HAL_FLASHEx_Erase(&flasherase_handle, &error);
		HAL_FLASH_Lock();

		return status;
	}
	return SECTOR_INVALID;
}

//This function writes from the given base address into the flash region byte by byte//
uint8_t execute_mem_wr(uint8_t *pbuffer, uint32_t mem_addr, uint32_t len)
{
	uint8_t status = HAL_OK;
	HAL_FLASH_Unlock();

	for(uint32_t i = 0; i < len; i++)
	{
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_addr + i, pbuffer[i]);
	}

	HAL_FLASH_Lock();
	return status;
}

//This function configures the read/write protection in the flash sectors
//Modifying user option bytes
//To modify the user option value, follow the sequence below:
//1.Check that no Flash memory operation is ongoing by checking the BSY bit in the
//FLASH_SR register
//2.Write the desired option value in the FLASH_OPTCR register.
//3.Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
//4.Wait for the BSY bit to be cleared.
//@param sector details : to configure which sector
//@param protection_mode : 1 = write protect
//                         2 : read and write protect
//@param DI : disable
uint8_t config_flash_rw_protect(uint8_t sector_details, uint8_t protection_mode, uint8_t DI)
{
//	modify the address 0x1FFF C008 bit 15(SPRMOD)
	volatile uint32_t *pOTPCR = &FLASH->OPTCR;

	if(DI)
	{
		// disable all the r/w protection on sectors
		HAL_FLASH_OB_Unlock();

		//clear the bit 31 (default state)
		*pOTPCR &= ~(1 << 31);

		//clear the protection by making all the bits of sector as 1
		*pOTPCR |= (0xFF << 16);

       //setting operation start bit
		*pOTPCR |= (1 << 1);

		//wait till there is no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET );

		HAL_FLASH_OB_Lock();

		return 0;
	}

	else if(protection_mode == (uint8_t) 1)
	{
		//write protection to sectors mentioned in sector details//

		//Unlock configuration for option bytes
		HAL_FLASH_OB_Unlock();

		//wait till there is no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET );

//      For bit 31 of the register
//		nWRPi [7:0] = from bit 16 to 23
//		SPRMOD: Selection of Protection Mode of nWPRi bits
//		0: nWPRi bits used for sector i write protection (Default)
//		1: nWPRi bits used for sector i PCROP protection (Sector)

		*pOTPCR &= ~(1 << 31);
		*pOTPCR &= ~(sector_details << 16);

//		setting operation start bit
		*pOTPCR |= (1 << 1);

		//wait till there is no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET );

		HAL_FLASH_OB_Lock();
	}

	else if(protection_mode == (uint8_t) 2)
	{
		//read and write protection to the sectors mentioned
		//Unlock configuration for option bytes
		HAL_FLASH_OB_Unlock();

		//wait till there is no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET );

		//set the bit 31 in the FLASH_OPTCR
		*pOTPCR |= (1 << 31);

		//rw protect on sectors mentioned
		*pOTPCR &= ~(0xff < 16);
		*pOTPCR |= (sector_details << 16);

       //setting operation start bit
		*pOTPCR |= (1 << 1);

		//wait till there is no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET );

		HAL_FLASH_OB_Lock();

	}

	return 0;

}

uint16_t read_OB_rw_protect_status(void)
{
	FLASH_OBProgramInitTypeDef ob_init;
	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&ob_init);
	HAL_FLASH_OB_Lock();
	return (uint16_t)ob_init.WRPSector;
}

uint8_t read_otp_block(uint16_t block_num)
{
	uint32_t otp_data[32] = {0};
	if(block_num < 0 || block_num > 15)
	{
		printm("Invalid OTP block number!\n");
		return 0;
	}
	uint32_t block_offset = block_num * 32;
	uint32_t block_addr = FLASH_OTP_BASE + block_offset;

	for(uint8_t i = 0; i <= 31; i++)
	{
		otp_data[i] = *( (uint8_t*)(block_addr + i) );
	}

	printm("OTP block %d Data :\n", block_num);
	for(uint8_t i = 0; i <= 31; i++)
	{
		printm("%02X ", otp_data[i]);
	}
	printm("\n");

	return otp_data[0];
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
