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
		BL_READ_SECTOR_STATUS};
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
//This function writes data in UART2//
void bootloader_uart_write_data(uint8_t *pbuffer, uint32_t len)
{
	HAL_UART_Transmit(&huart2, pbuffer, len, HAL_MAX_DELAY);
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
