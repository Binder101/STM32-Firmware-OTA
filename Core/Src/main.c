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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include<stdbool.h>
#include<stdint.h>
#include<stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 128
#define ACK_BYTE 0x06
#define NACK_BYTE 0x15
#define ERROR_BYTE_1 0x20
#define ERROR_BYTE_2 0x21
#define HEADER_CHECK 0x22
#define BAD_HEADER_CHECK 0x23
#define SHORT_PACKET_BYTE 0x24
#define LONG_PACKET_BYTE 0x25
#define SEQUENCE_ERROR 0x90
#define RESPONSE_LENGTH 8
#define MAX_PAYLOAD_SIZE 71
/* USER CODE END PD */

/* USER CODE BEGIN PV */
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t ackByte = ACK_BYTE;
uint8_t nackByte = NACK_BYTE;
uint8_t header_good_byte = HEADER_CHECK;
uint8_t header_bad_byte = BAD_HEADER_CHECK;
uint8_t errorByte1 = ERROR_BYTE_1;
uint8_t errorByte2 = ERROR_BYTE_2;
uint8_t shortPacketByte = SHORT_PACKET_BYTE;
uint8_t longPacketByte = LONG_PACKET_BYTE;
uint8_t sequenceErrorByte = SEQUENCE_ERROR;
uint8_t response[RESPONSE_LENGTH];
uint8_t payload[MAX_PAYLOAD_SIZE];

int lengthData;
int sequence;
int sequence1;
int sequence2;
int nextSequence = 0;
int responseLength = 0;


volatile uint16_t oldPos = 0;
volatile uint16_t newPos = 0;
volatile uint8_t dataReady = 0;
volatile uint16_t dataLength = 0;

uint8_t processedData[RX_BUFFER_SIZE];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
void UART_DMA_Init(void)
{
    HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE);
}

void UartRx_Circular_Reset(void) {
    __disable_irq();
    oldPos = 0;
    newPos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    dataReady = 0;
    dataLength = 0;
    memset(rxBuffer, 0, RX_BUFFER_SIZE);
    __enable_irq();
}



static uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t len) {
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; ++i) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc;
}

bool verify_crc32_payload_crc(const uint8_t *packet, size_t packet_len) {
    if (packet_len < 4) return false;
    size_t payload_len = packet_len - 4;
    uint32_t crc = 0xFFFFFFFFu;
    crc = crc32_update(crc, packet, payload_len);
    crc ^= 0xFFFFFFFFu;

    uint32_t rx = (uint32_t)packet[payload_len]
                | ((uint32_t)packet[payload_len+1] << 8)
                | ((uint32_t)packet[payload_len+2] << 16)
                | ((uint32_t)packet[payload_len+3] << 24);

    return crc == rx;
}

void appendByte(uint8_t value){
	if(responseLength < RESPONSE_LENGTH){
		response[responseLength] = value;
		responseLength++;
	}
	else{
		char *msg = "Response length full";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	}
}

void processDataHeaders(void){
	if(processedData[0] == 0xAA && processedData[1] == 0x55){
		// HAL_UART_Transmit(&huart2, &header_good_byte, 1, 100);
		appendByte(0x11);
	} else {
		// HAL_UART_Transmit(&huart2, &header_bad_byte, 1, 100);
		appendByte(0x01);
	}
}

void processDataSequence(void){
	unsigned int tens = processedData[2];
	unsigned int ones = processedData[3];
	int sequenceNo = 0;
	if(ones == 0){
		sequenceNo = tens;
	} else {
		sequenceNo = tens + ones;
	}
	sequence1 = tens;
	sequence2 = ones;

	sequence = sequenceNo;
	if(sequence == nextSequence){
		nextSequence = sequenceNo + 1;
		appendByte(0x11);
	}
	else {
		appendByte(0x01);
	}
	// HAL_UART_Transmit(&huart2, &sequenceNo, 1, 100);
}

void processDataFooters(void){
	uint8_t footer_byte_1 = processedData[73];
	uint8_t footer_byte_2 = processedData[74];
	if(footer_byte_1 == 0x0D && footer_byte_2 == 0x0A){
		appendByte(0x11);
	} else {
		appendByte(0x01);
	}
}

void processLastFlag(void) {
	uint8_t last_flag_byte = processedData[75];

	int initLen = 2;
	int endLen = 3;
	// size_t n = sizeof(processedData)/sizeof(processedData[0]);

	if(last_flag_byte == 0x01){
		memcpy(payload, processedData + initLen, lengthData - endLen - initLen);
		appendByte(0x11);
		if(verify_crc32_payload_crc(payload, lengthData-endLen-initLen)){
			appendByte(0x12);
		}
		else{
			appendByte(0x14); // Error in the crc-checksum
		}
	} else if (last_flag_byte == 0x00) {
		memcpy(payload, processedData + initLen, lengthData - endLen - initLen);
		appendByte(0x11);
		if(verify_crc32_payload_crc(payload, lengthData-endLen-initLen)){
			appendByte(0x13);
		} else {
			appendByte(0x14); // Error in the crc-checksum
		}

	} else {
		appendByte(0x01);

	}
}

void processDataChunkLength(void){
	uint8_t chunkLength = processedData[4];
	if(chunkLength == 0x40) {
		appendByte(0x11);
	} else {
		appendByte(0x01);
	}
}


uint8_t ProcessReceivedData(uint8_t* data, uint16_t length)
{
	lengthData = length;
    if (length == 0 || length >= RX_BUFFER_SIZE) {
    	UartRx_Circular_Reset();
    	return 0;
    }
    else if (length == 76){
		memcpy(processedData, data, length);
		processDataHeaders();
		processDataSequence();
		processDataChunkLength();
		processDataFooters();
		processLastFlag();
		UartRx_Circular_Reset();
		return 1;
    }
    else if (length < 76){
    	UartRx_Circular_Reset();
    	return 2;
    }
    else if (length > 76){
    	UartRx_Circular_Reset();
    	return 3;
    }
    else {
    	UartRx_Circular_Reset();
    	return 4;
    }
}


void SendAcknowledgment(uint8_t isValid)
{
    if (isValid == 0) {
        HAL_UART_Transmit(&huart2, &nackByte, 1, 100);
    } else if (isValid == 1){
        HAL_UART_Transmit(&huart2, &ackByte, 1, 100);
    } else if (isValid == 2){
    	HAL_UART_Transmit(&huart2, &shortPacketByte, 1, 100);
    } else if (isValid == 3){
    	HAL_UART_Transmit(&huart2, &longPacketByte, 1, 100);
    } else if (isValid == 4){
    	HAL_UART_Transmit(&huart2, &errorByte2, 1, 100);
    }
}


void CheckForNewData(void)
{
    newPos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    if(newPos != oldPos)
    {
        if(newPos > oldPos)
        {
            dataLength = newPos - oldPos;
            uint8_t isValid = ProcessReceivedData(&rxBuffer[oldPos], dataLength);
            SendAcknowledgment(isValid);
            HAL_UART_Transmit(&huart2, response, sizeof(response)/sizeof(response[0]), 100);
            // HAL_UART_Transmit(&huart2, payload, 71, 100);
            responseLength = 0;

        }
        else
        {
            uint16_t firstPart = RX_BUFFER_SIZE - oldPos;
            uint16_t secondPart = newPos;
            memcpy(processedData, &rxBuffer[oldPos], firstPart);
            memcpy(&processedData[firstPart], &rxBuffer[0], secondPart);

            dataLength = firstPart + secondPart;

            uint8_t isValid = ProcessReceivedData(processedData, dataLength);
            SendAcknowledgment(isValid);
            HAL_UART_Transmit(&huart2, response, sizeof(response)/sizeof(response[0]), 100);
            // HAL_UART_Transmit(&huart2, payload, 71, 100);
            responseLength = 0;
        }

        oldPos = newPos;
        dataReady = 1;
    }
}
/* USER CODE END 0 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  UART_DMA_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  CheckForNewData();
	  if(dataReady)
	  {
		  dataReady = 0;
		  printf("Received %d bytes of data\r\n", dataLength);
		  for(int i = 0; i < dataLength; i++)
		  {
			  printf("%02X ", processedData[i]);
		  }
		  printf("\r\n");
	  }

	  HAL_Delay(10);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(300);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // Handle UART errors
        printf("UART Error occurred\r\n");

        // Restart DMA reception
        HAL_UART_DMAStop(&huart2);
        HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE);

        // Send NACK to indicate error
        SendAcknowledgment(0);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // Full buffer filled - can be used for additional processing
        // In circular mode, this indicates second half of buffer is full
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // Half buffer filled - can be used for additional processing
        // In circular mode, this indicates first half of buffer is full
    }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
