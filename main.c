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
#include "cmsis_os.h"
#include "i2c-lcd.h"
#include "stdio.h"
#include "string.h"

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9
#define LED_PIN GPIO_PIN_8
#define PUMP_PIN GPIO_PIN_12
#define UART_RX_BUFFER_SIZE 64
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Biến toàn cục cho DHT11 */
uint8_t RHI, TCI, SUM;  // Độ ẩm nguyên (RHI), nhiệt độ nguyên (TCI), checksum
char temp[15];
char humi[15];

/* Biến toàn cục cho độ ẩm đất */
uint16_t soilMoisture = 0;
float am, phantram;

/* Biến toàn cục cho UART */
uint8_t rx_buffer[100];
uint8_t rx_index = 0;
uint8_t rx_data[1];
uint8_t rx_complete = 0;
uint8_t pump_status = 0;
uint8_t led_status = 0;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* FreeRTOS thread handles */
osThreadId DHT11TaskHandle;
osThreadId SensorTaskHandle;
osThreadId LCDTaskHandle;

osThreadId CommunicationTaskHandle;
osMessageQId UARTRxQueueHandle;
/* USER CODE END PTD */

/* Biến UART */
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
uint8_t uartRxData;
volatile uint8_t ledState = 0;   // Trạng thái LED
volatile uint8_t pumpState = 0;  // Trạng thái PUMP
volatile uint32_t uartRxIndex = 0;
volatile uint8_t uartRxComplete = 0;
char uartTxBuffer[128];
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
//I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void DHT11Task(void const *argument);
void SensorTask(void const *argument);
void LCDTask(void const *argument);
void ProcessCommand(char *command);
void microDelay(uint16_t delay);
uint8_t DHT11_Start(void);
uint8_t DHT11_Read(void);
void CommunicationTask(void const *argument);

/* USER CODE END PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    // Store received character
    if(uartRxIndex < UART_RX_BUFFER_SIZE - 1)
    {
      uartRxBuffer[uartRxIndex++] = uartRxData;

      // Check for end of command (\r\n or \n)
      if(uartRxData == '\n' || uartRxData == '\r')
      {
        uartRxBuffer[uartRxIndex] = '\0';  // Null terminate
        uartRxComplete = 1;  // Set flag for processing

        // Send to queue for processing in CommunicationTask
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        osMessagePut(UARTRxQueueHandle, (uint32_t)uartRxBuffer, 0);

        // Reset index for next message
        uartRxIndex = 0;
      }
    }
    else
    {
      // Buffer overflow protection
      uartRxIndex = 0;
    }

    // Restart UART receive interrupt
    HAL_UART_Receive_IT(&huart1, &uartRxData, 1);
  }
}

void ProcessCommand(char *command)
{
  // Example command: "PUMP:ON,LED:OFF"
  if(strstr(command, "PUMP:ON") != NULL)
  {
    pumpState = 1;
    HAL_GPIO_WritePin(GPIOA, PUMP_PIN, GPIO_PIN_SET);
  }
  else if(strstr(command, "PUMP:OFF") != NULL)
  {
    pumpState = 0;
    HAL_GPIO_WritePin(GPIOA, PUMP_PIN, GPIO_PIN_RESET);
  }

  if(strstr(command, "LED:ON") != NULL)
  {
    ledState = 1;
    HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_SET);
  }
  else if(strstr(command, "LED:OFF") != NULL)
  {
    ledState = 0;
    HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_RESET);
  }
}

void CommunicationTask(void const *argument)
{
  for(;;)
  {
    // Wait for data to process from queue
    osEvent event = osMessageGet(UARTRxQueueHandle, osWaitForever);
    if(event.status == osEventMessage)
    {
      char *commandStr = (char *)event.value.p;
      ProcessCommand(commandStr);
    }

    // Periodically send sensor data to ESP32
    sprintf(uartTxBuffer, "T:%dC H:%d%% M:%d%%\r\n", TCI, RHI, (int)am);
    HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, strlen(uartTxBuffer), 100);

    osDelay(2000); // Send sensor data every 2 seconds
  }
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void microDelay(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT11_Start(void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate);
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
  osDelay(20);
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
  microDelay(30);
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate);
  microDelay(40);
  if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
  {
    microDelay(80);
    if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  uint32_t pMillis = HAL_GetTick();
  uint32_t cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read(void)
{
  uint8_t a, b = 0;
  for (a = 0; a < 8; a++)
  {
    uint32_t pMillis = HAL_GetTick();
    uint32_t cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {
      cMillis = HAL_GetTick();
    }
    microDelay(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
      b &= ~(1 << (7 - a));
    else
      b |= (1 << (7 - a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  HAL_TIM_Base_Start(&htim1);
  lcd_init();
  lcd_clear();

  HAL_UART_Receive_IT(&huart1, &uartRxData, 1);

    // Define UART message queue
    osMessageQDef(UARTRxQueue, 10, uint8_t*);
    UARTRxQueueHandle = osMessageCreate(osMessageQ(UARTRxQueue), NULL);

  /* Create the thread(s) */
  osThreadDef(DHT11Task, DHT11Task, osPriorityAboveNormal, 0, 128);
  DHT11TaskHandle = osThreadCreate(osThread(DHT11Task), NULL);

  osThreadDef(SensorTask, SensorTask, osPriorityNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  osThreadDef(LCDTask, LCDTask, osPriorityNormal, 0, 128);
  LCDTaskHandle = osThreadCreate(osThread(LCDTask), NULL);

  osThreadDef(CommunicationTask, CommunicationTask, osPriorityNormal, 0, 256);
    CommunicationTaskHandle = osThreadCreate(osThread(CommunicationTask), NULL);
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
}

void DHT11Task(void const *argument)
{
  for (;;)
  {
    if (DHT11_Start())
    {
      RHI = DHT11_Read();
      DHT11_Read();
      TCI = DHT11_Read();
      DHT11_Read();
      SUM = DHT11_Read();
      if (RHI + TCI == SUM)
      {
        // Đúng checksum
      }
    }
    osDelay(2000);
  }
}

void SensorTask(void const *argument)
{
  for (;;)
  {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
      soilMoisture = HAL_ADC_GetValue(&hadc1);
      phantram = 4095 - soilMoisture;
      am = ((float)phantram / 4096) * 100;
    }
    HAL_ADC_Stop(&hadc1);
    osDelay(1000);
  }
}

void LCDTask(void const *argument)
{
  for (;;)
  {
    sprintf(temp, "%d C", TCI);  // nhiệt độ
    sprintf(humi, "%d %%", RHI); //  độ ẩm

    lcd_put_cur(0, 0);
    lcd_send_string("T:");
    lcd_put_cur(0, 2);
    lcd_send_string(temp);
    lcd_put_cur(0, 7);
    lcd_send_string("H:");
    lcd_put_cur(0, 9);
    lcd_send_string(humi);

    char buffer[16];
    lcd_put_cur(1, 0);
    lcd_send_string("M:");
    lcd_put_cur(1, 2);
    sprintf(buffer, "%d %%", (int)am);  // Độ ẩm đất
    lcd_send_string(buffer);

    osDelay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | LED_PIN | PUMP_PIN, GPIO_PIN_RESET);

   GPIO_InitStruct.Pin = GPIO_PIN_1 | LED_PIN | PUMP_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
