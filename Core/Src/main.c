/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
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
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t dataRec[UART_REC_BUFFER_SIZE];
uint8_t dataTran[UART_TRAN_BUFFER_SIZE];
// uint8_t dataRecHeader[UART_HEADER_SIZE];
// uint8_t dataMsgRec[UART_REC_BUFFER_SIZE];
UART_Rec_State uart_rec_stage_ = WAITING_START;

int16_t velLeft;
int16_t velRight;

uint32_t encoder_pulse1 = 0, encoder_pulse2 = 0;
uint32_t count_temp1 = 0, count_temp2 = 0, count_test=0;
uint32_t count_recent1 =0, count_recent2 =0, count_update1=0, count_update2=0;
uint32_t tran_cnt = 0, rec_cnt = 0;
int16_t motor_speed1=0, motor_speed2 =0;

float kp, ki, kd; // need save to ROM
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  uart_rec_stage_ = WAITING_START;
  HAL_UART_Receive_DMA(&huart1, dataRec, 1);

  // memset(dataTran, 0, UART_TRAN_BUFFER_SIZE);
  // HAL_UART_Transmit_DMA(&huart1, dataTran, UART_TRAN_BUFFER_SIZE);
  // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
//  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // uint16_t pwmLeft = 0;
  // uint16_t pwmRight = 0;
  while (1)
  {
//	HAL_UART_Transmit_DMA(&huart1, dataTran, UART_TRAN_BUFFER_SIZE);
	// HAL_Delay(10);
	//   pwmLeft = abs((uint16_t)(velLeft*1000/37));
	//   pwmRight = abs((uint16_t)(velRight*1000/37));
	//   pwmLeft = (pwmLeft > 1000) ? 1000 : pwmLeft;
	//   pwmRight = (pwmRight> 1000) ? 1000 : pwmRight;
	//   if(velLeft > 0){
	// 	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
	// 	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	// 	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmLeft);
	//   }
	//   else if (velLeft < 0) {
	// 	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	// 	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	// 	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmLeft);
	//   }
	//   else {
	// 	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	// 	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
	//   }


	//   if(velRight < 0){
	// 	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
	// 	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	// 	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwmRight);
	//   }
	//   else if (velRight > 0){
	// 	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	// 	  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
	// 	  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwmRight);
	//   }
	//   else {
	// 	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	// 	  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
	//   }


//	char msg[50];
//	sprintf(msg, "SL: %i - SR: %i\n", motor_speed1, motor_speed2);
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
//	HAL_Delay(500);
    /* USER CODE END WHILE */

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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 7200-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

  if (uart_rec_stage_ == WAITING_START){ //Process header
    if(dataRec[0] != 0x2A) {
      // Read until face start byte
      HAL_UART_Receive_DMA(&huart1, dataRec, 1);
    }
    else {
      uart_rec_stage_ = WAITING_HEADER;
      HAL_UART_Receive_DMA(&huart1, dataRec, UART_HEADER_SIZE-1);
    }
    return;
  }

  if (uart_rec_stage_ == WAITING_HEADER){
    u_int8_t cmd_count_index = 0;
    uint32_t curr_rec_count = (uint32_t)dataRec[cmd_count_index]   << 24
                            | (uint32_t)dataRec[cmd_count_index+1] << 16
                            | (uint32_t)dataRec[cmd_count_index+2] << 8
                            | dataRec[cmd_count_index+3];
    if(curr_rec_count == rec_cnt) return;
    rec_cnt = curr_rec_count;
    u_int8_t cmdLen = dataRec[cmd_count_index + 4];
    HAL_UART_Receive_DMA(&huart1, dataRec, cmdLen - UART_HEADER_SIZE);
  }

  if (uart_rec_stage_ == WAITING_MSG) {
    handleMessage();
    uart_rec_stage_ = WAITING_START;
    HAL_UART_Receive_DMA(&huart1, dataRec, 1);
  }


	// Feedback cmd

//	char msg[30];
//	sprintf(msg, "velL: %i, velR: %i\n", velLeft, velRight);
//
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	dataTran[7] += 1;
}

void handleMessage(){
	u_int8_t cmd	= dataRec[0];
	uint8_t params_index = 1;
	switch (cmd) {
		case 0x50: // Set rpm
			velLeft = (int)dataRec[params_index] << 8 | dataRec[params_index+1];
			velRight = (int)dataRec[params_index+2] << 8 | dataRec[params_index+3];
      setRpm(velLeft, velRight);
      responseCmd(CMD_SET_VEL);
      break;
    case 0x55: // Set Kp
      memcpy(&kp, &dataRec[params_index], 4);
      responseCmd(CMD_SET_PID_KP);
			break;
    case 0x56: // Set Kp
      memcpy(&ki, &dataRec[params_index], 4);
      responseCmd(CMD_SET_PID_KI);
			break;
    case 0x57: // Set Kp
      memcpy(&kd, &dataRec[params_index], 4);
      responseCmd(CMD_SET_PID_KD);
			break;
		default:
			break;
	}
}

void setRpm(int16_t velLeft, int16_t velRight) {
  uint16_t pwmLeft = 0;
  uint16_t pwmRight = 0;
  pwmLeft = abs((uint16_t)(velLeft*1000/37));
  pwmRight = abs((uint16_t)(velRight*1000/37));
  pwmLeft = (pwmLeft > 1000) ? 1000 : pwmLeft;
  pwmRight = (pwmRight> 1000) ? 1000 : pwmRight;
  if(velLeft > 0){
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmLeft);
  }
  else if (velLeft < 0) {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmLeft);
  }
  else {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
  }

  if(velRight < 0){
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwmRight);
  }
  else if (velRight > 0){
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwmRight);
  }
  else {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
  }
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void responseCmd(CMD_Type cmd_type) {
  memset(dataTran, 0, UART_TRAN_BUFFER_SIZE);
  dataTran[0] = 0x2B;
  tran_cnt++;
  uint8_t cnt_index = 1;
  dataTran[cnt_index] = tran_cnt >> 24;
  dataTran[cnt_index + 1] = tran_cnt >> 16;
  dataTran[cnt_index + 2] = tran_cnt >> 8;
  dataTran[cnt_index + 3] = tran_cnt;
  uint8_t msg_length_index = cnt_index + 4;
  uint8_t cmd_type_index = msg_length_index + 1;
  uint8_t msg_index = cmd_type_index + 1;
  dataTran[cmd_type_index] = cmd_type;
  uint8_t msg_length = 0;
  switch (cmd_type)
  {
  case CMD_FB_VEL:
    msg_length = (uint8_t)12;
    dataTran[msg_length_index] = msg_length;
    dataTran[msg_index] = motor_speed1 >> 8;
    dataTran[msg_index+1] = motor_speed1;
    dataTran[msg_index+2] = motor_speed2 >> 8;
    dataTran[msg_index+3] = motor_speed2;
    break;
  case CMD_SET_VEL:
    msg_length = (uint8_t)12;
    dataTran[msg_length_index] = msg_length;
    dataTran[msg_index] = velLeft >> 8;
    dataTran[msg_index+1] = velLeft;
    dataTran[msg_index+2] = velRight >> 8;
    dataTran[msg_index+3] = velRight;
    break;
  case CMD_SET_PID_KP:
    msg_length = (uint8_t)12;
    dataTran[msg_length_index] = msg_length;
    dataTran[msg_index] = (uint32_t)kp >> 24;
    dataTran[msg_index+1] = (uint32_t)kp >> 16;
    dataTran[msg_index+2] = (uint32_t)kp >> 8;
    dataTran[msg_index+3] = (uint32_t)kp;
  case CMD_SET_PID_KI:
    msg_length = (uint8_t)12;
    dataTran[msg_length_index] = msg_length;
    dataTran[msg_index] = (uint32_t)kp >> 24;
    dataTran[msg_index+1] = (uint32_t)kp >> 16;
    dataTran[msg_index+2] = (uint32_t)kp >> 8;
    dataTran[msg_index+3] = (uint32_t)kp;
  case CMD_SET_PID_KD:
    msg_length = (uint8_t)12;
    dataTran[msg_length_index] = msg_length;
    dataTran[msg_index] = (uint32_t)kp >> 24;
    dataTran[msg_index+1] = (uint32_t)kp >> 16;
    dataTran[msg_index+2] = (uint32_t)kp >> 8;
    dataTran[msg_index+3] = (uint32_t)kp;
  case CMD_STATUS:
    break;
  default:
    break;
  }
  dataTran[msg_length-1] = 0x3F;
  HAL_UART_Transmit_DMA(&huart1, dataTran, msg_length);
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

#ifdef  USE_FULL_ASSERT
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
