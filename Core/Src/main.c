/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include"stdio.h"
#include"added_function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum State bms_state = IDLE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CRUISE_CONTROL_MAX_SPEED 800.0f
#define CRUISE_CONTROL_MIN_SPEED 100.0f
#define PEDAL_MIN 200
#define PEDAL_MAX 3530


//#define PEDAL_MAX 2630
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Buttons' state
state but_state = { 0 };

// CAN outputs
static uint8_t bms_data[8] = { 0 };
static uint8_t inv_data[8] = { 0 };
uint8_t* const p_bms_data = bms_data;
uint8_t* const p_inv_data = inv_data;
uint8_t* const p_aux_data = &but_state.aux_state;

//Dashboard
static uint32_t bus_current = 0;
float* const p_bus_current = (float*) &bus_current;

static uint32_t bus_voltage = 0;
float* const p_bus_voltage = (float*) &bus_voltage;

static int32_t bat_current = 0;
int32_t* const p_bat_current = &bat_current;

static uint32_t bat_voltage = 0;
uint32_t* const p_bat_voltage = &bat_voltage;

static int16_t mppt1_current = 0;
int16_t* const p_mppt1_current = &mppt1_current;

static int16_t mppt1_voltage = 0;
int16_t* const p_mppt1_voltage = &mppt1_voltage;

static int16_t mppt2_current = 0;
int16_t* const p_mppt2_current = &mppt2_current;

static int16_t mppt2_voltage = 0;
int16_t* const p_mppt2_voltage = &mppt2_voltage;

static uint32_t veh_spd = 0;
float* const p_veh_spd = (float*) &veh_spd;

static uint32_t mot_spd = 0;
float* const p_mot_spd = (float*) &mot_spd;

static uint32_t tmp_min = 0;
float* const p_tmp_min = (float*) &tmp_min;

static uint32_t tmp_max = 0;
float* const p_tmp_max = (float*) &tmp_max;

static uint32_t vol_min = 0;
float* const p_vol_min = (float*) &vol_min;

static uint32_t vol_max = 0;
float* const p_vol_max = (float*) &vol_max;

static uint32_t cur_ref = 0;
float* const p_cur_ref = (float*) &cur_ref;
float current_ref = 0;

static uint32_t charge = 0;
float* const p_charge = (float*) &charge;

static uint32_t crs_spd = 0;
float* const p_crs_spd = (float*) &crs_spd;

uint32_t pedal_delay = 0;

//VARIABILE ADAUGATE DE PAUL
	//eroare software overcurrent
	uint8_t SWOC_flag = 0;

	float prev_current_ref = 0;
	static float delta = 0.05f;

	uint32_t bus_current_mailbox = CAN_TX_MAILBOX2;

	static const CAN_TxHeaderTypeDef bus_current_header = {0x502,0x00,CAN_RTR_DATA,CAN_ID_STD,8,DISABLE};
	static uint8_t bus_current_data[8];

	static float bus_current_limit = 1.0f;
	#define  MAX_CURRENT_REF  0.5f
	#define SPEED_LIMIT  20


	extern float Power_Sum_Print;

	float bus_pow = 0;

	extern uint8_t Start_Display_Power;
	extern uint8_t Display_Counter;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void set_invertor_state()
{
	bms_data[0] = (mot_spd) & 0xFF;
	bms_data[1] = (mot_spd >> 8) & 0xFF;
	bms_data[2] = (mot_spd >> 16) & 0xFF;
	bms_data[3] = (mot_spd >> 24) & 0xFF;

	bms_data[4] = (cur_ref) & 0xFF;
	bms_data[5] = (cur_ref >> 8) & 0xFF;
	bms_data[6] = (cur_ref >> 16) & 0xFF;
	bms_data[7] = (cur_ref >> 24) & 0xFF;
}
void update_display(I2C_HandleTypeDef *hi2c1, char *msg)
{
	if (!hi2c1)
		return;
	// If dashboard is disconnected
	if (HAL_I2C_IsDeviceReady(hi2c1, DEVICE_ADDR, 2, 10) != HAL_OK)
		return;

	char buffer[21];
	float mppt_pow = ((float)((float)mppt1_current + (float)mppt2_current) * (float)mppt1_voltage) / 100.0F;
	float bat_pow = ((float)bat_current * (float)bat_voltage) / 1000000.0F;
	bus_pow = *p_bus_current * *p_bus_voltage;

	// Display first row
	HD44780_SetCursor(0, 0);
	snprintf(buffer, 21, "%4.1f |V: %4.0f | %4.2f", *p_tmp_min, (*p_veh_spd) * 3.6, *p_vol_min);
	HD44780_PrintStr(buffer);

	// Display second row
	HD44780_SetCursor(0, 1);
	snprintf(buffer, 21, "%4.1f |%%: %4.0f | %4.2f", *p_tmp_max, (*p_charge) * 100.f, *p_vol_max);
	HD44780_PrintStr(buffer);

	// Display third row
	HD44780_SetCursor(0, 2);
	snprintf(buffer, 21, "%4.0f | %6.0f | %4.0f", mppt_pow, bat_pow, bus_pow);
	HD44780_PrintStr(buffer);

	// Display message
	HD44780_SetCursor(0, 3);
	HD44780_PrintStr(msg);
	HD44780_SetCursor(8, 3);
	if(Start_Display_Power == 1 && Display_Counter < 10)
	{
		snprintf(buffer, 21, "NEW LAP!!!  ");
	}
	else if(Display_Counter == 10 || Start_Display_Power == 0)
	{
		snprintf(buffer, 21, "Prv Lap:%4.0f", Power_Sum_Print); //contiuna
	}

	HD44780_PrintStr(buffer);

	HD44780_Display();
}
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
	//Delay cum au spus baietii de pe forumuri, nu merge 100% fara
		long j = 100000;
		while(--j){}

	uint16_t pedal_gradient = 0;

	char msg[20] = "\0";



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
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_CAN_Start(&hcan);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_Delay(50);
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Initialize */
	HD44780_Init(4);

	// Clear screen buffer
	HD44780_Clear();

	// BMS idle state
	p_inv_data[0] = 0x00;
	p_inv_data[1] = 0x00;

	memcpy(&bus_current_data[4], &bus_current_limit, sizeof(float));
	if (HAL_CAN_AddTxMessage(&hcan, &bus_current_header, bus_current_data, &bus_current_mailbox) != HAL_OK)
	{
		 Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		update_display(&hi2c1, msg);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Cast reference values into CAN output
		set_invertor_state();

		// If power on button is off keep idle state
		if (!but_state.power_on) {
			*p_cur_ref = 0.0f;
			*p_mot_spd = 0.0f;
			p_inv_data[0] = 0x00;
			strcpy(msg, "M Idle");
			continue;
		}

		// Enable BMS modes
		switch (bms_state) {
			case IDLE:
			case PRE_CHARGE:
				// Enable Accessories + Start + Run modes
				p_inv_data[0] = 0x70;
				strcpy(msg, "PreChg");
				continue;
			case DRIVE:
				// Enable Accessories + Run modes
				p_inv_data[0] = 0x30;
				break;
			case ERR:
			default:
				// Enter safe-state
				p_inv_data[0] = 0x00;

				//de resetat cand intra in eroare
		}

		if (pedal_delay == 0 ) {
			// Read pedal gradient
			HAL_Delay(50);
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
				pedal_gradient = HAL_ADC_GetValue(&hadc1);
			}
			HAL_ADC_Stop(&hadc1);
			// Normalize pedal gradient
			pedal_gradient = MIN(pedal_gradient, PEDAL_MAX);
			pedal_gradient = MAX(pedal_gradient, PEDAL_MIN);
		} else {
			pedal_delay--;
		}

		// Disable CC if pedal is pressed
		if (pedal_gradient > PEDAL_MIN || but_state.brake_swap == 1 || but_state.brake_state == 1)
			but_state.cruise_mode = 0;

		// Cruise speed control
		if (but_state.cruise_up && *p_crs_spd < CRUISE_CONTROL_MAX_SPEED)
			*p_crs_spd += 10.0f;

		if (but_state.cruise_down && *p_crs_spd > CRUISE_CONTROL_MIN_SPEED)
			*p_crs_spd -= 10.0f;

		// If CC is enabled keep previous reference current value
		if (but_state.brake_swap) {
			strcpy(msg, "Brk on");
			*p_cur_ref = (float)(pedal_gradient - PEDAL_MIN) / (float)(PEDAL_MAX - PEDAL_MIN);;
			*p_mot_spd = 0.0f;
		} else if (but_state.cruise_mode && *p_crs_spd > 100.0f) {
			strcpy(msg, "Cr Con");
			*p_cur_ref = 0.2f;
			*p_mot_spd = *p_crs_spd;
		} else if (but_state.drv_forward) {
				strcpy(msg, "Dr fwd ");
				//Testtt SWOC = Software overcurrent
					if(SWOC_flag == 1)
					{
						current_ref = (float)(pedal_gradient - PEDAL_MIN) / (float)(PEDAL_MAX - PEDAL_MIN);
						if(current_ref - prev_current_ref >= delta)
						{
							*p_cur_ref += delta;
						}
						else *p_cur_ref = current_ref;
						if (*p_cur_ref > MAX_CURRENT_REF && *(p_veh_spd) * 3.6 <= SPEED_LIMIT)
						{
							*p_cur_ref = MAX_CURRENT_REF;
						}
						prev_current_ref = current_ref;
						SWOC_flag = 0;
					}
				*p_mot_spd = 2000.0f;  // To quickly accelerate set large angular velocity reference
		} else if (but_state.drv_reverse) {
			strcpy(msg, "Dr rvs");
			*p_cur_ref = (float)(pedal_gradient - PEDAL_MIN) / (float)(PEDAL_MAX - PEDAL_MIN);
			*p_mot_spd = -2000.0f;  // To quickly accelerate set large angular velocity reference
		} else {
			// Neutral state
			strcpy(msg, "N mode");
			*p_cur_ref = 0.0f;
			*p_mot_spd = 0.0f;
		}

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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = 0; //this tells what to compare the incoming data to
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0; //this tells which bits to compare
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 10;

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 PC4 PC7
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA7 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 PB14 PB15
                           PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	NVIC_SystemReset();
	__disable_irq();
	if (hcan.ErrorCode != HAL_CAN_ERROR_NONE)
	{
		// Deinit CAN
		if (HAL_CAN_DeInit(&hcan) != HAL_OK)
		{
			while(1) {}
		}
		// Init CAN
		if (HAL_CAN_Init(&hcan) != HAL_OK)
		{
			Error_Handler();
		}
		// Start CAN
		if (HAL_CAN_Start(&hcan) != HAL_OK)
		{
			Error_Handler();
		}
	}
	__enable_irq();
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
