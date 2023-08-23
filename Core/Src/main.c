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
#include "lcd_app.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

//Dash Display
float tempmin = 29.3;
float tempmax = 29.4;
uint8_t spd = 120;
uint8_t soc = 100;
int avgpow = 2300;
int instpow = 830;
float highvol = 3.65;
float lowvol = 3.62;
char msg[20];

//CAN
uint32_t mailbox;
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];
uint8_t data[12];
uint16_t countCAN = 0;
uint16_t countCANf = 0;

uint8_t RxData[8];
uint8_t data[12];

//ADC
uint16_t adcValue = 0;

//Auxiliary variables
uint32_t auxID = 0x123;
uint8_t Aux_State = 0b00000000;
uint8_t head = 0;
uint8_t cam = 0;
uint8_t tail = 0;
uint8_t horn = 0;
uint8_t breakl = 0; //PB1
uint8_t fan = 0;
uint8_t blink_r = 0;
uint8_t blink_l = 0;

uint8_t head_tail = 0;

uint8_t count = 0;

//Inverter  ++some frame templates
uint8_t power_on = 0; //Run on inv PA10
uint8_t speed = 0; //??uint??
uint8_t forward = 0;  //???uint
uint8_t reverse = 0;  //???uint
uint8_t brake_swap = 0; //ok.

//Internal dash variables
uint8_t next = 0;
uint8_t cruise_up = 0;
uint8_t cruise_down = 0;
uint8_t cruise_button_state = 0;
uint8_t cruise_on = 0;
uint32_t cruise_speed = 0;
uint8_t pedal_count = 0;
uint16_t regen_acc_pedal_adc = 0;
uint8_t mech_brake_pedal_state = 0;

float current_ref = 0.00f;
float rpm_ref = 0.00f;
uint8_t message[8] = { 0 };

//Dashboard
static uint32_t veh_spd = 0;
static uint32_t mot_spd = 0;
static const float *p_veh_spd = (float*) &veh_spd;
static const float *p_mot_spd = (float*) &mot_spd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CAN_Transmit(uint32_t id, uint8_t *data, uint8_t len) {
	/*uint32_t mailbox;
	 CAN_TxHeaderTypeDef txHeader;*/

	txHeader.StdId = id;
	txHeader.ExtId = 0x00;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = len;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &mailbox) != HAL_OK) {
		Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	countCAN++;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	uint32_t canID = RxHeader.StdId;
	switch (canID) {
	// Pack total voltage
	case 0x403:
		veh_spd = ((uint32_t) RxData[7] << 24) + ((uint32_t) RxData[6] << 16)
				+ ((uint32_t) RxData[5] << 8) + ((uint32_t) RxData[4]);

		mot_spd = ((uint32_t) RxData[3] << 24) + ((uint32_t) RxData[2] << 16)
				+ ((uint32_t) RxData[1] << 8) + ((uint32_t) RxData[0]);
		break;
		// Temperatures
	case 0x18B428F4: //Electric cart temps id
		//countCANf++;
		break;
	}
}

//Timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//100ms
	if (htim == &htim3) {
		count++;
		//next = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
		//Auxiliary
		head = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)
				|| HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		cam = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
		tail = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)
				|| HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		horn = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		breakl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1); //check pedal !!!!!!!
		fan = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		blink_r = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
		blink_l = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);

		Aux_State = (head << 7) | (cam << 6) | (tail << 5) | (horn << 4)
				| (breakl << 3) | (fan << 2) | (blink_r << 1) | (blink_l << 0);

		CAN_Transmit(0x701, &Aux_State, 1);

		//Inv
		power_on = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
		forward = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
		reverse = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		//////////////mech_brake_pedal_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

		// Cruise control
		brake_swap = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
		mech_brake_pedal_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

		//Start ADC conversion
		HAL_ADC_Start(&hadc1);
		// Wait for ADC conversion to complete
		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
			adcValue = HAL_ADC_GetValue(&hadc1);
		}
		//Stop ADC conversion
		HAL_ADC_Stop(&hadc1);

		// MinMax value
		adcValue = MIN(adcValue, 3530);
		adcValue = MAX(adcValue, 200);

		//Getting out of the cruise control on user input
		if (mech_brake_pedal_state || brake_swap) //||adcValue>250
				{
			cruise_on = 0;
		}
		//delay the turning off of the cruise from pressing the pedal with two secs so
		//after initially engaging it, it wont turn off immidiately while the driver
		//lifts off the foot.
		if (cruise_on && adcValue > 250) {
			pedal_count++;
			if (pedal_count == 10) {
				cruise_on = 0;
				pedal_count = 0;
			}
		}

		//deciding the rpm and current reference
		if (power_on) {
			current_ref = (float) (adcValue - 200) / 3530.0f / 10;
			if (forward) {
				if (cruise_on) {
					//add min-max values!!!!!!!!!!!!!!!!!!!!!!!!!!!! 30-130 for example
					rpm_ref = cruise_speed;
					current_ref = 0.05;
				} else {
					rpm_ref = 2000;
				}
			}
			if (reverse) {
				cruise_on = 0;
				rpm_ref = -2000;
			}

		} else // if (power_on==0)
		{
			current_ref = 0;
		}

		//send current and rpm refrence to the inverter
		// Copy 'current' bytes in specified order
		uint32_t current_bits = *((uint32_t*) &current_ref);
		message[4] = (current_bits >> 0) & 0xFF;
		message[5] = (current_bits >> 8) & 0xFF;
		message[6] = (current_bits >> 16) & 0xFF;
		message[7] = (current_bits >> 24) & 0xFF;

		// Copy 'rpm' bytes in specified order
		uint32_t rpm_bits = *((uint32_t*) &rpm_ref);
		message[0] = (rpm_bits >> 0) & 0xFF;
		message[1] = (rpm_bits >> 8) & 0xFF;
		message[2] = (rpm_bits >> 16) & 0xFF;
		message[3] = (rpm_bits >> 24) & 0xFF;

		CAN_Transmit(0x501, message, 8);
	}
	//1sec
	if (htim == &htim4) {
		//1 sec is better for the cruise_on becouse of the debouncing.
		cruise_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		if (cruise_button_state == 1) {
			cruise_speed = *p_mot_spd;
		}
		//Deciding if the cruise control should be on or off
		if (cruise_button_state) {
			if (cruise_on) {
				cruise_on = 0;
			} else if (!cruise_on&&*p_mot_spd>200) {//////not final value
				cruise_on = 1;
			}
		}
		cruise_down = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
		cruise_up = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		/////!!!!!!!!!!!!!!!!!!!!!!!!!not final step(the 10 value)
		if (cruise_on && cruise_up && cruise_speed < 700) {
			cruise_speed += 10;
		}
		if (cruise_on && cruise_down && cruise_speed > 200) {
			cruise_speed -= 10;
		}

	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	countCAN = 0;
	message[0] = 0;
	adcValue = 0;
	Aux_State = 0;
	cruise_speed = 0;
	count++;
	current_ref = 0;

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
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_CAN_Start(&hcan);
	HAL_Delay(50);
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Initialize */
	HD44780_Init(4);

	/* Clear buffer */
	HD44780_Clear();

	SetUpDisplay(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		updateTempsMin(&hi2c1, tempmin);
		updateTempsMax(&hi2c1, tempmax);
		updateHighVoltage(&hi2c1, highvol);
		updateLowVoltage(&hi2c1, lowvol);
		updateSpeed(&hi2c1, *p_veh_spd);
		updateSOC(&hi2c1, soc);
		updateAvgPower(&hi2c1, avgpow);
		updateInstPower(&hi2c1, instpow);
		updateMessage((char*) &msg);
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
  htim3.Init.Period = 999;
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
  htim4.Init.Period = 4999;
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

  /*Configure GPIO pins : PC0 PC1 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA7 PA8 PA9
                           PA10 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_13;
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
	__disable_irq();
	while (1) {
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
