/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"added_function.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CRUISE_CONTROL_STEP 10
#define CAN_EMERGEN 0x100
#define CAN_PW1_MSG 0x200
#define CAN_PW2_MSG 0x210
#define CAN_BUS_MSG 0x402
#define CAN_MTR_MSG 0x403
#define CAN_SOC_MSG 0x6F4
#define CAN_BMS_STS 0x6F7
#define CAN_VOL_MSG 0x6F8
#define CAN_TMP_MSG 0x6F9
#define CAN_BATTERY 0x6FA

#define MASK_PWR_ON 0x0001
#define MASK_DRV_FW 0x0002
#define MASK_DRV_RV 0x0004
#define MASK_BRK_SW 0x0008
#define MASK_BRK_ST 0x0010
#define MASK_CRU_MD 0x0020
#define MASK_CRU_UP 0x0040
#define MASK_CRU_DW 0x0080
#define MASK_BLIN_L 0x0100
#define MASK_BLIN_R 0x0200
#define MASK_FAN    0x0400
#define MASK_LIG_BR 0x0800
#define MASK_HORN   0x1000
#define MASK_LIG_RR 0x2000
#define MASK_CAMERA 0x4000
#define MASK_LIG_HD 0x8000

#define SOFTWARE_OVERCURRENT_ERROR 0x401

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static const CAN_TxHeaderTypeDef bms_header = { 0x501, 0x00, CAN_RTR_DATA, CAN_ID_STD, 8, DISABLE };
static const CAN_TxHeaderTypeDef inv_header = { 0x505, 0x00, CAN_RTR_DATA, CAN_ID_STD, 8, DISABLE };
static const CAN_TxHeaderTypeDef aux_header = { 0x701, 0x00, CAN_RTR_DATA, CAN_ID_STD, 1, DISABLE };
static const CAN_TxHeaderTypeDef inv_error_header = {0x503,0x00,CAN_RTR_DATA,CAN_ID_STD,1,DISABLE};

//Adaugat de Paul, SWOC


//for UART
extern UART_HandleTypeDef huart2;
extern uint8_t safe_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void toggle_state(button* but, uint16_t mask, uint8_t input)
{
	extern state but_state;
	if (!but)
		return;

	extern state but_state;
	if (input == 1)
		but->counter++;
	else if (input == 0)
		but->counter = 0;

	if (but->counter == but->counter_limit) {
		if ((but_state.button_states & mask) == 0)
			but_state.button_states |= mask;
		else
			but_state.button_states &= ~mask;
	}
}
void impulse_state(button* but, uint16_t mask, uint8_t input)
{
	extern state but_state;
	if (!but)
		return;

	extern state but_state;
	if (input == 1)
		but->counter++;
	else if (input == 0)
		but->counter = 0;

	if (but->counter == but->counter_limit)
		but_state.button_states |= mask;
	else
		but_state.button_states &= ~mask;
}
void push_state(button* but, uint16_t mask, uint8_t input)
{
	if (!but)
		return;

	extern state but_state;
	if (input == 1 && but->counter < but->counter_limit)
		but->counter++;
	else if (input == 0)
		but->counter = 0;

	if (but->counter == but->counter_limit)
		but_state.button_states |= mask;
	else
		but_state.button_states &= ~mask;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// @formatter:off

static button cruise_mode = { 0, 5, 0 };
static button cruise_up   = { 0, 2, 0 };
static button cruise_down = { 0, 2, 0 };
static button head_lights = { 0, 2, 0 };
static button rear_lights = { 0, 2, 0 };
static button brake_light = { 0, 2, 0 };
static button camera      = { 0, 2, 0 };
static button horn        = { 0, 2, 0 };
static button fan         = { 0, 2, 0 };
static button blink_left  = { 0, 2, 0 };
static button blink_right = { 0, 2, 0 };
static button power_on    = { 0, 2, 0 };
static button drv_forward = { 0, 2, 0 };
static button drv_reverse = { 0, 2, 0 };
static button brake_swap  = { 0, 2, 0 };
static button brake_state = { 0, 2, 0 };
// @formatter:on
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */
extern enum State bms_state;
extern state but_state;
extern float *const p_bus_current;
extern float *const p_bus_voltage;
extern int32_t *const p_bat_current;
extern uint32_t *const p_bat_voltage;
extern uint16_t *const p_mppt1_current;
extern uint16_t *const p_mppt1_voltage;
extern uint16_t *const p_mppt2_current;
extern uint16_t *const p_mppt2_voltage;
extern float *const p_veh_spd;
extern float *const p_mot_spd;
extern float *const p_tmp_min;
extern float *const p_tmp_max;
extern float *const p_vol_min;
extern float *const p_vol_max;
extern float *const p_cur_ref;
extern float *const p_charge;
extern float *const p_crs_spd;
extern uint8_t *const p_bms_data;
extern uint8_t *const p_inv_data;
extern uint8_t *const p_aux_data;
extern uint32_t cruise_speed;
extern uint16_t pedal_gradient;
extern uint32_t pedal_delay;


extern uint8_t SWOC_flag;
float Power_Sum_Print  = 0;
uint8_t Power_Button_Pressed = 0;
uint8_t Start_Display_Power = 0;
uint8_t Display_Counter = 0;


extern float bus_pow;
extern uint8_t auxiliary_safe_state;

//extern float current_ref;



/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
	  NVIC_SystemReset();
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
	  NVIC_SystemReset();
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */
	CAN_RxHeaderTypeDef header;
	uint32_t bus_current, bus_voltage, veh_speed, mot_speed, soc;
	uint16_t min_voltage, max_voltage, min_temp, max_temp;
	uint8_t data[8];
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data);
	switch (header.StdId) {
	// Emergency button
	case CAN_EMERGEN:
		safe_state = data[0];
		break;
	// Output power form front MPPT
	case CAN_PW1_MSG:
		// @formatter:off
		*p_mppt1_voltage = ((data[4] << 8) + data[5]) * 0.01;  // scale factor of 0.01

		*p_mppt1_current = ((data[6] << 8) + data[7]) * 0.0005;  // scale factor of 0.0005
		// @formatter:on
		break;
	// Output power form MPPT
	case CAN_PW2_MSG:
		// @formatter:off
		*p_mppt2_voltage = ((data[4] << 8) + data[5]) * 0.01;  // scale factor of 0.01

		*p_mppt2_current = ((data[6] << 8) + data[7]) * 0.0005;  // scale factor of 0.0005
		// @formatter:on
		break;
	// Inverter DC Bus Voltage & Current
	case CAN_BUS_MSG:
		// @formatter:off
		bus_current = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4];
		memcpy(p_bus_current, &bus_current, sizeof(float));

		bus_voltage = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
		memcpy(p_bus_voltage, &bus_voltage, sizeof(float));
				// @formatter:on
		break;
	// Speed
	case CAN_MTR_MSG:
		// @formatter:off
		veh_speed = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4];
		memcpy(p_veh_spd, &veh_speed, sizeof(float));

		mot_speed = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
		if (but_state.cruise_mode == 0)
			memcpy(p_crs_spd, &mot_speed, sizeof(float));
				// @formatter:on
		break;
	//State of charge
	case CAN_SOC_MSG:
		// @formatter:off
		soc = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4];
		memcpy(p_charge, &soc, sizeof(float));
		// @formatter:on
		break;
	// Minimum & maximum voltages
	case CAN_VOL_MSG:
		// @formatter:off
		min_voltage = (data[1] << 8) + data[0];
		*p_vol_min = (float)min_voltage / 1000;

		max_voltage = (data[3] << 8) + data[2];
		*p_vol_max = (float)max_voltage / 1000;
		// @formatter:on
		break;
	// Minimum & maximum temperatures
	case CAN_TMP_MSG:
		// @formatter:off
		min_temp = (data[1] << 8) + data[0];
		*p_tmp_min = (float)min_temp / 10;

		max_temp = (data[3] << 8) + data[2];
		*p_tmp_max = (float)max_temp / 10;
		// @formatter:on
		break;
	// BMS status codes
	case CAN_BMS_STS:
		switch (data[1]) {
		case 1:  // Idle
			bms_state = IDLE;
			break;
		case 5:  // Enable Pack
		case 2:  // Measure
			break;
		case 3:  // Pre-charge
			bms_state = PRE_CHARGE;
			break;
		case 4:  // Run
			bms_state = DRIVE;
			break;
		case 0:


		default:
			bms_state = ERROR;
			break;
		}

		break;
	// BMU current & voltage
	case CAN_BATTERY:
		// @formatter:off
		*p_bat_voltage = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];

		*p_bat_current = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4];
		// @formatter:on
		break;

	case SOFTWARE_OVERCURRENT_ERROR:
					if(data[2] && 0x02)
					{
						uint32_t error_mailbox = 0;
						uint8_t error_data[1] = {0x00};

						//disable all interrupts to make sure the reset frame will be transmitted as fast as possible.

						__disable_irq();

						while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)) {} //wait for a mailbox to get free
						HAL_CAN_AddTxMessage(&hcan, &inv_error_header, error_data, &error_mailbox); //transmit can frame for error reset

						__enable_irq();
					}
			break;
	}
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	// @formatter:off

	uint32_t bms_mailbox = CAN_TX_MAILBOX0;
	uint32_t inv_mailbox = CAN_TX_MAILBOX1;
	uint32_t aux_mailbox = CAN_TX_MAILBOX2;



	// @formatter:on

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

	// BMS command
	if (HAL_CAN_AddTxMessage(&hcan, &bms_header, p_bms_data, &bms_mailbox)
			!= HAL_OK) {
		Error_Handler();

	}
	// Invertor RPM & current reference command
	if (HAL_CAN_AddTxMessage(&hcan, &inv_header, p_inv_data, &inv_mailbox)
			!= HAL_OK) {
		Error_Handler();
	}

	//AUX status

	if (HAL_CAN_AddTxMessage(&hcan, &aux_header, p_aux_data, &aux_mailbox)
			!= HAL_OK) {
		Error_Handler();
	}

	//Calcul_Putere_Lap();


	SWOC_flag = 1;

// 	Uart_Transmitter(&huart2,*p_cur_ref);



  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	uint8_t button_state;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	// @formatter:off

	button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	push_state(&head_lights, MASK_LIG_HD, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	push_state(&camera, MASK_CAMERA, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	push_state(&rear_lights, MASK_LIG_RR, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	push_state(&horn, MASK_HORN, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) ||  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) || HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	push_state(&brake_light, MASK_LIG_BR, button_state);

	but_state.fan = 0;
	button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	push_state(&fan, MASK_FAN, button_state);

	but_state.blink_right = 0;
	button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	push_state(&blink_right, MASK_BLIN_R, button_state);

	but_state.blink_left = 0;
	button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	push_state(&blink_left, MASK_BLIN_L, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	push_state(&power_on, MASK_PWR_ON, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
	push_state(&drv_forward, MASK_DRV_FW, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
	push_state(&drv_reverse, MASK_DRV_RV, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	push_state(&brake_swap, MASK_BRK_SW, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
	push_state(&brake_state, MASK_BRK_ST, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	impulse_state(&cruise_up, MASK_CRU_UP, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	impulse_state(&cruise_down, MASK_CRU_DW, button_state);

	button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	toggle_state(&cruise_mode, MASK_CRU_MD, button_state);

	//Safe State command
		if(auxiliary_safe_state == 1)
		{
			but_state.fan = 1;
			but_state.blink_left = 1;
			but_state.blink_right = 1;
		}

	if (button_state == 1) {
		pedal_delay = 50;
	}

	//power calculator
	static uint8_t count_power_button_press = 0;

	if( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) ) count_power_button_press++;
	else count_power_button_press = 0;

	if(count_power_button_press == 3)
	{
		Power_Button_Pressed = (!Power_Button_Pressed);
		Start_Display_Power = 1;
	}
	if(Start_Display_Power == 1)
	{
			Display_Counter++;
			if(Display_Counter == 10)
				{
					Display_Counter = 10;
					Start_Display_Power = 0;
				}
	}
	else Display_Counter = 0;
	Calul_Putere_Lap(bus_pow, Power_Button_Pressed, &Power_Sum_Print);

	// @formatter:on
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
