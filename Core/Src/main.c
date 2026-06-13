	/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2026 STMicroelectronics.
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

	/* USER CODE END Includes */

	/* Private typedef -----------------------------------------------------------*/
	/* USER CODE BEGIN PTD */
	uint8_t ON[8] = {0x03,0x00,0x00,0x30,0x00,0x00,0x00,0x00};
	uint8_t OFF[8] = {0x03,0x00,0x00,0x30,0x00,0x01,0x00,0x00};
	uint8_t Set_220V[8] = {0x03,0x00,0x00,0x21,0x43,0x5C,0x00,0x00};
	uint8_t Set_50V[8] = {0x03,0x00,0x00,0x21,0x42,0x48,0x00,0x00};
	uint8_t Set_3A[8] = {0x03,0x00,0x00,0x22,0x3D,0xCC,0xCC,0xCD};

	uint8_t reStatus[8]   = {0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00};
	uint8_t reVoltage[8]  = {0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	uint8_t reCurrent[8]  = {0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
	uint8_t reTemp   [8]  = {0x10, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};

	float fb_votage = 0, fb_current = 0, fb_temp = 0;
	CAN_TxHeaderTypeDef   TxHeader;
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t RxData[8];
	CAN_FilterTypeDef sFilterConfig_0;
	uint32_t TxMailbox;
	uint8_t state = 0;
	uint32_t test = 0;

	enum {
		SLEEP,
		OFFLINE,
		WAIT_REPLY,
		SEND_RQ_READ_STT,
		CAL_TEMP,
		SEND_RQ_READ_VOLTAGE,
		SEND_RQ_READ_CURRENT,
		SEND_RQ_READ_TEMPERATURE
	}Can_state;

	/* USER CODE END PTD */

	/* Private define ------------------------------------------------------------*/
	/* USER CODE BEGIN PD */

	/* USER CODE END PD */

	/* Private macro -------------------------------------------------------------*/
	/* USER CODE BEGIN PM */

	/* USER CODE END PM */

	/* Private variables ---------------------------------------------------------*/
	CAN_HandleTypeDef hcan;

	/* USER CODE BEGIN PV */

	/* USER CODE END PV */

	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	static void MX_GPIO_Init(void);
	static void MX_CAN_Init(void);
	/* USER CODE BEGIN PFP */

	/* USER CODE END PFP */

	/* Private user code ---------------------------------------------------------*/
	/* USER CODE BEGIN 0 */
	float convertFloat(uint8_t *data){
		union{
			float result;
			uint8_t cop_dat[4];
		}get_result;

		get_result.cop_dat[3] = data[4];
		get_result.cop_dat[2] = data[5];
		get_result.cop_dat[1] = data[6];
		get_result.cop_dat[0] = data[7];

		return get_result.result;

	}

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	  if (GPIO_Pin == GPIO_PIN_13){
		state = !state;
	  }
	}

	void maxwell_init()
	{
		sFilterConfig_0.FilterBank = 0;
		sFilterConfig_0.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig_0.FilterScale = CAN_FILTERSCALE_32BIT;

		sFilterConfig_0.FilterIdHigh     = 0x0000;
		sFilterConfig_0.FilterIdLow      = 0x0000;
		sFilterConfig_0.FilterMaskIdHigh = 0x0000;
		sFilterConfig_0.FilterMaskIdLow  = 0x0000;

		sFilterConfig_0.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig_0.FilterActivation = ENABLE;
		HAL_CAN_ConfigFilter(&hcan, &sFilterConfig_0);

		HAL_CAN_Start(&hcan);
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.ExtId = 0x06080780;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;
	}


	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
		if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxData) == HAL_OK){
			if(RxHeader.ExtId == 0x060F8000){
				if(Can_state == WAIT_REPLY){
					if     (RxData[3] == 0x40) Can_state = SEND_RQ_READ_VOLTAGE;
					else if(RxData[3] == 0x01) Can_state = SEND_RQ_READ_CURRENT;
					else if(RxData[3] == 0x02) Can_state = SEND_RQ_READ_TEMPERATURE;
					else if(RxData[3] == 0x04) Can_state = CAL_TEMP;
				}
			}
		}

	}

	void process(){
		static uint32_t time_request = 0;
		static uint8_t last_state = 0;
		if((HAL_GetTick()-time_request>=2000) && Can_state == SLEEP){
			test+=1;
			Can_state = SEND_RQ_READ_STT;
			time_request = HAL_GetTick();
		}
		if(state != last_state){
			if(state == 1){
			  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
			  HAL_CAN_AddTxMessage(&hcan,&TxHeader,ON,&TxMailbox);
			  Can_state = SLEEP;
			}else{
			  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
			  HAL_CAN_AddTxMessage(&hcan,&TxHeader,OFF,&TxMailbox);
			  Can_state = OFFLINE;
			}
			last_state = state;
		}



		switch(Can_state){
			case SEND_RQ_READ_VOLTAGE:
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
				if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,reVoltage, &TxMailbox)==HAL_OK){
					Can_state = WAIT_REPLY;
				}
			break;
			case SEND_RQ_READ_CURRENT:
				fb_votage = convertFloat(RxData);
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
				if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,reCurrent, &TxMailbox)==HAL_OK){
					Can_state = WAIT_REPLY;
				}
			break;
			case SEND_RQ_READ_TEMPERATURE:
				fb_current = convertFloat(RxData);
				if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,reTemp, &TxMailbox)==HAL_OK){
					Can_state = WAIT_REPLY;
				}
			break;
			case CAL_TEMP:
				fb_temp = convertFloat(RxData);
				Can_state = SLEEP;
			break;
			case SEND_RQ_READ_STT:
				while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
				if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,reStatus, &TxMailbox) == HAL_OK){
					Can_state = WAIT_REPLY;
				}
			break;
			case OFFLINE:
				fb_current = convertFloat(RxData);
				fb_votage = convertFloat(RxData);
				fb_temp = convertFloat(RxData);
			break;
			case WAIT_REPLY:
			default:
			break;
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
	  /* USER CODE BEGIN 2 */
	  maxwell_init();
	  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	  HAL_CAN_AddTxMessage(&hcan,&TxHeader,Set_50V,&TxMailbox);
	  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	  HAL_CAN_AddTxMessage(&hcan,&TxHeader,Set_3A,&TxMailbox);
	  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	  HAL_CAN_AddTxMessage(&hcan,&TxHeader,OFF,&TxMailbox);
	  Can_state = OFFLINE;

	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		  process();
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
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
	  hcan.Init.Prescaler = 16;
	  hcan.Init.Mode = CAN_MODE_NORMAL;
	  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	  hcan.Init.TimeTriggeredMode = DISABLE;
	  hcan.Init.AutoBusOff = DISABLE;
	  hcan.Init.AutoWakeUp = DISABLE;
	  hcan.Init.AutoRetransmission = ENABLE;
	  hcan.Init.ReceiveFifoLocked = DISABLE;
	  hcan.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&hcan) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN CAN_Init 2 */

	  /* USER CODE END CAN_Init 2 */

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

	  /*Configure GPIO pin : button_Pin */
	  GPIO_InitStruct.Pin = button_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
