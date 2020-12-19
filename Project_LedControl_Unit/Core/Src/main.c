/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include "StateLed.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mLedGreen    GPIO_PIN_12
#define mLedOrange   GPIO_PIN_13
#define mLedRed      GPIO_PIN_14
#define mLedBlue     GPIO_PIN_15


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef pRxHeader;
CAN_TxHeaderTypeDef pTxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t mTxMailbox;
uint8_t CanReceiveData[8];
uint8_t CanTransData[2];
uint32_t buf[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Init_Tx_Can_Config()
{   // Set Transmit parameters
	  pTxHeader.DLC = 2;
	  pTxHeader.IDE = CAN_ID_STD;
	  pTxHeader.RTR = CAN_RTR_DATA;
	  pTxHeader.StdId = 0x0155; // Id of LedControlUnit

}

void Init_Filter_Can_Config()
{   // Set Filter Parameters
	sFilterConfig.FilterActivation =ENABLE;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterFIFOAssignment =CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh = 0x1994<<5; // Id of MaxMinRangeControl Message
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xFFFF;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

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
  MX_CAN1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  Init_Tx_Can_Config();
  Init_Filter_Can_Config();
  HAL_CAN_Start(&hcan1);


  lcd_init();
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Hi LedControl");
  HAL_Delay(2000);

  // Received parameters from MaxMinRangeControl Unit

  //uint8_t mMotorSpeed;
  //uint8_t state=1; // Initial state of the system
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  MxMinMeasure_t mMeasurement_var = {0};
  MxMinMeasure_t mMaxRange_var = {0};
  MxMinMeasure_t mMinRange_var = {0};

  MxMinMeasure_t* mMeasurement_ptr = &mMeasurement_var;
  MxMinMeasure_t* mMaxRange_ptr = &mMaxRange_var;
  MxMinMeasure_t* mMinRange_ptr = &mMinRange_var;

  STATE_t state = NO_OBJECT;

  MotorSpeed_t mMotorSpeed = {0};



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  mMeasurement_ptr->id   = CanReceiveData[0];
	  mMeasurement_ptr->data = CanReceiveData[1];

	  mMaxRange_ptr->id   = CanReceiveData[2];
	  mMaxRange_ptr->data = CanReceiveData[3];

	  mMinRange_ptr->id   = CanReceiveData[4];
	  mMinRange_ptr->data = CanReceiveData[5];


	  lcd_clear();
	  lcd_put_cur(0, 0);
	  sprintf(buf,"d: %d cm",mMeasurement_ptr->data);
	  lcd_send_string(buf);
	  lcd_put_cur(0, 10);
	  sprintf(buf,"id: %d",mMeasurement_ptr->id);
	  lcd_send_string(buf);
	  lcd_put_cur(1, 0);
	  sprintf(buf,"Max:%d ",mMaxRange_ptr->data);
	  lcd_send_string(buf);
	  lcd_put_cur(1, 8);
	  sprintf(buf,"Min: %d",mMinRange_ptr->data);
	  lcd_send_string(buf);
	  HAL_Delay(100);

      if(mMaxRange_ptr->data == 0 || mMinRange_ptr->data == 0)
    	  state = ERROR_S; // State ERROR
      else if(mMeasurement_ptr->data <= mMinRange_ptr->data)
    	  state = HIGHSEQURITY; // State HIGHSEQURITY
      else if(mMeasurement_ptr->data > mMinRange_ptr->data && mMeasurement_ptr->data <= mMaxRange_ptr->data)
    	  state = LOWSEQURITY; // State LOWSEQURITY
      else if(mMeasurement_ptr->data >= mMaxRange_ptr->data)
    	  state = NO_OBJECT; // State NO_OBJECT

      switch(state)
      {
		  case ERROR_S:
		  {
	    	  HAL_GPIO_WritePin(GPIOD, mLedOrange, GPIO_PIN_SET);
	    	  HAL_GPIO_WritePin(GPIOD, mLedBlue, GPIO_PIN_RESET);
	    	  HAL_GPIO_WritePin(GPIOD, mLedRed, GPIO_PIN_RESET);
	    	  HAL_GPIO_WritePin(GPIOD, mLedGreen, GPIO_PIN_RESET);
	    	  mMotorSpeed.data = 0;
	    	  mMotorSpeed.id = 50;
	    	  CanTransData[1]= mMotorSpeed.data; // mMotorSpeed =0
	    	  CanTransData[0]= mMotorSpeed.id; // mMotorSpeed =0
	    	  HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, CanTransData, &mTxMailbox);// Send mMotorSpeed to MotorServoControl unit
	    	  break;
		  }
		  case HIGHSEQURITY:
		  {
			  HAL_GPIO_WritePin(GPIOD, mLedOrange, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedBlue, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedGreen, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedRed, GPIO_PIN_SET);
	    	  mMotorSpeed.data = 0;
	    	  mMotorSpeed.id = 50;
	    	  CanTransData[0]= mMotorSpeed.id; // mMotorSpeed = 0
	    	  CanTransData[1]= mMotorSpeed.data;
			  HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, CanTransData, &mTxMailbox);// Send mMotorSpeed to MotorServoControl unit
			  break;
		  }
		  case LOWSEQURITY:
		  {
			  HAL_GPIO_WritePin(GPIOD, mLedOrange, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedGreen, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedRed, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedBlue, GPIO_PIN_SET);
	    	  mMotorSpeed.data = 2;
	    	  mMotorSpeed.id = 50;
	    	  CanTransData[0]= mMotorSpeed.id;  // mMotorSpeed id
	    	  CanTransData[1]= mMotorSpeed.data;// mMotorSpeed =2
			  HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, CanTransData, &mTxMailbox);// Send mMotorSpeed to MotorServoControl unit
			  break;
		  }
		  case NO_OBJECT:
		  {
			  HAL_GPIO_WritePin(GPIOD, mLedOrange, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedBlue, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedRed, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, mLedGreen, GPIO_PIN_SET);
	    	  mMotorSpeed.data = 1;
	    	  mMotorSpeed.id = 50;
	    	  CanTransData[0]= mMotorSpeed.id;
	    	  CanTransData[1]= mMotorSpeed.data; // mMotorSpeed =1
			  HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, CanTransData, &mTxMailbox);// Send mMotorSpeed to MotorServoControl unit
			  break;

		  }

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
