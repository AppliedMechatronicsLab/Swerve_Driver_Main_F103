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
#include "as5600.h"
#include "pid.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "i2c_slave.h"
#include "FLASH_PAGE_F1.h"
#include "BUTTON.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_TypeDef RPID;
// BUTTON_Name Mode_Btn;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5600_1_AInit 0
#define PWM_Freq 5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// flash variables
uint32_t Page_Address = 0x0801FC00;
float Data_To_Flash[7] = {2, 0.8, 0.005, 25, -25, 1, 0};
float Rx_Data[7];
// vaiables for AS5600
float Angle = 180;
uint8_t checkAS = 1;
uint32_t count = 0;
uint32_t period;
float prev_angle = 0;
float angle_diff = 0;
float angle_raw = 0;
// variables for PID
double Angle_Temp, PIDOut, TempSetpoint;
float Kp, Ki, Kd;
int8_t Out_Max, Out_Min;
// variables for I2C slave
uint8_t Self_Address = 0x08;
bool I2C_flag = 0;
bool Init_Flag = 1;
// uint8_t Scan_Arr[128];
//  usart variables
uint8_t USART_Short_Tx_Buffer[14];
uint8_t USART_Rx_Buffer[35];
uint8_t USART_Long_Tx_Buffer[42];
uint8_t buffer;
uint8_t rx_index = 0;
bool Debug_Flag = 0;
bool Get_Flag = 0;
uint8_t float_arr_test[4];
float tesst = 0;
bool Set_Flag = 0;

uint8_t Status = 0;
uint8_t Angle_Calib_Check = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void PID_Setup()
{
  Kp = Rx_Data[0];
  Ki = Rx_Data[1];
  Kd = Rx_Data[2];
  Out_Max = Rx_Data[3];
  Out_Min = Rx_Data[4];
  Self_Address = (uint8_t)Rx_Data[5];
  hi2c1.Init.OwnAddress1 = Self_Address << 1;

  TempSetpoint = AS5600_1_AInit;
  PID(&RPID, &Angle_Temp, &PIDOut, &TempSetpoint, Kp, Ki, Kd, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&RPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&RPID, 2);
  PID_SetOutputLimits(&RPID, Out_Min, Out_Max);
}

void PWM_Gen(uint8_t channel, double duty)
{
  uint32_t period = (SystemCoreClock / 6000) - 1;
  htim1.Instance->ARR = period;
  switch (channel)
  {
  case 1:
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (duty == 0)
    {
      htim1.Instance->CCR1 = 0;
    }
    else
    {
      htim1.Instance->CCR1 = (uint32_t)((period * duty) / 100);
    }
    break;
  case 2:
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    if (duty == 0)
    {
      htim1.Instance->CCR2 = 0;
    }
    else
    {
      htim1.Instance->CCR2 = (uint32_t)((period * duty) / 100);
    }
    break;
  case 0:
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Stop(&htim1);
    break;
  default:
    break;
  }
}

void I2C_Scan()
{
  while (HAL_I2C_IsDeviceReady(&hi2c2, 0x36 << 1, 2, 10) != HAL_OK)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(80);
  }
}
void print_get()
{
  sprintf(USART_Long_Tx_Buffer, "000,000,00000,%4.1f,%4.1f,%4.1f,%3.0f,%3.0f,0x0%1.0f", Kp, Ki, Kd, (float)Out_Max, (float)Out_Min, (float)Self_Address);
  USART_Long_Tx_Buffer[41] = '\n';
  for (int i = 0; i < 8; i++)
  {
    HAL_UART_Transmit(&huart2, USART_Long_Tx_Buffer, sizeof(USART_Long_Tx_Buffer), 100);
  }
  memset(USART_Long_Tx_Buffer, 0, sizeof(USART_Long_Tx_Buffer));
  HAL_UART_Receive_IT(&huart2, &buffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  if (huart->Instance == USART2)
  {
    switch (buffer)
    {
    case 'r':
      buffer = 10;
      Debug_Flag = 1;
      HAL_UART_Receive_IT(&huart2, &buffer, 1);
      break;
    case 's':
      buffer = 10;
      Debug_Flag = 0;
      HAL_UART_Receive_IT(&huart2, &buffer, 1);
      break;
    case 'g':
      Get_Flag = 1;
      print_get();
      break;
    case 'a':
      Data_To_Flash[6] = Data_To_Flash[6] + 1;
      buffer = 'j';
      Flash_Write_Data(Page_Address, (uint32_t *)Data_To_Flash, 8);
      Flash_Read_Data(Page_Address, (uint32_t *)Rx_Data, 8);
      HAL_UART_Receive_IT(&huart2, &buffer, 1);
      break;
    case 'b':
      Data_To_Flash[6] = Data_To_Flash[6] - 1;
      buffer = 'j';
      Flash_Write_Data(Page_Address, (uint32_t *)Data_To_Flash, 8);
      Flash_Read_Data(Page_Address, (uint32_t *)Rx_Data, 8);
      HAL_UART_Receive_IT(&huart2, &buffer, 1);
      break;
    default:
      if (buffer != 10)
      {
        USART_Rx_Buffer[rx_index++] = buffer;
      }
      else if (buffer == 10)
      {
        if (rx_index > 8)
        {
          sscanf(USART_Rx_Buffer, "%f,%f,%f,%f,%f,%f", &Data_To_Flash[0], &Data_To_Flash[1], &Data_To_Flash[2], &Data_To_Flash[3], &Data_To_Flash[4], &Data_To_Flash[5]);
          Flash_Write_Data(Page_Address, (uint32_t *)Data_To_Flash, 8);
          Flash_Read_Data(Page_Address, (uint32_t *)Rx_Data, 8);
          Kp = Rx_Data[0];
          Ki = Rx_Data[1];
          Kd = Rx_Data[2];
          Out_Max = Rx_Data[3];
          Out_Min = Rx_Data[4];
          Self_Address = (uint8_t)Rx_Data[5];
          hi2c1.Init.OwnAddress1 = Self_Address << 1;
          PID_Setup();
          memset(USART_Rx_Buffer, 0, sizeof(USART_Rx_Buffer));
        }
        rx_index = 0;
      }
      HAL_UART_Receive_IT(&huart2, &buffer, 1);
      break;
    }
  }
}

float Infinite_Angle()
{
  prev_angle = angle_raw;
  angle_raw = (float)AS5600_GetAngle();
  if (angle_raw - prev_angle > 250.0)
  {
    angle_diff += 360.0;
  }
  else if (angle_raw - prev_angle < -250.0)
  {
    angle_diff -= 360.0;
  }

  return angle_raw - angle_diff;
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
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  // BUTTON_Init(&Mode_Btn, USER_BTN_GPIO_Port, USER_BTN_Pin);
  // uint16_t Time_Now = 0;
  // while (Time_Now < 1500)
  // {
  //   Time_Now = HAL_GetTick();
  //   if (BUTTON_Read(&Mode_Btn) == 2)
  //   {
  //     Angle_Calib_Check = 1;
  //     break;
  //   }
  //   else
  //   {
  //     Angle_Calib_Check = 0;
  //   }
  // }
  /* USER CODE BEGIN 2 */
  // Flash_Read_Data(Page_Address, (uint32_t *)Rx_Data, 7);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart2, &buffer, 1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

  Flash_Read_Data(Page_Address, (uint32_t *)Rx_Data, 7);
  Kp = Rx_Data[0];
  Ki = Rx_Data[1];
  Kd = Rx_Data[2];
  Out_Max = Rx_Data[3];
  Out_Min = Rx_Data[4];
  Self_Address = (uint8_t)Rx_Data[5];

  for (uint8_t i = 0; i < 7; i++)
  {
    Data_To_Flash[i] = Rx_Data[i];
  }
  

  PID_Setup();

  // fucntion to scan i2c device
  I2C_Scan();
  MX_I2C1_Init();
  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  // PWM_Gen(1, 0);
  // PWM_Gen(2, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (Return_Connected())
    {
      TempSetpoint = (double)Return_Data();
    }
    else
    {
      TempSetpoint = AS5600_1_AInit;
    }
   
    Angle_Temp = (int16_t)(Infinite_Angle() - Rx_Data[6]);
    
    PID_Compute(&RPID);
    if (PIDOut < 0)
    {
      PIDOut = -PIDOut;
      PWM_Gen(1, PIDOut);
      PWM_Gen(2, 0);
    }
    else
    {
      PWM_Gen(1, 0);
      PWM_Gen(2, PIDOut);
    }
    Angle_Temp = (int16_t)(Infinite_Angle() - Rx_Data[6]);
    if (Debug_Flag)
    {
      sprintf(USART_Short_Tx_Buffer, "%3.0f,%3.0f,%2.3f", Angle_Temp, (float)TempSetpoint, (float)PIDOut);
      USART_Short_Tx_Buffer[13] = '\n';
      if (USART_Short_Tx_Buffer[3] == ',' && USART_Short_Tx_Buffer[7] == ',')
      {
        HAL_UART_Transmit(&huart2, USART_Short_Tx_Buffer, 14, 100);
      }
      else
      {
        memset(USART_Short_Tx_Buffer, '/0', 14);
      }
      
    }
    Angle_Temp = (int16_t)(Infinite_Angle() - Rx_Data[6]);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = Self_Address << 1;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
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
