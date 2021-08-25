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
#include <stdio.h>
#include <string.h>
#include "HelperFcns.h"
#include "FIFO.h"
#include "MS5611.h"
#include "UBLOXM8N.h"
#include "ICM20948.h"
#include "HC06.h"
#include "w25qxx.h"
#include "Buzzer.h"
#include "FlashMemory.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _State_FSM
{
	Initialize_State,
	NonCalibrated_State,
	Calibrated_State,
	DataLogging_State,
} State_FSM;
typedef enum _Event_FSM
{
	Minus100_to_Zero_Event,
	Zero_to_Plus100_Event,
	Plus100_to_Zero_Event,
	Zero_to_Minus100_Event,
	Minus100_to_Plus100_Event,
	Plus100_to_Minus100_Event,
	Nothing_Event,
	BLT_Set_UsrData_Event,
	BLT_Get_UsrData_Event,
	BLT_Get_SensorData_Event,
	BLT_EraseMemory,
} Event_FSM;
typedef enum _SwitchPosition //switch position RC transceiver
{
	Minus100,
	Zero,
	Plus100,
	NonValid,
} SwitchPosition;
typedef struct _RCReceiverSignal
{
	SwitchPosition current_SwitchPosition;
	SwitchPosition old_SwitchPosition;
	uint32_t PWM_f; //frequency reading
	float PWM_DC; //duty cycle reading
	uint32_t TIM_IC_Val1; //used for frequency and DC calculation
	uint32_t TIM_IC_Val2; //used for frequency and DC calculation
	float PWM_DC_Thre; //Threshold for PWM change detection
	float PWM_DC_Minus100; //predefined DC value for -100 switch position
	float PWM_DC_Plus100; //predefined DC value for +100 switch position
	float PWM_DC_Zero; //predefined DC value for zero switch position
} RCReceiverSignal;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//MS5611 instance
MS5611_DEVICE MS5611_Dev;
//UBLOXM8N instance
UBLOXM8N_DEVICE UBLOXM8N_Dev;
//ICM20948 instance
ICM20948_DEVICE ICM20948_Dev;
//SPI FLASH instance
W25Qxx_DEVICE W25Q64V_Dev;
//HC08 instance
HC06_DEVICE HC06_Dev;
//Buzzer
BUZZER_DEVICE BUZZER_Dev;

//FSM variables
State_FSM state_FSM; //stores current FSM state
Event_FSM event_FSM; //stores new FSM event
fifo_t queue_FSM; //stores FSm events

//Other global variables
uint8_t dataBuffer[132]; //Data buffer (holds all sensor data from 1 sampling period: 132 bytes)
uint32_t nSamples; //variable that stores total samples during the full recording
uint8_t deviceName[50]; //stores the device name. Max 50 characters
RCReceiverSignal RCReceiverS; //variable that stores FM receiver signal info and TIM_IC intermediate values
uint8_t HC06_Rx_CmdBuffer[6]; //stores commands coming via usart through the bluetooth module
uint8_t HC06_Tx_UsrData[57]; //stores general device information data to transmit
uint8_t HC06_Tx_SensorData[4096]; //stores 1 4K sector of sensor data from memory to transmit

//Debug variables to count program cycles
uint32_t cycles1; uint32_t cycles2; uint32_t cyclesDiff;
uint8_t testVar;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void getNextEvent(void);
void getDeviceInfo(void);
void setDeviceInfo(void);

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //Initialize FSMs & other global variables
  state_FSM = Initialize_State;
  queue_FSM = fifo_create(10, sizeof(Event_FSM));

  RCReceiverS.PWM_DC_Minus100 = 12.93;
  RCReceiverS.PWM_DC_Plus100 = 7.33;
  RCReceiverS.PWM_DC_Zero = 10.13;
  RCReceiverS.PWM_DC_Thre = 0.7;

  //Initialize CS lines for SPI communications
  HAL_GPIO_WritePin(W25Q64_SPI1_CS_GPIO_Port, W25Q64_SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MS5611_SPI2_CS_GPIO_Port, MS5611_SPI2_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ICM20948_SPI3_CS_GPIO_Port, ICM20948_SPI3_CS_Pin, GPIO_PIN_SET);

  //Initialize user indicators
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); //turn off led
  BUZZER_Init(&BUZZER_Dev, Buzzer_PWR_CTRL_GPIO_Port, Buzzer_PWR_CTRL_Pin, &htim3);

  //Initialize MS5611
  MS5611_Init_Device(&MS5611_Dev, &hspi2, MS5611_SPI2_CS_GPIO_Port, MS5611_SPI2_CS_Pin);
  MS5611_Set_Config(&MS5611_Dev, OSR_4096, OSR_4096);

  //Initialize ICM20948
  ICM20948_Init_Device(&ICM20948_Dev, &hspi3, ICM20948_SPI3_CS_GPIO_Port ,ICM20948_SPI3_CS_Pin);
  ICM20948_Set_Sensors_Default_Config(&ICM20948_Dev);

  //Initialize UBLOXM8N
  UBLOXM8N_Init_Device(&UBLOXM8N_Dev, &huart2);

  //Initialize W25Q64V
  W25qxx_Init_Device(&W25Q64V_Dev, &hspi1, W25Q64_SPI1_CS_GPIO_Port, W25Q64_SPI1_CS_Pin);

  //Initialize HC06
  HC06_Init_Device(&HC06_Dev, &huart1, HC06_PWR_CTRL_GPIO_Port, HC06_PWR_CTRL_Pin);
  //HC06_ConfigBaudRate(&HC06_Dev, BR_115200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  getNextEvent();
	  switch (state_FSM)
	  {
	  case Initialize_State:
		  //Read device info stored in external flash and store it in MCU RAM: name and nSamples
		  getDeviceInfo();
		  //Initialization buzzer sound 2beeps + long beep
		  BUZZER_nBeeps_Blck(&BUZZER_Dev, 2);
		  BUZZER_TurnON_Blck(&BUZZER_Dev, 500);
		  //Start measuring PWM signal from receiver (tim4 ic mode)
		  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
		  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
		  //BLE power on and enable idle line interrupt on its usart
		  HC06_PowerON(&HC06_Dev);
		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		  //HAL_UART_Receive_IT(&huart1, &HC06_Rx_CmdBuffer[0], 6);
		  //Change FSM state
		  state_FSM = NonCalibrated_State;
		  break;
	  case NonCalibrated_State:
		  switch (event_FSM)
		  {
		  case Minus100_to_Zero_Event: //User switch from A to B --> turn off bluetooth and start calibration
			  //Turn off BLE
			  HC06_PowerOFF(&HC06_Dev);
			  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
			  //Start led blinking
			  __HAL_TIM_SET_PRESCALER(&htim9,207); //200ms time
			  HAL_TIM_Base_Start_IT(&htim9);
			  //Beep 3 times
			  BUZZER_nBeeps_Blck(&BUZZER_Dev, 3);
			  //Perform calibration operations
			  MS5611_Calibrate_Altitude(&MS5611_Dev, 5);
			  //Stop led blinking and turn it off
			  HAL_TIM_Base_Stop_IT(&htim9);
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			  //Beep 3 times
			  BUZZER_nBeeps_Blck(&BUZZER_Dev, 3);
			  //Change FSM state
			  state_FSM = Calibrated_State;
			  break;
		  case BLT_Set_UsrData_Event: //Set device information request from PC app

			  break;
		  case BLT_Get_UsrData_Event: //Get device information request from PC app
			  HAL_UART_AbortReceive_IT(&huart1);
			  HC06_Tx_UsrData[0] = W25Q64V_Dev.isErased; //1 = is erased
			  memcpy(&HC06_Tx_UsrData[1],&W25Q64V_Dev.written4KSectorCount,2); //max 4kSectorCount = 2048 (only 2 bytes required)
			  memcpy(&HC06_Tx_UsrData[3],&deviceName[0],50); //device name
			  memcpy(&HC06_Tx_UsrData[53],&nSamples,4); //number of recorded samples
			  HAL_UART_Transmit(&huart1, HC06_Tx_UsrData, 57, 100);
			  HAL_UART_Receive_IT(&huart1, &HC06_Rx_CmdBuffer[0], 6);
			  break;
		  case BLT_Get_SensorData_Event: //Get sensor data request from PC app
			  HAL_UART_AbortReceive_IT(&huart1);
			  uint32_t i = 0;
			  //uint32_t j = 0;
			  for (i=0; i<W25Q64V_Dev.written4KSectorCount; i++)
			  {
				  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_RESET);
				  W25qxx_ReadSector(&W25Q64V_Dev, &HC06_Tx_SensorData[0], i, 0, 4096);
				  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_SET);
				  HAL_UART_Transmit(&huart1, &HC06_Tx_SensorData[0], 4096, 1000);
				  HAL_Delay(220);
//				  for (j=0; j<8; j++) //the 4096 byte sector is transferred in 8 packets of 512 bytes
//				  {
//					  HAL_UART_Transmit(&huart1, &HC06_Tx_SensorData[j*512], 512, 1000);
//					  HAL_Delay(20);
//				  }
//				  HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_SET);
			  }
			  HAL_UART_Receive_IT(&huart1, &HC06_Rx_CmdBuffer[0], 6);
			  break;
		  case BLT_EraseMemory:
			  W25qxx_EraseChip(&W25Q64V_Dev);
			  W25qxx_IsErased(&W25Q64V_Dev);
			  nSamples = 0;
			  uint8_t n = 0;
			  for (n=0; n<50; n++)
			  {
				  deviceName[n] = ' ';
			  }
			  break;
			  setDeviceInfo();
		  default:
			  break;
		  }
		  break;
	  case Calibrated_State:
		  switch (event_FSM)
		  {
		  case Zero_to_Plus100_Event: // --> erase flash and start recording sensors
			  //Stop beeping if is running and beep 2 times
			  BUZZER_Beep_Stop(&BUZZER_Dev);
			  BUZZER_nBeeps_Blck(&BUZZER_Dev, 2);
			  //Start led blinking
			  __HAL_TIM_SET_PRESCALER(&htim9,207); //200ms time
			  HAL_TIM_Base_Start_IT(&htim9);
			  //Erase flash memory if not empty
			  if (!W25qxx_IsErased(&W25Q64V_Dev))
			  {
				  W25qxx_EraseWrittenMemory(&W25Q64V_Dev);
			  }
			  //Clear the samples counter
			  nSamples = 0; //in MCU RAM
			  setDeviceInfo(); //in flash
			  //Stop led blinking and turn it off
			  HAL_TIM_Base_Stop_IT(&htim9);
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			  //Beep 2 times
			  BUZZER_nBeeps_Blck(&BUZZER_Dev, 2);
			  //Start acquisition timer
			  HAL_TIM_Base_Start_IT(&htim11);
			  //Change FSM state
			  state_FSM = DataLogging_State;
			  break;
		  case Zero_to_Minus100_Event: // --> go back to non calibrated state and turn bluetooth on
			  //Stop buzzer beeping if running
			  BUZZER_Beep_Stop(&BUZZER_Dev);
			  BUZZER_nBeeps_Blck(&BUZZER_Dev, 2);
			  BUZZER_TurnON_Blck(&BUZZER_Dev, 500);
			  //Turn on BLE
			  HC06_PowerON(&HC06_Dev);
			  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
			  HAL_UART_Receive_IT(&huart1, &HC06_Rx_CmdBuffer[0], 6);
			  //Change FSM state
			  state_FSM = NonCalibrated_State;
			  break;
		  default: // --> by default keep looking if GPS got a fix position..
			  if ((UBLOXM8N_Dev.navResult == Fix)&&(BUZZER_Dev.itState)) //if GPS is fix and beep has not been stopped
			  {
				  BUZZER_Beep_Stop(&BUZZER_Dev);
			  }
			  else if ((UBLOXM8N_Dev.navResult != Fix)&&(!BUZZER_Dev.itState)) //if GPS is not fix and beep has not started
			  {
				  BUZZER_Beep_Start(&BUZZER_Dev, 200, 1000);
			  }
		  }
		  break;
	  case DataLogging_State:
		  switch (event_FSM)
		  {
		  case Plus100_to_Zero_Event:
			  //Stop acquisition timer
			  HAL_TIM_Base_Stop_IT(&htim11);
			  //Store device info in external flash and MCU RAM
			  setDeviceInfo(); //stores nSamples saved (and device name)
			  //Beep 3 times
			  BUZZER_nBeeps_Blck(&BUZZER_Dev, 3);
			  //Change FSM state
			  state_FSM = Calibrated_State;
			  break;
		  default:
			  break;
		  }
		  break;
	  default:
		  break;
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 136;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 34000;
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 518;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 15;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 42499;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart1.Init.BaudRate = 921600;
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
  huart2.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|ICM20948_SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, W25Q64_SPI1_CS_Pin|MS5611_SPI2_CS_Pin|Buzzer_PWR_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Test_Pin|HC06_PWR_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin ICM20948_SPI3_CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|ICM20948_SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : W25Q64_SPI1_CS_Pin MS5611_SPI2_CS_Pin Buzzer_PWR_CTRL_Pin */
  GPIO_InitStruct.Pin = W25Q64_SPI1_CS_Pin|MS5611_SPI2_CS_Pin|Buzzer_PWR_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Test_Pin HC06_PWR_CTRL_Pin */
  GPIO_InitStruct.Pin = Test_Pin|HC06_PWR_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Helper functions
void getNextEvent(void)
{
	if(!fifo_get(queue_FSM, &event_FSM)) //if no event available
	{
		event_FSM = Nothing_Event;
	}
}
void getDeviceInfo(void)
{
	//Read device name
	W25qxx_ReadBytes(&W25Q64V_Dev, &deviceName[0], 0x7FF000, 50); //address is first byte of last 4Ksector
	//Read nSamples stored
	uint8_t nSamplesU8[4];
	W25qxx_ReadBytes(&W25Q64V_Dev, &nSamplesU8[0], 0x7FF032, 4);
	nSamples = (nSamplesU8[0] | nSamplesU8[1]<<8 | nSamplesU8[2]<<16 | nSamplesU8[3]<<24);
}
void setDeviceInfo(void)
{
	//Erase last sector where this type of info is stored
	W25qxx_Erase4KSector(&W25Q64V_Dev, 2047);
	//Write device name and nSamples
	uint8_t buffInfo[54];
	memcpy(&buffInfo[0],&deviceName[0],50); //device name
	memcpy(&buffInfo[50],&nSamples,4); //nSamples
	W25qxx_WritePage(&W25Q64V_Dev, &buffInfo[0], 32752, 0, 54); //first page of last sector
}

//Interrupt callbacks
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11) //acquisition trigger
	{
		//HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_RESET);
		//Read IMU
		ICM20948_Get_AGMT_RawData(&ICM20948_Dev);
		//Start conversion or read conversion result barometer
		if (MS5611_Dev.current_ADC_Var == Pressure)
		{
			MS5611_Read_ADC(&MS5611_Dev);
			MS5611_Init_ADC_Conv(&MS5611_Dev, Temperature);
			MS5611_Get_DeltaAltitude(&MS5611_Dev);
		}
		else if (MS5611_Dev.current_ADC_Var == Temperature)
		{
			MS5611_Read_ADC(&MS5611_Dev);
			MS5611_Init_ADC_Conv(&MS5611_Dev, Pressure);
		}
		//Fill data buffer and write external flash
		float2uint8(MS5611_Dev.deltaH, &dataBuffer[0], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.accelData.x, &dataBuffer[4], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.accelData.y, &dataBuffer[8], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.accelData.z, &dataBuffer[12], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.gyroData.x, &dataBuffer[16], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.gyroData.y, &dataBuffer[20], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.gyroData.z, &dataBuffer[24], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.magnetoData.x, &dataBuffer[28], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.magnetoData.y, &dataBuffer[32], 1);
		float2uint8(ICM20948_Dev.sensorsDataProc.magnetoData.z, &dataBuffer[36], 1);
		memcpy(&dataBuffer[40],UBLOXM8N_Dev.navData,92);
		W25Q64_WriteBytes(&W25Q64V_Dev, &dataBuffer[0], 132);
		//Increment samples counter
		nSamples++;
		//HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_SET);
	}
	if (htim == &htim9) //status led
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
	if (htim == &htim3) //buzzer
	{
		if (BUZZER_Dev.state) //if buzzer is ON
		{
			BUZZER_OFF(&BUZZER_Dev);
			__HAL_TIM_SET_PRESCALER(&htim3,BUZZER_Dev.miliON);
		}
		else //if buzzer is OFF
		{
			BUZZER_ON(&BUZZER_Dev);
			__HAL_TIM_SET_PRESCALER(&htim3,BUZZER_Dev.miliOFF);
		}
	}
}
void UART_IdleLineCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2) //GNSS uart idle line detection
	{
		HAL_UART_Receive_IT(&huart2, &(UBLOXM8N_Dev.pvtMsg[0]), 100); //se reciben bloques de 10 bits
	}
	if (huart == &huart1) //Bluetooth uart idle line detection
	{
		HAL_UART_Receive_IT(&huart1, &HC06_Rx_CmdBuffer[0], 6); //se reciben bloques de 6 bits
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2) //GNSS uart completed PVT message transmission
	{
		UBLOXM8N_processPVTmsg(&UBLOXM8N_Dev);
	}
	if (huart == &huart1) //HC06 full message received, generate events
	{
		if (strncmp((char*)&HC06_Rx_CmdBuffer[0],"HC06-",5) == 0) //checks first 5 characters
		{
			Event_FSM eventTemp;
			switch((char)HC06_Rx_CmdBuffer[5])
			{
			case '0': //Get device info
				eventTemp = BLT_Get_UsrData_Event;
				break;
			case '1': //Set device info
				eventTemp = BLT_Set_UsrData_Event;
				break;
			case '2': //Calib sensors
				//eventTemp = Nothing_Event;
				break;
			case '3': //Erase memory
				eventTemp = BLT_EraseMemory;
				break;
			case '4': //Read memory
				eventTemp = BLT_Get_SensorData_Event;
				break;
			default:
				break;
			}
			fifo_add(queue_FSM, &eventTemp);
		}
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4) //FM receiver signal PWM measurement
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) //rising edge interrupt
		{
			//Calculate DC and frequency
			RCReceiverS.TIM_IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //rising edge val
			if (RCReceiverS.TIM_IC_Val1 != 0)
			{
				RCReceiverS.TIM_IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); //falling edge val
				RCReceiverS.PWM_DC = (float)RCReceiverS.TIM_IC_Val2*100/RCReceiverS.TIM_IC_Val1;
				RCReceiverS.PWM_f = HAL_RCC_GetPCLK1Freq()/RCReceiverS.TIM_IC_Val1; //tim clock is 2xPCLK1
			}
			else
			{
				RCReceiverS.PWM_DC = 0;
				RCReceiverS.PWM_f = 0;
			}
			//Classify the value
			if ((RCReceiverS.PWM_DC > (RCReceiverS.PWM_DC_Minus100-RCReceiverS.PWM_DC_Thre)) && (RCReceiverS.PWM_DC < (RCReceiverS.PWM_DC_Minus100+RCReceiverS.PWM_DC_Thre)))
			{
				RCReceiverS.current_SwitchPosition = Minus100;
			}
			else if ((RCReceiverS.PWM_DC > (RCReceiverS.PWM_DC_Zero-RCReceiverS.PWM_DC_Thre)) && (RCReceiverS.PWM_DC < (RCReceiverS.PWM_DC_Zero+RCReceiverS.PWM_DC_Thre)))
			{
				RCReceiverS.current_SwitchPosition = Zero;
			}
			else if ((RCReceiverS.PWM_DC > (RCReceiverS.PWM_DC_Plus100-RCReceiverS.PWM_DC_Thre)) && (RCReceiverS.PWM_DC < (RCReceiverS.PWM_DC_Plus100+RCReceiverS.PWM_DC_Thre)))
			{
				RCReceiverS.current_SwitchPosition = Plus100;
			}
			else
			{
				RCReceiverS.current_SwitchPosition = NonValid;
			}

			//Detect change and generate FSM events
			if (RCReceiverS.current_SwitchPosition != RCReceiverS.old_SwitchPosition)
			{
				Event_FSM eventTemp;
				switch (RCReceiverS.old_SwitchPosition)
				{
				case Minus100:
					switch (RCReceiverS.current_SwitchPosition)
					{
					case Plus100:
						eventTemp = Minus100_to_Plus100_Event;
						break;
					case Zero:
						eventTemp = Minus100_to_Zero_Event;
						break;
					case NonValid:
						break;
					default:
						break;
					}
					break;
				case Zero:
					switch (RCReceiverS.current_SwitchPosition)
					{
					case Plus100:
						eventTemp = Zero_to_Plus100_Event;
						break;
					case Minus100:
						eventTemp = Zero_to_Minus100_Event;
						break;
					case NonValid:
						break;
					default:
						break;
					}
					break;
				case Plus100:
					switch (RCReceiverS.current_SwitchPosition)
					{
					case Minus100:
						eventTemp = Plus100_to_Minus100_Event;
						break;
					case Zero:
						eventTemp = Plus100_to_Zero_Event;
						break;
					case NonValid:
						break;
					default:
						break;
					}
					break;
				case NonValid:
					break;
				}
				fifo_add(queue_FSM, &eventTemp);
				RCReceiverS.old_SwitchPosition = RCReceiverS.current_SwitchPosition;
			}
		}
	}
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
