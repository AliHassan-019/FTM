/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STEP_DELAY 50 			// Decrease delay to increase servo speed
#define SERVO_MIN_ANGLE 0   // Minimum servo angle in degrees
#define SERVO_MAX_ANGLE 180 // Maximum servo angle in degrees
#define SERVO_MIN_PULSE 25  // Corresponding pulse width for min angle (1ms)
#define SERVO_MAX_PULSE 125 // Corresponding pulse width for max angle (2ms)
#define DT_PIN GPIO_PIN_0   // Load cell  DT Pin
#define DT_PORT GPIOB       // load Cell DT  Port
#define SCK_PIN GPIO_PIN_1  // Load Cell SCK Pin
#define SCK_PORT GPIOB      // Load Cell SCK Port
#define Buffer_Size 9       // Buffer Size for receiving Data


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for Stepper_Motor */
osThreadId_t Stepper_MotorHandle;
const osThreadAttr_t Stepper_Motor_attributes = {
  .name = "Stepper_Motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Servo_Motor */
osThreadId_t Servo_MotorHandle;
const osThreadAttr_t Servo_Motor_attributes = {
  .name = "Servo_Motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Load_Cell */
osThreadId_t Load_CellHandle;
const osThreadAttr_t Load_Cell_attributes = {
  .name = "Load_Cell",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Stepper_Semaphore */
osSemaphoreId_t Stepper_SemaphoreHandle;
const osSemaphoreAttr_t Stepper_Semaphore_attributes = {
  .name = "Stepper_Semaphore"
};
/* Definitions for Servo_Semaphore */
osSemaphoreId_t Servo_SemaphoreHandle;
const osSemaphoreAttr_t Servo_Semaphore_attributes = {
  .name = "Servo_Semaphore"
};
/* Definitions for Load_Cell_Semaphore */
osSemaphoreId_t Load_Cell_SemaphoreHandle;
const osSemaphoreAttr_t Load_Cell_Semaphore_attributes = {
  .name = "Load_Cell_Semaphore"
};
/* USER CODE BEGIN PV */

uint16_t current_angle = 10;
int state = 0;
int length = 16500;
int stepper_speed = 15000;
int servo_angle = 90;
bool servo_active = false;
char buffer[24] = {0X5A,0XA5,0X07,0X82,0X10,0X00,0X00,0x00,0x00,0x00,
	0x5A,0XA5,0X0B,0X82,0X03,0X10,0X5a,0Xa5,0X01,0X00,0X01,0X01,0X00,0X00
};
bool toggle_flag = false;  // Define a flag to toggle between 0 and measured value
uint8_t rx_buffer[Buffer_Size];
uint16_t vp_address = 0x1100;  // VP address for the Transmitting
uint16_t VP_Address; 	// VP Address For Receiving
volatile int start_flag = 0;
volatile int stop_flag = 0;
volatile bool stop_servo = false;
bool movement_in_progress = false;
volatile bool servo_running = false;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void Stepper_Motor_Init(void *argument);
void Servo_Motor_Init(void *argument);
void Load_Cell_Init(void *argument);

/* USER CODE BEGIN PFP */

void move_servo_to_angle(uint16_t target_angle, uint16_t speed);
void SendDataToLCD(int value);
void DisplayGraph(int reading);
void DisplayNumber(uint16_t vp__address, int values);
void Start_Process(void);
void Stop_Process(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t tare =7933904;
float knownOriginal = 100000;  // in milli gram
float knownHX711 = 45539;
int force;


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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
		HAL_Delay(500);
		
		HAL_UART_Receive_DMA(&huart2, rx_buffer, Buffer_Size);
	 //HAL_UART_Transmit_DMA(&huart2, rx_buffer, Buffer_Size);

	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	

	 HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
   osDelay(10);
   HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
   osDelay(10);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Stepper_Semaphore */
  Stepper_SemaphoreHandle = osSemaphoreNew(1, 0, &Stepper_Semaphore_attributes);

  /* creation of Servo_Semaphore */
  Servo_SemaphoreHandle = osSemaphoreNew(1, 0, &Servo_Semaphore_attributes);

  /* creation of Load_Cell_Semaphore */
  Load_Cell_SemaphoreHandle = osSemaphoreNew(1, 0, &Load_Cell_Semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Stepper_Motor */
  Stepper_MotorHandle = osThreadNew(Stepper_Motor_Init, NULL, &Stepper_Motor_attributes);

  /* creation of Servo_Motor */
  Servo_MotorHandle = osThreadNew(Servo_Motor_Init, NULL, &Servo_Motor_attributes);

  /* creation of Load_Cell */
  Load_CellHandle = osThreadNew(Load_Cell_Init, NULL, &Load_Cell_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	
	
	
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(50);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Stepper_DIr_GPIO_Port, Stepper_DIr_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCK_Pin|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : Stepper_DIr_Pin */
  GPIO_InitStruct.Pin = Stepper_DIr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Stepper_DIr_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DT_Pin */
  GPIO_InitStruct.Pin = DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCK_Pin PB3 */
  GPIO_InitStruct.Pin = SCK_Pin|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


uint16_t angle_to_pwm(uint16_t angle)
{
    return (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) + SERVO_MIN_PULSE;
}

void move_servo_to_angle(uint16_t target_angle, uint16_t speed)
{
    uint16_t target_pwm = angle_to_pwm(target_angle);
    uint16_t current_pwm = angle_to_pwm(current_angle);

    // Loop to increment/decrement the PWM until target is reached
    while (current_pwm != target_pwm)
    {
        // Immediately stop if stop_servo is true
        if (stop_servo)
        {
            // Stop the servo immediately
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            current_angle = target_angle;  // Update angle as if it reached target
            return; // Exit function immediately to stop further movement
        }

        // Update PWM to move towards target
        if (current_pwm < target_pwm)
        {
            current_pwm++;
        }
        else
        {
            current_pwm--;
        }

        // Update the PWM value for the servo
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pwm);

        // Add delay to control the speed of the movement
        osDelay(speed);  // Adjust this delay for smooth movement
    }

    // After loop, set the current angle to target angle
    current_angle = target_angle;
}



void adjust_stepper_speed(uint16_t speed)
{
    __HAL_TIM_SET_AUTORELOAD(&htim1, speed);  // Adjust the period based on the speed value
}


int32_t getHX711(void)
{
    uint32_t data = 0;
    uint32_t startTime = HAL_GetTick();
    
    // Wait for the DT pin to go low
    while (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
    {
        if (HAL_GetTick() - startTime > 200)
            return 0;  
    }
    
   
    for (int8_t len = 0; len < 24; len++)
    {
        HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
        data = data << 1;
        HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
        if (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
            data++;
    }
    
    
    data = data ^ 0x800000;
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    osDelay(1);
    
    return data;
}


	float weigh()
	{
			int32_t total = 0;
			int32_t samples = 1;
			int milligram;
			float coefficient;
			float force;

			for(uint16_t i = 0; i < samples; i++)
			{
					total += getHX711();
			}
			int32_t average = (int32_t)(total / samples);
			coefficient = knownOriginal / knownHX711;
			milligram = (int)(average - tare) * coefficient;
			//return milligram;

//			// Converting milligrams to kilograms
			float mass_kg = milligram / 1000000.0;

			// Calculate force in Newtons (F = m * g)
			force = mass_kg * 9.81f;
			force = force*1000;
			//force = force-3000;
			if(servo_angle == 120)
			{
				force = force-1000;
			}
			
			if(servo_angle == 150)
			{
				force = force-1700;
			}
			
			if (force < 0  )
			{
					force = 0;
			}

			return force;
	}
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    if (huart == &huart2)  
//    {
        
        VP_Address = (rx_buffer[4] << 8) | rx_buffer[5];
				if (VP_Address == 0x3000)
        {
            switch(rx_buffer[8])
            {
                case 0:
									start_flag = 1;
									stop_servo = false;
									move_servo_to_angle(11, STEP_DELAY);

									//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
									break;

								case 1:
									start_flag = 0;
									stop_servo = true;
									HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // Explicitly stop stepper PWM
									osSemaphoreRelease(Stepper_SemaphoreHandle); // Release semaphore for clean state
									break;
							
                
            }
					}
        
        else if (VP_Address == 0x1000)
        {
            switch(rx_buffer[8])
            {
                case 0:
									length = 16500;
//                    stepper_speed = 15000;
//										adjust_stepper_speed(stepper_speed);
                    break;  
                case 1:
									length = 22000;
//                    stepper_speed = 11000;
//										adjust_stepper_speed(stepper_speed); 
										break;							
                case 2:
									length = 25000;
									
//                   stepper_speed = 7000;
//									 adjust_stepper_speed(stepper_speed);
                   break;
            }
					}
				else if (VP_Address == 0x2000)
        {
            switch(rx_buffer[8])
            {
                case 0:
                    servo_angle = 90;
                    break;  
                case 1:
                    servo_angle = 120;
										break;							
                case 2:
                   servo_angle = 150;
                   break;
            }
					}
//        }
        
        //HAL_UART_Receive_DMA(&huart2, rx_buffer, Buffer_Size);
}




void DisplayNumber(uint16_t vp__address, int values)
{
			buffer[4] = (vp__address >> 8) & 0xFF;  // High byte of VP address
			buffer[5] = vp__address & 0xFF;         // Low byte of VP address
			buffer[6] = (values >> 8) & 0xFF;     // High byte of value
			buffer[7] = values & 0xFF; 					 // Low byte of value

			buffer[22] = (values >> 8) & 0xFF;     // High byte of value
			buffer[23] = values & 0xFF; 					 // Low byte of value
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, sizeof(buffer));
}
	

void Stop_Process(void)
{
	
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	osSemaphoreRelease(Stepper_SemaphoreHandle);
	osSemaphoreRelease(Servo_SemaphoreHandle);
	servo_active = false;
	
	
	
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Stepper_Motor_Init */
/**
  * @brief  Function implementing the Stepper_Motor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Stepper_Motor_Init */

void Stepper_Motor_Init(void *argument)
{
    for (;;)
    {
        adjust_stepper_speed(stepper_speed);

        if (start_flag)
        {
            // Attempt to acquire semaphore for stepper
            if (osSemaphoreAcquire(Stepper_SemaphoreHandle, osWaitForever) == osOK)
            {
                TIM1->CCR1 = 50; // Set duty cycle
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Start PWM

                for (int i = 0; i < length; i += 10)
                {
                    if (!start_flag) break; // Exit loop if stop flag is set
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // Stop PWM after execution

                if (start_flag) 
                {
                    servo_active = true;
                    osSemaphoreRelease(Servo_SemaphoreHandle);
                }
            }
        }
        else
        {
            // Ensure stepper stops immediately when `start_flag` is cleared
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            osSemaphoreRelease(Stepper_SemaphoreHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Allow other tasks to execute
    }
}


  /* USER CODE END 5 */

		

/* USER CODE BEGIN Header_Servo_Motor_Init */
/**
* @brief Function implementing the Servo_Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Motor_Init */
void Servo_Motor_Init(void *argument)
{
    for (;;)
    {
        if (start_flag)
        {
            stop_servo = false;
            movement_in_progress = true;
            servo_running = true; // Set servo as active

            if (osSemaphoreAcquire(Servo_SemaphoreHandle, osWaitForever) == osOK)
            {
                move_servo_to_angle(10, STEP_DELAY);
                if (!start_flag || stop_servo) goto stop_servo_cleanup;

                vTaskDelay(pdMS_TO_TICKS(1000));

                move_servo_to_angle(servo_angle, STEP_DELAY);
                if (!start_flag || stop_servo) goto stop_servo_cleanup;

                vTaskDelay(pdMS_TO_TICKS(1000));

                move_servo_to_angle(10, STEP_DELAY);
                if (!start_flag || stop_servo) goto stop_servo_cleanup;

                movement_in_progress = false;
                servo_running = false; // Mark servo operation as complete

                if (start_flag)
                {
                    servo_active = false;
                    osSemaphoreRelease(Stepper_SemaphoreHandle);
                }
            }
        }
        else
        {
        stop_servo_cleanup:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // Stop PWM
            
            movement_in_progress = false;
            servo_running = false; // Mark servo operation as complete
            osSemaphoreRelease(Stepper_SemaphoreHandle);
						stop_servo = true;
						move_servo_to_angle(10, STEP_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Allow other tasks to execute
    }
}




/* USER CODE BEGIN Header_Load_Cell_Init */
/**
* @brief Function implementing the Load_Cell thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Load_Cell_Init */
void Load_Cell_Init(void *argument)
{
  /* USER CODE BEGIN Load_Cell_Init */
  /* Infinite loop */
  for(;;)
    {
        // Activate load cell only when servo is active
        if(servo_active)
        {            
            //HAL_Delay(50);
            force = weigh();  // Reading weight from load cell
            // Send weight to LCD
           DisplayNumber(vp_address, force);
        }
        else
        {
            // If the servo is not active, always send 0
            force = 0;
          DisplayNumber(vp_address, force);
        }
        vTaskDelay(pdMS_TO_TICKS(500));  // Adjust delay as needed
    }
  /* USER CODE END Load_Cell_Init */
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
  /* User can add his own implementation to report the file
 name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
