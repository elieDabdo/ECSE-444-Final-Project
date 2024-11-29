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
#include <stdio.h>
#include "arm_math.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ARM_MATH_CM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId ButtonTaskHandle;
osThreadId TerminalTaskHandle;
osThreadId SensorTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartButtonTask(void const * argument);
void StartTerminalTask(void const * argument);
void StartSensorTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RAD_TO_DEG 57.2957795
#define ROWS 7
#define COLS 76

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

char button_status = 0;
int sensor_number = 0;

int16_t xyz_accel[] = {0,0,0};

// gyroscope
int16_t xyz_gyro[] = {0,0,0};

// pressure
int16_t pressure = 0;

// magnometer
int16_t xyz_mag[] = {0,0,0};

int arrow_position = 1;
int difficultyLevel = 1;

int car_position = 3; //row
int car_column = 0;
osMutexId accelDataMutex;
float roll = 0.0f;

static uint32_t seed = 1;
char board[ROWS][COLS];

int obstacleGenerationFrequency = 10;
int terminalTaskCounter = 10;

int movingCounter = 10;
int gameOver = 0;

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  BSP_PSENSOR_Init();
  BSP_MAGNETO_Init();

//  int16_t *ptr = &xyz;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexDef(accelDataMutex);         // Define the mutex
  accelDataMutex = osMutexCreate(osMutex(accelDataMutex)); // Create the mutex
  if (accelDataMutex == NULL) {
      printf("Failed to create accelDataMutex\n\r");
      Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of ButtonTask */
  osThreadDef(ButtonTask, StartButtonTask, osPriorityHigh, 0, 128);
  ButtonTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

  /* definition and creation of TerminalTask */
  osThreadDef(TerminalTask, StartTerminalTask, osPriorityAboveNormal, 0, 200);
  TerminalTaskHandle = osThreadCreate(osThread(TerminalTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityAboveNormal, 0, 200);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void printAcceleration(int16_t *accelData) {
    printf("Acceleration: X: %d, Y: %d, Z: %d\n\r", accelData[0], accelData[1], accelData[2]);
}

void printGyroscope(int16_t *gyroData) {
    printf("Gyroscope: X: %d, Y: %d, Z: %d\n\r", gyroData[0], gyroData[1], gyroData[2]);
}

void printMagnetometer(int16_t *magData) {
    printf("Magnetometer: X: %d, Y: %d, Z: %d\n\r", magData[0], magData[1], magData[2]);
}

void srand(uint32_t new_seed) {
    seed = new_seed;
}

uint32_t rand(void) {
    seed = (1103515245 * seed + 12345) % (1 << 31);
    return seed;
}

uint32_t getRandomNumber0to6(void) {
    uint32_t random_uint32 = rand();
    // Scale to range 0-6
    return random_uint32 % 7;
}

void generateObstacles(int level){
	// Medium Level, 2 obstacles
	if (level == 1) {
		int firstRow = getRandomNumber0to6();
		int secondRow = getRandomNumber0to6();
		board[firstRow][COLS -1] = 'O';
		board[secondRow][COLS -1] = 'O';
	}
	if (level == 2) {
			int firstRow = getRandomNumber0to6();
			int secondRow = getRandomNumber0to6();
			int thirdRow = getRandomNumber0to6();
			board[firstRow][COLS -1] = 'O';
			board[secondRow][COLS -1] = 'O';
			board[thirdRow][COLS -1] = 'O';

		}
}

void initializeBoard() {
    // Fill the board with empty spaces
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            board[i][j] = ' '; // Empty space
        }
    }
}

// Generates board from game data, return 1 if game over, 0 if nothing, 2 if win
int generateBoard() {
    // Clear previous positions
	if (car_column >= COLS-4){
			return 2;
		}
	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {

			if (board[i][j] == 'O') {
				// Move the '-' to the left if possible
				board[i][j] = ' ';
				if (j > 2) {
					board[i][j - 3] = 'O';
					// Game over
					if (car_position == i && (car_column == (j - 3)|| car_column + 1 == (j-3) || car_column + 2 == (j-3) || car_column + 3 == (j-3))){
						return 1;
					}
				}
			} else {
				// Clear any other characters
				board[i][j] = ' ';
			}
	    }
	 }
    // Place the car on the specified row
    strncpy(&board[car_position][car_column], "o/=\o", 4); // Put the car at column 0
    return 0;
}

void displayBoard() {
    // Top border
    printf("+------------------------------------------------------------------------------+\n\r");
    for (int i = 0; i < ROWS; i++) {
        printf("|"); // Left border
        for (int j = 0; j < COLS; j++) {
            printf("%c", board[i][j]);
        }
        printf("|\n\r"); // Right border
        if (i < ROWS - 1) {
            // Horizontal row divider
            printf("|------------------------------------------------------------------------------|\n\r");
        }
    }
    // Bottom border
    printf("+------------------------------------------------------------------------------+\n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
	printf("\n\r");
	printf("\n\r");
}

void printGameOver() {
    printf("\n\r");
    printf(" GGGGG  AAAAA  M     M  EEEEE    OOO  V   V EEEEE RRRR  \n\r");
    printf("G       A   A  MM   MM  E       O   O V   V E     R   R \n\r");
    printf("G  GG   AAAAA  M M M M  EEEE    O   O V   V EEEE  RRRR  \n\r");
    printf("G   G   A   A  M  M  M  E       O   O V   V E     R  R  \n\r");
    printf(" GGGG   A   A  M     M  EEEEE    OOO   VVV  EEEEE R   R \n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
    printf("\n\r");
}

void printWinner() {
    printf("\n\r");
    printf(" W   W  III  N   N  N   N  EEEEE  RRRR    \n\r");
    printf(" W   W   I   NN  N  NN  N  E      R   R   \n\r");
    printf(" W W W   I   N N N  N N N  EEEE   RRRR    \n\r");
    printf(" WW WW   I   N  NN  N  NN  E      R  R    \n\r");
    printf(" W   W  III  N   N  N   N  EEEEE  R   R   \n\r");
    printf("\n\r");
    printf("\n\r");
	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
}

void displayMenu(){
	if (arrow_position == 1){
		printf("SELECT A DIFFICULTY:");
		printf("\n\r");
		printf("MEDIUM   <-- \n\r");
		printf("\n\r");
		printf("HARD \n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
	}
	else if(arrow_position == 2){
		printf("SELECT A DIFFICULTY:");
		printf("\n\r");
		printf("MEDIUM \n\r");
		printf("\n\r");
		printf("HARD     <-- \n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
		printf("\n\r");
	}
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartButtonTask */
/**
  * @brief  Function implementing the ButtonTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);

    //update position of the menu arrow
    osMutexWait(accelDataMutex, osWaitForever);
	if (roll > 20.0f && arrow_position <2){
		arrow_position++;
	}
	if (roll < -20.0f && arrow_position >1){
		arrow_position--;;
	}
	osMutexRelease(accelDataMutex);

	//suspends the start menu task if the button is pressed to let the game begin
    button_status = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
    if (button_status ==0) {
		if (button_status ==0){
	    	difficultyLevel = arrow_position;
			vTaskResume(TerminalTaskHandle); // resumes the game task
			vTaskSuspend(ButtonTaskHandle); // Suspend the menu task (NULL means current task)
		}
    }

    displayMenu();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTerminalTask */
/**
* @brief Function implementing the TerminalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTerminalTask */
void StartTerminalTask(void const * argument)
{
  /* USER CODE BEGIN StartTerminalTask */
	vTaskSuspend(TerminalTaskHandle); // Suspends the game task until it is resumed by the menu task
  /* Infinite loop */
  for(;;)
  {
    osDelay(300);

    if (terminalTaskCounter == obstacleGenerationFrequency){
        	generateObstacles(difficultyLevel);
        }

    if (terminalTaskCounter== movingCounter){
    	car_column = car_column + 5;
    }
    if (terminalTaskCounter == 0) {
    	terminalTaskCounter = obstacleGenerationFrequency;
    } else {
    	terminalTaskCounter = terminalTaskCounter - 1;
    }

    osMutexWait(accelDataMutex, osWaitForever);
    if (roll > 20.0f && car_position <6){
		car_position = car_position + 1;
	}
	if (roll < -20.0f && car_position >0){
		car_position = car_position - 1;
	}
    osMutexRelease(accelDataMutex);

    //printf("%d\n\r", getRandomNumber0to6());

//    printf("____________________________________________________________________________\n\r");
//    for (int i=0; i<=6; i++){
//    	if (i == car_position){
//    		printf("o/=\o\n\r");
//    	} else {
//    		printf("\n\r");
//    	}
//
//    }
//    printf("____________________________________________________________________________\n\r");
//    printf("\n\r");
//    printf("\n\r");
    int gameStatus = generateBoard();
    if (gameStatus==1){
    	printGameOver();
    	vTaskDelay(pdMS_TO_TICKS(1000)); //tasks wait for 1 second before going back to menu
    	vTaskResume(ButtonTaskHandle); // resumes the menu task
		vTaskSuspend(TerminalTaskHandle); // Suspend the game task (NULL means current task)
    }
    if (gameStatus ==2){
    	printWinner();
    	vTaskDelay(pdMS_TO_TICKS(1000)); //tasks wait for 1 second before going back to menu
    	vTaskResume(ButtonTaskHandle); // resumes the menu task
		vTaskSuspend(TerminalTaskHandle); // Suspend the game task (NULL means current task)
    }
    displayBoard();
//    if (status) {
//    	displayBoard(); }
//    } else {
//    	printGameOver();
//    	vTaskSuspend(NULL); // Suspend the current task (NULL means current task)
//    }

  }
  /* USER CODE END StartTerminalTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);

    BSP_ACCELERO_AccGetXYZ(&xyz_accel);
    int16_t ax = xyz_accel[0];
    int16_t ay = xyz_accel[1];
    int16_t az = xyz_accel[2];

    float32_t square = 0.0f;

	osMutexWait(accelDataMutex, osWaitForever);
	if (ax != 0 || ay != 0 || az != 0) {
		arm_sqrt_f32((float32_t)((ax * ax + az * az) * 57), &square);
		roll = atan2((double)ay, (double)square) * 57;
	}
    osMutexRelease(accelDataMutex);
  }
  /* USER CODE END StartSensorTask */
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
	printf("Error happened");
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
