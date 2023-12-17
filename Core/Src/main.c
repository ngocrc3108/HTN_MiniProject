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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_PERIOD_MS 10 // ms
#define PIXEL_PER_METER 1000.0
#define LCD_HEIGHT_PX 320
#define LCD_WIDTH_PX 240
#define BALL_RADIUS_PX 20
#define LCD_HEIGHT LCD_HEIGHT_PX / PIXEL_PER_METER
#define LCD_WIDTH LCD_WIDTH_PX / PIXEL_PER_METER
#define BALL_RADIUS BALL_RADIUS_PX / PIXEL_PER_METER
#define OFFSET 0.5 // do nhay
#define MAX_Y OFFSET + LCD_HEIGHT
#define MIN_Y OFFSET + BALL_RADIUS
#define MAX_X LCD_WIDTH / 2.0
#define MIN_X -MAX_X
#define MAX_Z 5 // 5m
#define MIN_Z 0
#define TOP_HEIGHT (float)(LCD_WIDTH / 2.0 - BALL_RADIUS) * MAX_Z / (LCD_WIDTH / 2.0 - 2*BALL_RADIUS)
#define G_FORCE 9.8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    float x;
    float y;
    float z;
} vector;

typedef struct {
	vector root;
	vector velocity;
	float time;
} BALL;

typedef enum {
	GAME_PLAYING = 0,
	GAME_OVER
} Game_Status;

Game_Status gameStatus = GAME_PLAYING;
vector gyroAngle = {0, 0, 0};
vector gyroLastAngularVelocity = {0, 0, 0};
float gyroRawData[3];
uint16_t score = 0;
BALL ball;

vector getCoordinates() {
    vector result;

    result.x = ball.root.x + ball.time*ball.velocity.x;
    result.y = ball.root.y + ball.time*ball.velocity.y;
    result.z = ball.root.z + ball.time*ball.velocity.z - 0.5*G_FORCE*ball.time*ball.time;

    return result;
}

vector multiplyVector(vector A, vector B) {
    vector result;

    result.x = A.y*B.z - A.z*B.y;
    result.y = A.z*B.x - A.x*B.z;
    result.z = A.x*B.y - A.y*B.x;

    return result;
}

uint16_t getShadowRadiusPx(float z) {
    if(z <= MAX_Z)
        return (uint16_t)(BALL_RADIUS*TOP_HEIGHT / (TOP_HEIGHT - z) * PIXEL_PER_METER);
    return (uint16_t)(LCD_WIDTH / 2.0 * PIXEL_PER_METER);
}

void setVector(vector* v, float x, float y, float z) {
	v->x = x;
	v->y = y;
	v->z = z;
}

// convert degree/s to radians/s
vector convertDpsToRds(vector v) {
	vector result;
	setVector(&result, v.x*0.0174533, v.y*0.0174533, v.z*0.0174533);
	return result;
}

float getVectorLength(vector v) {
	return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

void convertXYToPx(uint16_t* x, uint16_t* y, vector v) {
    const uint16_t radius = getShadowRadiusPx(v.z);
    const uint16_t max_X = LCD_WIDTH_PX - 2*radius;
    const uint16_t max_Y = LCD_HEIGHT_PX - 2*radius;

    *x = fabs(v.x) * PIXEL_PER_METER - radius;
    *y = fabs(v.y) * PIXEL_PER_METER - radius;

    uint16_t tempX = *x - *x / max_X * max_X;
    uint16_t tempY = *y - *y / max_Y * max_Y;

    *x = (*x / max_X) % 2 == 0 ? tempX : max_X - tempX;
    *y = (*y / max_Y) % 2 == 0 ? tempY : max_Y - tempY;

    *x += radius;
    *y += radius;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for GyroSample */
osThreadId_t GyroSampleHandle;
const osThreadAttr_t GyroSample_attributes = {
  .name = "GyroSample",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayLCD */
osThreadId_t DisplayLCDHandle;
const osThreadAttr_t DisplayLCD_attributes = {
  .name = "DisplayLCD",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for TrackBall */
osThreadId_t TrackBallHandle;
const osThreadAttr_t TrackBall_attributes = {
  .name = "TrackBall",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartGyroSample(void *argument);
void StartDisplayLCD(void *argument);
void StartTrackBall(void *argument);

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
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  BSP_GYRO_Init(); // L3GD20_FULLSCALE_500
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);//select on which layer we write
  BSP_LCD_DisplayOn();//turn on LCD
  BSP_LCD_Clear(LCD_COLOR_WHITE);//clear the LCD on white color
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);//set text background color
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

  setVector(&ball.root, 0.12, 0.16, 5);
  setVector(&ball.velocity, 0, 0, 0);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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
  /* creation of GyroSample */
  GyroSampleHandle = osThreadNew(StartGyroSample, NULL, &GyroSample_attributes);

  /* creation of DisplayLCD */
  DisplayLCDHandle = osThreadNew(StartDisplayLCD, NULL, &DisplayLCD_attributes);

  /* creation of TrackBall */
  TrackBallHandle = osThreadNew(StartTrackBall, NULL, &TrackBall_attributes);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartGyroSample */
/**
  * @brief  Function implementing the GyroSample thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGyroSample */
void StartGyroSample(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	// read angular velocity
	BSP_GYRO_GetXYZ(gyroRawData);
	for(uint8_t i = 0; i < 3; i++) {
		// Convert mdps to dps.
		gyroRawData[i] = gyroRawData[i] / 1000;
	}

	// Riemann sum - Midpoint
	gyroAngle.x += (gyroRawData[0] + gyroLastAngularVelocity.x) / 2.0 * SAMPLE_PERIOD_MS / 1000.0;
	gyroAngle.y += (gyroRawData[1] + gyroLastAngularVelocity.y) / 2.0 * SAMPLE_PERIOD_MS / 1000.0;
	gyroAngle.z += (gyroRawData[2] + gyroLastAngularVelocity.z) / 2.0 * SAMPLE_PERIOD_MS / 1000.0;

	setVector(&gyroLastAngularVelocity, gyroRawData[0], gyroRawData[1], gyroRawData[2]);

	osDelay(SAMPLE_PERIOD_MS); // tan so lay mau la 100 Hz
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayLCD */
/**
* @brief Function implementing the DisplayLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayLCD */
void StartDisplayLCD(void *argument)
{
  /* USER CODE BEGIN StartDisplayLCD */
  /* Infinite loop */
  uint16_t shadowRadius;
  for(;;) {
	  BSP_LCD_Clear(LCD_COLOR_WHITE);
	  char string[30];
	  sprintf(string, "Score: %d", score);
	  BSP_LCD_DisplayStringAtLine(0, (uint8_t*)string);

	  if(gameStatus == GAME_PLAYING) {
		  uint16_t x, y;
		  vector vBall = getCoordinates();
		  convertXYToPx(&x, &y, vBall);
		  shadowRadius = getShadowRadiusPx(vBall.z);
		  BSP_LCD_DrawCircle(x, y, shadowRadius);
	  }
	  else if(gameStatus == GAME_OVER) {
		  BSP_LCD_DisplayStringAtLine(1, (uint8_t*)"GAME OVER!");
		  while(1)
			  osDelay(1000000);
	  }
  }
  /* USER CODE END StartDisplayLCD */
}

/* USER CODE BEGIN Header_StartTrackBall */
/**
* @brief Function implementing the TrackBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTrackBall */
void StartTrackBall(void *argument)
{
  /* USER CODE BEGIN StartTrackBall */
	uint32_t lastTick = HAL_GetTick();
  /* Infinite loop */
  for(;;) {
	vector vBall = getCoordinates();

	char string[30];
	sprintf(string, "Ball's height = %f", vBall.z);
	CDC_Transmit_FS((uint8_t*)string, strlen(string));

	if(vBall.z <= 0.1) {
		// kiem tra luc danh cua vot.
		vector angularVelocityRad = convertDpsToRds(gyroLastAngularVelocity);
		vector temp = vBall;
		vector velocity = multiplyVector(angularVelocityRad, temp);

		// chi xet huong danh len.
		velocity.x = 0;
		velocity.y = 0;

		// danh nguoc huong hoac danh khong du luc
		if(velocity.z < 3) {
			// game over
			gameStatus = GAME_OVER;
			while(1)
				osDelay(10000000);
		}

		score++;

		ball.root = vBall;
		ball.velocity = velocity;
		ball.time = 0;
	}

	ball.time += (float)(HAL_GetTick() - lastTick) / 1000.0;
	lastTick = HAL_GetTick();

    osDelay(10);
  }
  /* USER CODE END StartTrackBall */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
