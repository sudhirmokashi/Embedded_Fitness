/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "user_settings.h"

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "MY_LIS3DSH.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void InitDWTConfig(void);
static float CalculateCalories(float, float,unsigned long);
static void PrintAccelerometerXYZValues(int16_t ,int16_t ,int16_t ,float,float);
static void DetectSquats(int16_t, int16_t, int16_t, float, float);
static void DetectJumpingJacks(int16_t, int16_t, int16_t, float, float);
static void DetectSitups(int16_t, int16_t, int16_t, float, float);
static float CheckSitupsIntensity(void);
static void DetectExercise(void);
static void ExerciseAccelerometerData(void);
static void SuryanamaskarAccelerometerData(void);
static void DetectSuryanamaskar(int16_t,int16_t,int16_t,float,float);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

LIS3DSH_DataRaw myDataRaw;
LIS3DSH_DataScaled myDataScaled;

struct oddVal
{
	int16_t x;
	int16_t y;
	int16_t z;
	float Roll;
	float Pitch;
};

struct eveVal
{
	int16_t x;
	int16_t y;
	int16_t z;
	float Roll;
	float Pitch;
};

struct diffVal
{
	int16_t x;
	int16_t y;
	int16_t z;
	float Roll;
	float Pitch;
};

uint8_t drdyFlag = 0;
float Roll,Pitch;
unsigned long t1=0,t2=0,diff=0;
int16_t ExerciseCounter = 1,SquatsCount = 0, JumpJackCount = 0,SitupsCount = 0, mode=0, SuryaNamaskarPose = 0, SuryaNamaskarCount = 0;
struct oddVal v1 = {0};
struct eveVal v2 = {0};
struct diffVal v3 = {0};
//struct diffVal v4 = {0};
GPIO_PinState myPushButton;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
	LIS3DSH_InitTypeDef myAccConfigDef;
	//struct oddVal v1 = {0};
	//struct eveVal v2 = {0};
	//struct diffVal v3 = {0};
	//FILE *fptr = NULL;
	//float myVar = 1.23456;
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
	InitDWTConfig();
  /* USER CODE BEGIN 2 */
	myAccConfigDef.dataRate = LIS3DSH_DATARATE_12_5;
	myAccConfigDef.fullScale= LIS3DSH_FULLSCALE_4;
	myAccConfigDef.enableAxes= LIS3DSH_XYZ_ENABLE;
	myAccConfigDef.antiAliasingBW= LIS3DSH_FILTER_BW_50;
	myAccConfigDef.interruptEnable = true;
	LIS3DSH_Init(&hspi1,&myAccConfigDef);
	
	//LIS3DSH_X_calibrate(-1000,980);
	//LIS3DSH_Y_calibrate(-1020,1040);
	//LIS3DSH_Z_calibrate(-920,1040);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//fptr = fopen("F:/STM32F407/LIS3DSH_ThirdAttempt/LIS3DSH/MDK-ARM/logs/log.txt","a");
	/*if(fptr == NULL)
	{
		printf("Could not open log file\n");
		return EXIT_FAILURE;
	}
	fprintf(fptr,"opening log file...\n");*/
	printf("\r 5 seconds delay, long press the push button\n");
	HAL_Delay(5000);
	myPushButton = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	if(myPushButton == 1)
	{
			mode = mode + 1;
	}
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		if(drdyFlag == 1)
		{
			drdyFlag = 0;
			DetectExercise();
		}
  }
	//fprintf(fptr,"closing log file...\n");
	//fclose(fptr);
	//fptr = NULL;
	//return EXIT_SUCCESS;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MEMS_CS_Pin */
  GPIO_InitStruct.Pin = MEMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MEMS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

void PrintAccelerometerXYZValues(int16_t x,int16_t y,int16_t z,float roll,float pitch)
{
			printf(" **************************************** \n");
			printf(" **********RAW DATA********** \n");
			printf(" X value = %d\n",x);
			printf(" Y value = %d\n",y);
			printf(" Z value = %d\n",z);
			//printf(" **********SCALED DATA********** \n");			
	    //printf(" myDataScaled.x = %f\n",myDataScaled.x);
			//printf(" myDataScaled.y = %f\n",myDataScaled.y);
			//printf(" myDataScaled.z = %f\n",myDataScaled.z);
			printf(" **********ANGLE DATA********** \n");
			printf("Roll = %f\n",roll);
			printf("Pitch = %f\n",pitch);
			printf("\n");
			printf(" **************************************** \n");
}


void DetectExercise(void)
{
				if(mode >= 1)
				{
					printf("\rExercise Mode: Yoga\n");
					HAL_GPIO_TogglePin(GPIOD,BLUE_Pin);
					SuryanamaskarAccelerometerData();
					t1 = DWT->CYCCNT;
					DetectSuryanamaskar(v1.x,v1.y,v1.z,v1.Roll,v1.Pitch);
					if(SuryaNamaskarCount == MAX_SURYANAMASKAR_COUNT)
					{
						printf("***ALL SURYANAMASKARS COMPLETED, LETS MOVE TO NEXT EXERCISE, 5 seconds break!!!****\n");
						t2 = DWT->CYCCNT;
						diff = t2-t1;
						printf("Calories Burnt = %0.5f\n",CalculateCalories(MET_SURYANAMASKAR,WEIGHT,(diff/168000000)));
						HAL_Delay(5000);
					}
				}
				else
				{
					printf("\rNormal Exercise Mode\n");
					ExerciseAccelerometerData();
					if((v3.z < v3.x) && (v3.z < v3.y) && (v3.Pitch >= 15) && (v3.Roll <= 50))
					{
						printf("****************Squats detected!!!!!!!!**********************\n");
						t1 = DWT->CYCCNT;
						HAL_GPIO_TogglePin(GPIOD,GREEN_Pin);
						while(1)
						{
								ExerciseAccelerometerData();
								if((ExerciseCounter % 2 != 0) && (ExerciseCounter > 1))
								{
									DetectSquats(v3.x,v3.y,v3.z,v3.Roll,v3.Pitch);
								}
								if(SquatsCount == MAX_SQUATS_COUNT)
								{
									printf("***ALL SQUATS COMPLETED, LETS MOVE TO NEXT EXERCISE, 5 seconds break!!!****\n");
									t2 = DWT->CYCCNT;
									diff = t2-t1;
									printf("Calories Burnt = %0.5f\n",CalculateCalories(MET_SQUATS,WEIGHT,(diff/168000000)));
									HAL_Delay(5000);
								}
						}
					}
					else if((v3.x < v3.y) && (v3.x < v3.z) && (v3.Pitch >= 80) && (v3.Roll >= 80))
					{
						printf("****************Jumping Jacks detected!!!!!!!!**********************\n");
						t1 = DWT->CYCCNT;
						HAL_GPIO_TogglePin(GPIOD,ORANGE_Pin);
						while(1)
						{
								ExerciseAccelerometerData();
								if((ExerciseCounter % 2 != 0) && (ExerciseCounter > 1))
								{
									DetectJumpingJacks(v3.x,v3.y,v3.z,v3.Roll,v3.Pitch);
								}
								if(JumpJackCount == MAX_JUMPINGJACKS_COUNT)
								{
									printf("***ALL JUMPING JACKS COMPLETED, LETS MOVE TO NEXT EXERCISE, 5 seconds break!!!****\n");
									t2 = DWT->CYCCNT;
									diff = t2-t1;
									printf("Calories Burnt = %0.5f\n",CalculateCalories(MET_JUMPINGJACKS,WEIGHT,(diff/168000000)));
									HAL_Delay(5000);
								}
						}
					}
					else if((v3.z < v3.x) && (v3.z < v3.y) && (v3.Pitch >= 100) && (v3.Roll >= 50))
					{
						printf("****************Situpps detected!!!!!!!!**********************\n");
						t1 = DWT->CYCCNT;
						HAL_GPIO_TogglePin(GPIOD,RED_Pin);
						while(1)
						{
								ExerciseAccelerometerData();
								if((ExerciseCounter % 2 != 0) && (ExerciseCounter > 1))
								{
									DetectSitups(v3.x,v3.y,v3.z,v3.Roll,v3.Pitch);
								}
								if(SitupsCount == MAX_SITUPS_COUNT)
								{
									printf("***ALL SITUPS COMPLETED, LETS MOVE TO NEXT EXERCISE, 5 seconds break!!!****\n");
									t2 = DWT->CYCCNT;
									diff = t2-t1;
									printf("Calories Burnt = %0.5f\n",CalculateCalories(CheckSitupsIntensity(),WEIGHT,(diff/168000000)));
									HAL_Delay(5000);
								}
						}
					}
				}
}


void DetectSquats(int16_t xVal, int16_t yVal, int16_t zVal, float Roll, float Pitch)
{
	
		if((zVal < xVal) && (zVal < yVal) && (Pitch >= 15))
		{
			SquatsCount = SquatsCount + 1;
			printf("Squats count = %d \n",SquatsCount);
		}
}


void DetectJumpingJacks(int16_t xVal, int16_t yVal, int16_t zVal, float Roll, float Pitch)
{
	
		if((xVal < yVal) && (xVal < zVal) && (Pitch >= 100) && (Roll >= 100))
		{
			JumpJackCount = JumpJackCount + 1;
			printf("Jumping Jacks count = %d \n",JumpJackCount);
		}
}


void DetectSitups(int16_t xVal, int16_t yVal, int16_t zVal, float Roll, float Pitch)
{
		if((zVal < xVal) && (zVal < yVal) && (Pitch >= 100) && (Roll >= 50))
		{
			SitupsCount = SitupsCount + 1;
			printf("Situps Count = %d \n",SitupsCount);
		}
}
void DetectSuryanamaskar(int16_t xVal, int16_t yVal, int16_t zVal, float Roll, float Pitch)
{
	if((xVal > 0) && (yVal < 0) && (zVal > 0) && (SuryaNamaskarPose == 0))
	{
		printf("\rSurya Namaskar Pose 1 : Prarthana Mudra\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal > 0) && (yVal < 0) && (zVal < 0) && (SuryaNamaskarPose == 1))
	{
		printf("\rSurya Namaskar Pose 2 : Hastottanasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal > 0) && (yVal > 0) && (zVal < 0) && (SuryaNamaskarPose == 2))
	{
		printf("\rSurya Namaskar Pose 3 : Padhastasan \n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal < 0) && (yVal > 0) && (zVal > 0) && (SuryaNamaskarPose == 3))
	{
		printf("\rSurya Namaskar Pose 4 : Ashwa Sanchalanasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal < 0) && (yVal > 0) && (zVal > 0) && (SuryaNamaskarPose == 4))
	{
		printf("\rSurya Namaskar Pose 5 : Dandasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal > 0) && (yVal > 0) && (zVal > 0) && (SuryaNamaskarPose == 5))
	{
		printf("\rSurya Namaskar Pose 6 : Ashtang Pranam\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal < 0) && (yVal > 0) && (zVal > 0) && (SuryaNamaskarPose == 6))
	{
		printf("\rSurya Namaskar Pose 7 : Bhujangasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal < 0) && (yVal > 0) && (zVal > 0 ) && (SuryaNamaskarPose == 7))
	{
		printf("\rSurya Namaskar Pose 8 : Parvatasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal > 0) && (yVal > 0) && (zVal > 0) && (SuryaNamaskarPose == 8))
	{
		printf("\rSurya Namaskar Pose 9 : Ashwa Sanchalanasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal > 0) && (yVal > 0) && (zVal < 0) && (SuryaNamaskarPose == 9))
	{
		printf("\rSurya Namaskar Pose 10 : Padhastasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal > 0) && (yVal < 0) && (zVal < 0) && (SuryaNamaskarPose == 10))
	{
		printf("\rSurya Namaskar Pose 11 : Hastottanasan\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if((xVal > 0) && (yVal < 0) && (zVal > 0) && (SuryaNamaskarPose == 11))
	{
		printf("\rSurya Namaskar Pose 12 : Prarthana Mudra\n");
		SuryaNamaskarPose = SuryaNamaskarPose + 1;
	}
	else if(SuryaNamaskarPose == 12)
	{
		printf("\rSurya Namaskar Completed\n");
		SuryaNamaskarCount = SuryaNamaskarCount + 1;
		SuryaNamaskarPose = 0;
	}
}


void ExerciseAccelerometerData()
{
	
			if(ExerciseCounter % 2 != 0)
			{
				printf("3 seconds delay initiated\n");
				HAL_Delay(3000);
				printf("Taking Initial values\n");
				myDataRaw = LIS3DSH_GetDataRaw();
				myDataScaled = LIS3DSH_GetDataScaled();
				Roll = CalcAngle(myDataScaled.z,myDataScaled.x);
				Pitch = CalcAngle(myDataScaled.z,myDataScaled.y);
				v1.x = myDataRaw.x;
				v1.y = myDataRaw.y;
				v1.z = myDataRaw.z;
				v1.Roll = Roll;
				v1.Pitch = Pitch;
				PrintAccelerometerXYZValues(v1.x,v1.y,v1.z,v1.Roll,v1.Pitch);
				ExerciseCounter = ExerciseCounter + 1;
			}
			else
			{
					printf("3 seconds delay initiated\n");
					HAL_Delay(3000);
					printf("Taking final values\n");
					myDataRaw = LIS3DSH_GetDataRaw();
					myDataScaled = LIS3DSH_GetDataScaled();
					Roll = CalcAngle(myDataScaled.z,myDataScaled.x);
					Pitch = CalcAngle(myDataScaled.z,myDataScaled.y);
					v2.x = myDataRaw.x;
					v2.y = myDataRaw.y;
					v2.z = myDataRaw.z;
					v2.Roll = Roll;
					v2.Pitch = Pitch;					
					PrintAccelerometerXYZValues(v2.x,v2.y,v2.z,v2.Roll,v2.Pitch);
					ExerciseCounter = ExerciseCounter + 1;
			}
			if(v1.x != 0 && v2.x != 0 && (ExerciseCounter % 2 != 0) && (ExerciseCounter > 1))
			{
				v3.x = abs(abs(v2.x) - abs(v1.x));
				v3.y = abs(abs(v2.y) - abs(v1.y));
				v3.z = abs(abs(v2.z) - abs(v1.z));
				v3.Roll = abs((abs((int)(v2.Roll)) - abs((int)(v1.Roll))));
				v3.Pitch = abs((abs((int)(v2.Pitch)) - abs((int)(v1.Pitch))));
				PrintAccelerometerXYZValues(v3.x,v3.y,v3.z,v3.Roll,v3.Pitch);
			}
			
}


void SuryanamaskarAccelerometerData()
{
				printf("5 seconds delay initiated\n");
				HAL_Delay(5000);
				printf("Taking Initial values\n");
				myDataRaw = LIS3DSH_GetDataRaw();
				myDataScaled = LIS3DSH_GetDataScaled();
				Roll = CalcAngle(myDataScaled.z,myDataScaled.x);
				Pitch = CalcAngle(myDataScaled.z,myDataScaled.y);
				v1.x = myDataRaw.x;
				v1.y = myDataRaw.y;
				v1.z = myDataRaw.z;
				v1.Roll = Roll;
				v1.Pitch = Pitch;
				PrintAccelerometerXYZValues(v1.x,v1.y,v1.z,v1.Roll,v1.Pitch);
}

float CalculateCalories(float MET, float weight,unsigned long duration)
{
	float calories = 0.0f;
	calories = ((MET*7.7f*weight*2.2f)/(200*60));
	return (calories*CALORIES);
}

float CheckSitupsIntensity()
{
	if(SITUPS_INTENSITY == 1)
		return MET_SITUPS_LIGHT;
	else if(SITUPS_INTENSITY == 2)
		return MET_SITUPS_MODERATE;
	else
		return MET_SITUPS_VIGOROUS;
}

void InitDWTConfig(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	drdyFlag = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

































































