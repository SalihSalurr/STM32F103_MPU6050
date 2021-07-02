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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	char buf[100] = " ";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RAD_TO_DEG 57.295779513082320876798154814105
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
	
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint32_t timer;
float dereceX;



Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};


int16_t Accel_X_Raw = 0;
int16_t Accel_Y_Raw = 0;
int16_t Accel_Z_Raw = 0;	
	
int16_t Gyro_X_Raw = 0;
int16_t Gyro_Y_Raw = 0;
int16_t Gyro_Z_Raw = 0;
	
double KalmanAngleX;
double KalmanAngleY;

float Ax,Ay,Az,Gx,Gy,Gz;



double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void MPU6050_Init (void) {
	uint8_t check,Data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,WHO_AM_I_REG, 1, &check, 1, 1000);
		
	if(check == 104){
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
		
		Data = 0x07;			
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
			
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}
void MPU6050_Read_Accel (void){
	
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	
	Accel_X_Raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1] );
	Accel_Y_Raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3] );
	Accel_Z_Raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5] );

	Ax = Accel_X_Raw / 16384.0;
	Ay = Accel_Y_Raw / 16384.0;
	Az = Accel_Z_Raw / 16384.0;

}	
	
void MPU6050_Read_Gyro (void){
	
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	
	Gyro_X_Raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1] );
	Gyro_Y_Raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3] );
	Gyro_Z_Raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5] );

	Gx = Gyro_X_Raw / 131.0;
	Gy = Gyro_Y_Raw / 131.0;
	Gz = Gyro_Z_Raw / 131.0;
}		

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx) {
    uint8_t Rec_Data[14];
    int16_t temp;


	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);
	
	Accel_X_Raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1] );
	Accel_Y_Raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3] );
	Accel_Z_Raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5] );
  //  temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
	Gyro_X_Raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1] );
	Gyro_Y_Raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3] );
	Gyro_Z_Raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5] );

	Ax = Accel_X_Raw / 16384.0;
	Ay = Accel_Y_Raw / 16384.0;
	Az = Accel_Z_Raw / 16384.0;
   // Az = Accel_Z_Raw / Accel_Z_corrector;
    //Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
	Gx = Gyro_X_Raw / 131.0;
	Gy = Gyro_Y_Raw / 131.0;
	Gz = Gyro_Z_Raw / 131.0;

    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
           Accel_X_Raw * Accel_X_Raw + Accel_Z_Raw * Accel_Z_Raw);
    if (roll_sqrt != 0.0) {
        roll = atan(Accel_Y_Raw / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-Accel_X_Raw, Accel_Z_Raw) * RAD_TO_DEG;
    if ((pitch < -90 && KalmanAngleY > 90) || (pitch > 90 && KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        KalmanAngleY = pitch;
    } else {
        KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, Gy, dt);
    }
    if (fabs(KalmanAngleY) > 90)
        Gx = -Gx;
    KalmanAngleX = Kalman_getAngle(&KalmanX, roll, Gy, dt);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		MPU6050_Read_All(&hi2c1);
		//dereceX = KalmanX.angle;
		sprintf(buf, " Ax: %.2f Ay: %.2f Az: %.2f \n Gx: %.2f Gy: %.2f Gz: %.2f  \n\r", Ax, Ay, Az, Gx, Gy, Gz);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf, strlen(buf), 1000);
		HAL_Delay(500);
		
		
		
		/*
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		sprintf(buf, " Ax: %.2f Ay: %.2f Az: %.2f \n Gx: %.2f Gy: %.2f Gz: %.2f \n\r", Gx, 	Gy, Gz, Ax, Ay, Az);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf, strlen(buf), 1000);
		HAL_Delay(500);
*/
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
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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