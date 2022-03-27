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
#include <BME280.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "I2Cdev.h"
#include "BMP085.h"
#include "dfplayer.h"
#include "stm32f1xx_it.h"
#include "bmp280.h"
#include "FLASH_PAGE_F1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_ON()            SET_BIT(GPIOA->ODR, GPIO_ODR_ODR7)
#define LED_OFF()           CLEAR_BIT(GPIOA->ODR, GPIO_ODR_ODR7)
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#define PRESSURE_DELTA_BMP085 150
#define PRESSURE_DELTA_BMP280 150
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;

uint8_t minute_global;
float pressure, temperature, humidity, init_preasure;
uint8_t touch_counter;
uint8_t mp3_number = 1;
char i2c_ports[128];
float delta = 0;

bool error = false;


HAL_StatusTypeDef result;
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
void check_rx()
{
    if(READ_BIT(USART1->SR, USART_SR_ORE))
    {
        (void) USART1->DR;
    }
    else if(READ_BIT(USART1->SR, USART_SR_FE))
    {
        (void) USART1->DR;
    }
    else if(READ_BIT(USART1->SR, USART_SR_ORE))
    {
        (void) USART1->DR;
    }
}

void status_led(uint8_t sensor)
{
    switch (sensor) {
        case 1:
            //HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
            for(int i = 0; i<=1; i++)
            {
                LED_ON();
                HAL_Delay(500);
                LED_OFF();
                HAL_Delay(500);
            }
            break;

        case 2:
            for(int i = 0; i<=2; i++)
            {
                LED_ON();
                HAL_Delay(500);
                LED_OFF();
                HAL_Delay(500);
            }
            break;

        case 3:
            while (error)
            {
                LED_ON();
                HAL_Delay(100);
                LED_OFF();
                HAL_Delay(100);
            }
    }
}

void i2c_scanner(char buffer[])
{
	memset(i2c_ports, 0, 4);
	for (int i=1; i<128; i++)
	{
		char test[128];
 	  /*
 	   * the HAL wants a left aligned i2c address
 	   * &hi2c1 is the handle
 	   * (uint16_t)(i<<1) is the i2c address left aligned
 	   * retries 2
 	   * timeout 2
 	   */
		result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
		if (result == HAL_OK)
		{
			//printf("0x%X", i); // Received an ACK at that address
			sprintf(test, "0x%X ", i);
			strcat(buffer, test);
		}
	}
}

float get_pressure()
{
    float pressure_sensor;
    if(i2c_ports[2] == '7' && i2c_ports[3] == '6')
    {
        pressure_sensor = BME280_ReadPressure();
        HAL_Delay(200);
    }
    if(i2c_ports[2] == '7' && i2c_ports[3] == '7')
    {
        BMP085_setControl(BMP085_MODE_PRESSURE_3);
        HAL_Delay(BMP085_getMeasureDelayMilliseconds(BMP085_MODE_PRESSURE_3));
        pressure_sensor = BMP085_getPressure();
        HAL_Delay(200);
    }
    return pressure_sensor;
}

void pressure_sensor_INIT()
{
    if(i2c_ports[2] == '7' && i2c_ports[3] == '6')
    {
        status_led(1);
        bmp280_init_default_params(&bmp280.params);
        bmp280.addr = BMP280_I2C_ADDRESS_0;
        bmp280.i2c = &hi2c1;
        bool bme280p = bmp280.id == BME280_CHIP_ID;
        BME280_Init();
        HAL_Delay(200);
    }

    if(i2c_ports[2] == '7' && i2c_ports[3] == '7')
    {
        status_led(2);
        I2Cdev_init(&hi2c1);
        while(!BMP085_testConnection());
        BMP085_initialize();
        HAL_Delay(200);
    }

    if(i2c_ports[2] == 0 && i2c_ports[3] == 0)
    {
        error = true;
        status_led(3);
    }
}

void presure_delta_calibration()
{
    HAL_Delay(2000);
    if(!READ_BIT(GPIOA->IDR, GPIO_IDR_IDR6))
    {
        LED_ON();
        while(!READ_BIT(GPIOA->IDR, GPIO_IDR_IDR6)){ HAL_Delay(100);}
        HAL_Delay(1000);
        float measurment_one = get_pressure();
        while(READ_BIT(GPIOA->IDR, GPIO_IDR_IDR6)){ HAL_Delay(100);}
        LED_OFF();
        HAL_Delay(1000);
        LED_ON();
        float measurment_second = get_pressure();
        float delta = measurment_second - measurment_one;

        if(delta<0)
        {
            error = true;
            status_led(3);
        }

        Flash_Write_NUM(0x0800A000, delta);
        for(int i = 0; i<=2; i++)
        {
            LED_OFF();
            HAL_Delay(100);
            LED_ON();
        }
        HAL_Delay(100);
        LED_OFF();


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
  SysTick_Config(SystemCoreClock/48000000);
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
  i2c_scanner(i2c_ports);   // Scan i2c ports
  pressure_sensor_INIT();         // Check sensor type
  presure_delta_calibration();
  delta = Flash_Read_NUM(0x0800A000);
  //delta = 150;
  init_preasure = get_pressure();
  DF_Init(30);             // Set Volume
  DF_MP3_Play(16);      // Play start music

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      check_rx();
      pressure = get_pressure();

      if(pressure-init_preasure>=delta)
      {
          DF_MP3_Play(mp3_number);
          mp3_number++;
          touch_counter++;
      }

      if(mp3_number==15){mp3_number=0;}
      if(touch_counter>=5)
      {
          HAL_Delay(1000);
          for(int i = 0; i<=10; i++)
          {
              LED_ON();
              HAL_Delay(50);
              LED_OFF();
              HAL_Delay(50);
          }
          init_preasure = get_pressure();
          touch_counter = 0;
      }
      HAL_Delay(200);
      //DF_MP3_Play(2);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DF_Busy_Pin */
  GPIO_InitStruct.Pin = DF_Busy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DF_Busy_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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

