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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "u8g2_stm32.h"
#include "ui.h"
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

/* USER CODE BEGIN PV */

static uint32_t pressStartTime = 0;

static int8_t isPowerBoot = 0;
uint32_t adcValues[3] = {0};

int isShow = 1;
u8g2_t u8g2;


double SR = 0.005;
const double R25 = 20000.0;
const double R26 = 3300.0;
const int adc_max = 4095;
const double adc_max_voltage = 3.3;
const int magnification = 39;
const int intervals = 1800;

/* USER CODE BEGIN PV */
double calculate_voltage(const uint32_t adc_value)
{
    const double adc_voltage = adc_value * (adc_max_voltage / adc_max);

    return adc_voltage * (R25 + R26) / R26;
}

double calculate_current(const uint32_t adc_value)
{
    return adc_value * adc_max_voltage / (adc_max * magnification * SR);
}

void HAL_GPIO_EXTI_Rising_Callback(const uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
        const uint32_t pressDuration = HAL_GetTick() - pressStartTime;
        if (pressDuration >= intervals)
        {
            isPowerBoot = 1;
            HAL_GPIO_TogglePin(XG_GPIO_Port,XG_Pin);
            isShow = !isShow;
            // u8g2_SetPowerSave(&u8g2, isShow);
        }
        else
        {
            isPowerBoot = 0;
        }
        // Print the status of isPowerBoot
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "isPowerBoot: %d %d \r\n", isPowerBoot, isShow);
        HAL_UART_Transmit_IT(&huart1, buffer, strlen(buffer));
    }
}

void HAL_GPIO_EXTI_Falling_Callback(const uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
        pressStartTime = HAL_GetTick(); // Record the time when the button is pressed
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM17) // 检查是否为 TIM2 中断
    {
        const uint32_t current_adc = adcValues[0];
        const uint32_t voltage_adc = adcValues[1];
        // int temperature_adc = adcValues[3];

        // float b = temperature_adc * (3.3 / 4095);
        // float temperature = (1.43 - b) / 0.0043 + 25;

        const double current = calculate_current(current_adc);
        const double voltage = calculate_voltage(voltage_adc);


        char data[50];
        snprintf(data, sizeof(data), "voltage %d %.2fV \r\ncurrent %d %.2fA \r\n", voltage_adc, voltage, current_adc,
                 current);
        HAL_UART_Transmit_IT(&huart1, data, strlen(data));
        //
        // snprintf(data, sizeof(data), "temperature %.2f \r\n", temperature);
        // HAL_UART_Transmit_IT(&huart1, data, strlen(data));


        // float voltage = generate_random_float(19, 21);
        // float current = generate_random_float(5, 13);
        // float power = voltage * current;
        // updateUI(&u8g2, voltage, current, power);
        // HAL_Delay(1000);
    }
    else if (htim->Instance == TIM16)
    {
        if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float generate_random_float(float min, float max)
{
    return min + (max - min) * ((float)rand() / (float)RAND_MAX);
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
    HAL_RCC_DeInit();
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_I2C2_Init();
    MX_IWDG_Init();
    MX_TIM17_Init();
    MX_TIM16_Init();
    /* USER CODE BEGIN 2 */

    // HAL_GPIO_WritePin(XG_GPIO_Port,XG_Pin, isPowerBoot);
    u8g2Init(&u8g2);

    HAL_UART_Transmit_IT(&huart1, "Hello World\r\n", strlen("Hello World\r\n"));

    HAL_ADC_Start_DMA(&hadc1, adcValues, sizeof(adcValues));
    HAL_TIM_Base_Start_IT(&htim16);
    HAL_TIM_Base_Start_IT(&htim17);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // HAL_Delay(1500);


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
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
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
