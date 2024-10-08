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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define constants for PWM pulse widths for the servo
 #define SERVO_MIN_PULSE_WIDTH 	1000 // 1ms pulse width (-60 degrees)
 #define SERVO_MAX_PULSE_WIDTH 2000  // 2ms pulse width (+60 degrees)
 #define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width (neutral position)
 #define SERVO_PERIOD 20000  // 20ms period for PWM signal
 #define FILTER_WINDOW_SIZE 3        // Size of the moving average window
 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t map(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void set_servo_position(uint16_t pulse_width);
uint16_t read_potentiometer(void);

uint16_t moving_average_filter(uint16_t new_value);
uint16_t adc_buffer[FILTER_WINDOW_SIZE] = {0};  // Buffer to store ADC readings
uint8_t buffer_index = 0;                    // Index to keep track of current sample
uint32_t pot_value = 0; 


uint32_t adc_to_millivolts(uint16_t adc_value);
void send_voltage_over_uart(uint32_t voltage);

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM on channel 1 of TIM2 (PA0)
  HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
  HAL_ADC_Start (&hadc1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
// Read the potentiometer value
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        uint16_t pot_value = read_potentiometer();

        // Apply moving average filter to smooth the ADC reading
        uint16_t filtered_value = moving_average_filter(pot_value);

        // Map the potentiometer value (0-4095) to the servo pulse width (1000-2000)
        uint16_t servo_pulse_width = map(filtered_value, 0, 4095, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

        // Set the servo position based on the mapped pulse width
        set_servo_position(servo_pulse_width);

        uint32_t voltage = adc_to_millivolts(filtered_value);
      
        static uint32_t last_time = 0;
    if (HAL_GetTick() - last_time >= 1000)
    {
        last_time = HAL_GetTick();
        
        send_voltage_over_uart(voltage);
    }

    HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


// Function to map the potentiometer ADC value to servo pulse width
uint16_t map(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Function to set the PWM duty cycle for the servo (servo position)
void set_servo_position(uint16_t pulse_width)
{
    // Set the PWM compare value (CCR) to control the pulse width
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
}

// Function to read the potentiometer value using ADC
uint16_t read_potentiometer(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  // Wait for ADC conversion
    HAL_ADC_GetValue(&hadc1);  // Get ADC value (0-4095)
    uint16_t pot_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return pot_value;
}

// Function to implement a moving average filter
uint16_t moving_average_filter(uint16_t new_value)
{
    static uint32_t sum = 0;  // Keep track of the sum of the buffer values

    // Subtract the oldest value from the sum
    sum -= adc_buffer[buffer_index];

    // Add the new value to the buffer and to the sum
    adc_buffer[buffer_index] = new_value;
    sum += new_value;

    // Increment the buffer index and wrap around if necessary
    buffer_index = (buffer_index + 1) % FILTER_WINDOW_SIZE;

    // Return the average of the buffer values
    return (uint16_t)(sum / FILTER_WINDOW_SIZE);
}

// Function to convert ADC value to millivolts
uint32_t adc_to_millivolts(uint16_t adc_value)
{
    // Vref is assumed to be 3.3V (3300 mV), and ADC resolution is 12 bits (0-4095)
    return (uint32_t)(adc_value * 3300 / 4095);
}

void uint32_to_str(uint32_t num, char* str) {
    char temp[12];  // Временный буфер для числа (максимум 10 цифр для uint32_t + знак конца строки)
    int i = 0;

    // Обработка числа
    do {
        temp[i++] = (num % 10) + '0';  // Берем последнюю цифру числа и превращаем ее в символ
        num /= 10;  // Убираем последнюю цифру
    } while (num > 0);

    // Развернем строку, так как цифры в temp идут в обратном порядке
    int j = 0;
    while (i > 0) {
        str[j++] = temp[--i];  // Копируем цифры в правильном порядке
    }

    str[j] = ' ';  // Завершаем строку нулевым символом
}

// Function to send voltage over UART in millivolts
void send_voltage_over_uart(uint32_t voltage)
{

  
  char buffer[50] = "Voltage: ";  
  char endstr[4] =  "mV\n\0";
  char num[12];

  uint32_to_str(voltage, num);
  
  strcat(buffer, num);
  strcat(buffer, endstr);

  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer) - 1, 1000);


    /*char buffer[50];  // Make sure this is large enough
    int len = snprintf(buffer, sizeof(buffer), "ADC Voltage: %lu mV\r\n", voltage);*/
    // Debug: Check if snprintf is working
     //HAL_UART_Transmit(&huart2, (uint8_t *)"Entering send_voltage_over_uart\r\n", 34, 1000);
    /*if (len < 0)
    {
        // Handle snprintf error
        char *error_msg = "snprintf error\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)error_msg, strlen(error_msg), 1000);
        return;
    }

    if (len >= sizeof(buffer))
    {
        // Output was truncated; adjust len to avoid sending garbage
        len = sizeof(buffer) - 1;
    }

    // Send the formatted string over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 1000);*/
    // Debug: Confirm successful transmission
    //HAL_UART_Transmit(&huart2, (uint8_t *)"Exiting send_voltage_over_uart\r\n", 32, 1000);
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
