/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CPR 2000 // TODO: adjust after measurement
#define RPM_SMOOTH 5

// Simple printf retarget to USART2
int __io_putchar(int ch) {
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
  return ch;
}

// Motor helpers
static inline void set_duty_pct(uint8_t pct) {
	if (pct > 100) pct = 100;
	uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1; // ARR+1
	uint32_t ccr = (period * pct + 50) / 100;               // rounded
	if (ccr > period) ccr = period;                          // clamp
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

// IN1/IN2 direction control (you mapped PA0=IN1 (PWM), PA1=IN2 (GPIO))
static inline void motor_dir_forward(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN2 = 0
  // IN1 is PWM on PA0 (TIM2_CH1)
}
static inline void motor_dir_reverse(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // IN2 = 1
  // IN1 (PWM) will be inverted effect relative to direction
}

// Soft-start: brief kick then settle
static inline void soft_start_to(uint8_t target_pct, uint8_t kick_pct, uint16_t kick_ms) {
  set_duty_pct(kick_pct);
  HAL_Delay(kick_ms);
  set_duty_pct(target_pct);
}

// State variables (declare ONLY here)
static uint16_t enc_last;
static uint32_t t_sample;
static uint32_t t_print;
static int32_t  delta_accum;
static float rpm_history[RPM_SMOOTH];
static uint8_t rpm_idx = 0;

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);           // if you're using PWM
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);     // start encoder

  set_duty_pct(20);

  enc_last    = __HAL_TIM_GET_COUNTER(&htim3);        // <-- runtime init OK here
  t_sample    = HAL_GetTick();
  t_print     = HAL_GetTick();
  delta_accum = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	//FIND CPR of Motor Code Snippet
	  /*
	    uint16_t now2 = __HAL_TIM_GET_COUNTER(&htim3);
	    printf("enc=%u\r\n", now2);
	    HAL_Delay(200);
	    */

	  uint32_t now = HAL_GetTick();

	    if ((now - t_sample) >= 10) {              // sample every 10 ms
	      t_sample += 10;

	      uint16_t enc_now = __HAL_TIM_GET_COUNTER(&htim3);
	      int16_t  d       = (int16_t)(enc_now - enc_last); // wrap-safe
	      enc_last = enc_now;

	      delta_accum += d;
	    }

	    if ((now - t_print) >= 100) {              // print every 100 ms
	      uint32_t dt = now - t_print;
	      t_print = now;

	      float cps = (1000.0f / (float)dt) * (float)delta_accum;
	      float rps = cps / (float)CPR;
	      float measured_rpm = rps * 60.0f;

	      rpm_history[rpm_idx++] = measured_rpm;
	      if (rpm_idx >= RPM_SMOOTH) rpm_idx = 0;

	      float rpm_sum = 0.0f;
	      for (uint8_t i = 0; i < RPM_SMOOTH; i++) rpm_sum += rpm_history[i];
	      float rpm_filtered = rpm_sum / RPM_SMOOTH;

	      // --- Simple P controller addition ---
	      static float duty = 0.0f;             // persistent duty value (0–100%)
	      const float target_rpm = 20.0f;      // desired speed
	      const float Kp = 0.05f;               // proportional gain (start small!)
	      static float integral = 0.0f;
	      const float Ki = 0.005f;  // small, start tiny

	      // anti-windup clamp
	      if (integral > 10.0f) integral = 10.0f;
	      if (integral < -10.0f) integral = -10.0f;


	      float error = target_rpm - rpm_filtered;       // speed error
	      if (fabs(error) <= 1.0f) error = 0.0f;
	      else integral += Ki * error;

	      if (error != 0.0f){
	      duty += Kp * error + integral;                   // adjust duty
	      if (duty > 100.0f) duty = 100.0f;
	      if (duty < 0.0f) duty = 0.0f;
	      set_duty_pct(duty);                   // update PWM output
	      }

	      printf("RPM=%.1f target=%.1f duty=%.1f%%\r\n", rpm_filtered, target_rpm, duty);



	      printf("accum=%ld  cps=%.1f  RPM=%.1f\r\n",
	    		  (long)delta_accum, cps, rpm_filtered);

	      delta_accum = 0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
