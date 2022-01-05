/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "as5048.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "defines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANGLE_LOOP	1
#define DRV_LOOP	2
#define PC_LOOP	3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint32_t angle_loop, drv_loop, pc_loop, count_max;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SetLoopsFrequency(uint8_t loop, uint32_t value);
void SetUpdateFreq(uint32_t freq);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float speed_drv1 =0.1f;
float speed = 0.0f;
volatile uint8_t f_send_to_drv = 0;  // flag - to send 
#define BUF_SIZE_DRV			6
#define BUF_SIZE_DRV_SEND	7
uint8_t buf_drv_send[BUF_SIZE_DRV_SEND];					// uint8_t code + float speed
uint8_t buf_drv_send_decoded[BUF_SIZE_DRV_SEND - 2];
volatile uint8_t lets_time_to_read_angle = 0, lets_time_to_send_angle = 0;
uint16_t angle_raw_int = 0;
uint16_t angle_raw_int_filtred = 0;
float angle_raw_float = 0;
float angle_raw_float_filtred = 0;
uint8_t str_1f[BUF_SIZE_FLOAT_UART+2];
//uint8_t str_2f[2*BUF_SIZE_FLOAT_UART+2+1];	 // format: $float;   ore $float1 float2;
uint8_t str_2f[50];	 // format: $float;   ore $float1 float2;
uint32_t update_freq = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
//float temp_f  = 0;
//uint16_t len = 0;
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
	SystemCoreClockUpdate();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF); // clear interrupt bits
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	USART1->CR1 |= USART_CR1_RXNEIE;
	USART2->CR1 |= USART_CR1_RXNEIE;
	//SysTick_Config(SystemCoreClock/2000); // 2000 times per second
	//Set update frequency in Hz (Note!! Timer clock = 1 MHz)
	SetUpdateFreq(500);	
	//Set angle loop frequency in Hz
	SetLoopsFrequency(ANGLE_LOOP, 100); // 100 Hz => 10 ms
	//Set driver loop frequency in Hz
	SetLoopsFrequency(DRV_LOOP, 50);    // 50 Hz => 20 ms
	//Set PC loop frequency in Hz
	SetLoopsFrequency(PC_LOOP, 20);     // 20 Hz => 50 ms
	//Update timer start
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF); // clear int flag
	HAL_TIM_Base_Start_IT(&htim1);

	HAL_TIM_Base_Start_IT(&htim3);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (f_send_to_drv) {
			f_send_to_drv = 0;
			buf_drv_send_decoded[0] = 'v';
			memcpy(buf_drv_send_decoded + 1, &speed, sizeof(float));
			cobs_encode(buf_drv_send_decoded, BUF_SIZE_DRV_SEND - 2, buf_drv_send); // inp, lenght = 5 ?, outp
			HAL_UART_Transmit(&huart2, buf_drv_send, BUF_SIZE_DRV_SEND, 0x0FFF);
		}
		if (lets_time_to_read_angle) {
			lets_time_to_read_angle = 0;
			//temp_f = get_angle();	
			angle_raw_int = get_angle_raw();	
			angle_raw_float = (float)(angle_raw_int)*0.021973997;			
			angle_raw_int_filtred = Filter_SMA(angle_raw_int);	
			angle_raw_float_filtred = (float)(angle_raw_int_filtred)*0.021973997;
			// Speed calcualtion
			if (angle_raw_float_filtred > 185) speed = 0; 
			else {
				speed = 1.1125 - 0.00612 * angle_raw_float_filtred;
			}
			if (speed <0.0f) speed = 0.0f;
			if (speed > 1.0f) speed = 1.0f;
		}		
		if (lets_time_to_send_angle)
		{
			lets_time_to_send_angle = 0;
			//len = strlen(str_2f);
			sprintf(str_2f, "$%f %f;" , angle_raw_float, angle_raw_float_filtred);
			HAL_UART_Transmit(&huart1, str_2f, strlen(str_2f), 0x0FFF);	
		}
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

/* USER CODE BEGIN 4 */
void SetUpdateFreq(uint32_t freq)
{
	if(freq > 1000000) freq = 1000000;
	TIM3->ARR = SystemCoreClock/(TIM3->PSC+1)/freq;
	update_freq = count_max = freq;
}


void SetLoopsFrequency(uint8_t loop, uint32_t value)
{
	if(value != 0)
	{
		switch(loop)
		{
			case ANGLE_LOOP:		
			{
				angle_loop = update_freq/value;
				break;
			}
			case DRV_LOOP:		
			{
				drv_loop = update_freq/value;
				break;
			}
			case PC_LOOP:		
			{
				pc_loop = update_freq/value;
				break;
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

