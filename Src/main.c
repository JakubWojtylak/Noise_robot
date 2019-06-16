/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
//Zmienne przycisk************************************
volatile GPIO_PinState StanPrzycisku;
volatile GPIO_PinState PoprzedniStanPrzycisku = GPIO_PIN_SET;
volatile uint32_t PoprzedniCzasPrzycisku;
//****************************************************

uint32_t adc[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


int adc_read(uint32_t channel)
{

	 ADC_ChannelConfTypeDef adc_ch;
	 adc_ch.Channel = channel;

	 adc_ch.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	 adc_ch.Rank = ADC_REGULAR_RANK_1;

	 HAL_ADC_ConfigChannel(&hadc1, &adc_ch);

	 HAL_ADC_Start(&hadc1);
	 HAL_ADC_PollForConversion(&hadc1, 100);

	 return HAL_ADC_GetValue(&hadc1);
}


void ruchPrzod(uint16_t v, uint16_t t){

	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, GPIO_PIN_RESET);


	if(v > 1000) v = 1000;

	TIM1->CCR1  = v;
	TIM1->CCR2 = v;

	for(int i=0; i<t; i++){
		HAL_Delay(1);
	}

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;


}

void obrot_lewo(uint16_t v, uint16_t t)
{
	   HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, GPIO_PIN_RESET);


	if(v > 1000) v = 1000;

	TIM1->CCR1  = v;
	TIM1->CCR2 = v;

	for(int i=0; i<t; i++){
		HAL_Delay(1);

	}

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;



}


void obrot_prawo(uint16_t v, uint16_t t){
	   HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, GPIO_PIN_SET);

		if(v>1000) v = 1000;
		TIM1->CCR1  = v;
		TIM1->CCR2 = v;
		for(int i=0; i<t; i++){
			HAL_Delay(1);

		}
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;

}
void obrot180(){
obrot_lewo(1000,2000);
}


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //HAL_ADCEx_Calibration_Start(&hadc1);

  //HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_ADC_Start_DMA(&hadc1,adc,4);


  //HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_SET);

  char DataToSend[100]; // Tablica zawierajaca dane do wyslania
  uint8_t MessageLength = 0; // Zawiera dlugosc wysylanej wiadomosci


  /*TIM1->CCR1 = 900;
  TIM1->CCR2 = 0;

  HAL_Delay(2000);

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 900;

  HAL_Delay(2000);

  TIM1->CCR1 = 900;
  TIM1->CCR2 = 900;
*/
/*   HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, GPIO_PIN_SET);


   	TIM1->CCR1 = 1000;
   	TIM1->CCR2 = 1000;*/


  ruchPrzod(500, 5000);
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //Mikrofon1 = adc_read(ADC_CHANNEL_0);
	  //Mikrofon2 = adc_read(ADC_CHANNEL_1);
	  /*Mikrofon3 = adc_read(ADC_CHANNEL_2);
	  Mikrofon4 = adc_read(ADC_CHANNEL_3);*/

	 //MessageLength = sprintf(DataToSend, "Mikrofon1: %d, Mikrofon2: %d, Mikrofon3: %d, Mikrofon4: %d\n\r", Mikrofon1, Mikrofon2, Mikrofon3, Mikrofon4);
	  //CDC_Transmit_FS((uint8_t*)DataToSend, MessageLength);



	  //HAL_ADC_Start(&hadc1);
	  //Mikrofon1 = HAL_ADC_GetValue(&hadc1);

	  //HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState OdczytanyStanPrzycisku;


	if(GPIO_Pin == Krancowka1_Pin)
	{
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);

		OdczytanyStanPrzycisku = HAL_GPIO_ReadPin( Krancowka1_GPIO_Port,  Krancowka1_Pin);


		if (OdczytanyStanPrzycisku != StanPrzycisku)
		{
		  StanPrzycisku = OdczytanyStanPrzycisku;

		  if (StanPrzycisku == GPIO_PIN_RESET)
		  {

			  TIM1->CCR1 = 0;
			  TIM1->CCR2 = 0;

		  }

		}

		PoprzedniStanPrzycisku = OdczytanyStanPrzycisku;


		HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
