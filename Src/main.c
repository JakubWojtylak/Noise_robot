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
//Zmiana stanu przycisku2*****************************
GPIO_PinState StanPrzycisku2;
GPIO_PinState PoprzedniStanPrzycisku2 = GPIO_PIN_SET;
uint32_t PoprzedniCzasPrzycisku2;
//***************************************************

const uint16_t ProgiHalasu[] = { 50, 100, 150, 250, 300};
volatile uint8_t KtoryProgHalasu;
uint32_t adc[4];
uint32_t adc1,adc2,adc3,adc4;
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
  //670 -> obrot kola o 360 kolo r = 30mm
  //410 (moze 400) -> obrot robota o 90 stopni przelicznik 4,56 (moze 4,45)

  KtoryProgHalasu = 0;

  uint16_t WartoscMax = 0;
  uint8_t MikrofonMax = 0;
  uint16_t Tymczasowa = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/*
		OdczytanyStanPrzycisku = HAL_GPIO_ReadPin(Poziom_halasu_GPIO_Port, Poziom_halasu_Pin);

		if (OdczytanyStanPrzycisku != PoprzedniStanPrzycisku)
		{
			PoprzedniCzasPrzycisku = HAL_GetTick();
		}

		if ((HAL_GetTick() - PoprzedniCzasPrzycisku) > 10)
		{
			if (OdczytanyStanPrzycisku != StanPrzycisku)
			{
				StanPrzycisku = OdczytanyStanPrzycisku;

				if (StanPrzycisku == GPIO_PIN_RESET)
				{
					KtoryProgHalasu = (KtoryProgHalasu + 1)%5;
					PoprzedniCzasPrzycisku = HAL_GetTick();

				}
			}
		}

		PoprzedniStanPrzycisku = OdczytanyStanPrzycisku;*/



//	  if(adc[0] > ProgiHalasu[KtoryProgHalasu])
//	  {
//			obrot_prawo(1000, 615);
//					HAL_Delay(30);
//
//	  }
//	  else if(adc[1] > ProgiHalasu[KtoryProgHalasu])
//	  {
//			obrot_lewo(1000, 615);
//					HAL_Delay(30);
//
//	  }
//	  else if(adc[2] > ProgiHalasu[KtoryProgHalasu])
//	  {
//			obrot_lewo(1000, 205);
//					HAL_Delay(30);
//;
//	  }
//	  else if(adc[3] > ProgiHalasu[KtoryProgHalasu])
//	  {
//			obrot_prawo(1000, 205);
//
//	  }
//
//
//
//










//
	  __disable_irq();
	  	WartoscMax = 0;
	  	MikrofonMax = 0;
		adc1 = adc[0];
		adc2 = adc[1];

	  	adc3=adc[2];
	  	adc4=adc[3];

	  	//Tymczasowa = adc[0];
	  	if(adc1 > ProgiHalasu[KtoryProgHalasu] && WartoscMax < adc1)
		{
	  		WartoscMax = adc1;
	  		MikrofonMax = 1;
		}

	  	//Tymczasowa = adc[1];
	  	if(adc2 > ProgiHalasu[KtoryProgHalasu] && WartoscMax < adc2)
		{
	  		WartoscMax = adc2;
			MikrofonMax = 2;
		}

	  	//Tymczasowa = adc[2];
	  	if(adc3 > ProgiHalasu[KtoryProgHalasu] && WartoscMax < adc3)
		{
	  		WartoscMax = adc3;
			MikrofonMax = 3;
		}

	  	//Tymczasowa = adc[3];
	  	if(adc4 > ProgiHalasu[KtoryProgHalasu] && WartoscMax < adc4)
		{
	  		WartoscMax = adc4;
			MikrofonMax = 4;
		}


	  	switch(MikrofonMax)
	  	{
			case 1:
				obrot_prawo(1000, 615);
				HAL_Delay(30);
				break;

			case 2:
				obrot_lewo(1000, 615);
				HAL_Delay(30);
				break;

			case 3:
				obrot_lewo(1000, 205);
				HAL_Delay(30);
				break;

			case 4:
				obrot_prawo(1000, 205);
				HAL_Delay(30);
				break;

			default:
				break;
	  	}
	  	 __enable_irq();
//
//HAL_ADC_Start_DMA(&hadc1,adc,4);
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
	GPIO_PinState OdczytanyStanPrzycisku2;

	if(GPIO_Pin == Krancowka1_Pin)
	{
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);

		OdczytanyStanPrzycisku = HAL_GPIO_ReadPin( Krancowka1_GPIO_Port,  Krancowka1_Pin);



		  if (StanPrzycisku == GPIO_PIN_RESET)
		  {

			  TIM1->CCR1 = 0;
			  TIM1->CCR2 = 0;

		  }



		PoprzedniStanPrzycisku = OdczytanyStanPrzycisku;


		HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	}


	if(GPIO_Pin == PoziomHalasu_Pin)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

		OdczytanyStanPrzycisku2 = HAL_GPIO_ReadPin( PoziomHalasu_GPIO_Port,  PoziomHalasu_Pin);



		  if (StanPrzycisku2 == GPIO_PIN_RESET)
		  {
			  KtoryProgHalasu = (KtoryProgHalasu + 1)%5;

		  }



		PoprzedniStanPrzycisku2 = OdczytanyStanPrzycisku2;


		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
