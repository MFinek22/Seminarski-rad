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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD_biblioteka.h"
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

uint8_t TurnOff_On = 0;						// Nalazi se na GPIOB, PIN 13
uint8_t NumUdaljenost = 0;					// Nalazi se na GPIOB, PIN 14
uint8_t StupUdaljenost = 0;					// Nalazi se na GPIOB, PIN 15
static uint16_t global_gpio_pin = 0;

#define TRIG_PIN GPIO_PIN_1
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
char snum[5];
char strUdaljenost[20] = {0};
char strTekstNum[20] = {0};
int znak = 0b11111111;
uint8_t barGraf[8] = {
        0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ledCrvena (void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim6, 1299);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	// pistalica
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)==1){
 	__HAL_TIM_SET_AUTORELOAD(&htim2, 300);
 	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 10);
	}
	else{
	__HAL_TIM_SET_AUTORELOAD(&htim2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}

}
void ledZuta (void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim6, 1999);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	// pistalica
 	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==1){
 	 	__HAL_TIM_SET_AUTORELOAD(&htim2, 300);
 	 	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5);
 		}
 	else{
 		__HAL_TIM_SET_AUTORELOAD(&htim2, 0);
 		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
 		}

}
void ledZelena (void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim6, 2999);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
 	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==1){
 	 	__HAL_TIM_SET_AUTORELOAD(&htim2, 300);
 	 	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3);
 		}
 	else{
 		__HAL_TIM_SET_AUTORELOAD(&htim2, 0);
 		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
 		}

}
void ledNula (void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
 	__HAL_TIM_SET_AUTORELOAD(&htim2, 0);
 	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

}
void signalizacijaNum (void){
    if(Distance<7){
    	LCD_Clear();
    	LCD_SetCursor(6,0);
    	LCD_PrintStr("POZOR");
    	LCD_SetCursor(6,1);
    	LCD_PrintStr("STOJ!");
    	ledCrvena();
    }
    else if(Distance<15){
    	LCD_Clear();
    	LCD_SetCursor(0,1);
    	LCD_PrintStr(strUdaljenost);
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost");
    	ledZuta();
    }
    else if(Distance<30){
    	LCD_Clear();
    	LCD_SetCursor(0,1);
    	LCD_PrintStr(strUdaljenost);
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost");
		ledZelena();
    }
    else{
    	LCD_Clear();
    	LCD_SetCursor(0,1);
    	LCD_PrintStr(strUdaljenost);
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost");
    	ledNula();
    }
}

void signalizacijaBar (void){
    if(Distance<7){
    	LCD_Clear();
    	LCD_SetCursor(3,0);
    	LCD_PrintStr("UPOZORENJE");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(7,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(8,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(9,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(10,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(11,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(12,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(13,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(14,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(15,1);
    	LCD_PrintSpecialChar(0xFF);
    	ledCrvena();
    }
    else if(Distance<10 && Distance>=8){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(7,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(8,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(9,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(10,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(11,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(12,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZuta();
    }
    else if(Distance<12 && Distance>=10){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(7,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(8,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(9,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(10,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(11,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZuta();
    }
    else if(Distance<14 && Distance>=12){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(7,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(8,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(9,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(10,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZuta();
    }
    else if(Distance<16 && Distance>=14){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(7,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(8,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(9,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZuta();
    }
    else if(Distance<16 && Distance>=14){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(7,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(8,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZuta();
    }
    else if(Distance<18 && Distance>=16){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(7,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else if(Distance<20 && Distance>=18){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(6,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else if(Distance<22 && Distance>=20){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(5,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else if(Distance<24 && Distance>=22){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(4,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else if(Distance<26 && Distance>=24){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(3,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else if(Distance<28 && Distance>=26){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(2,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else if(Distance<30 && Distance>=28){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
    	LCD_SetCursor(1,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else if(Distance<32 && Distance>=30){
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	LCD_SetCursor(0,1);
    	LCD_PrintSpecialChar(0xFF);
		ledZelena();
    }
    else{
    	LCD_Clear();
    	LCD_SetCursor(0,0);
    	LCD_PrintStr("Udaljenost stup:");
    	ledNula();
    }
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  LCD_Init(2);
  LCD_Clear();

  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(TurnOff_On %2 == 1){

		  // Senzor pocinje mjeriti - pocetak naredbi za senzor

		  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		  __HAL_TIM_SET_COUNTER(&htim1, 0);
		  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

		  pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		  // wait for the echo pin to go high
		  while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
		  Value1 = __HAL_TIM_GET_COUNTER (&htim1);

		  pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		  // wait for the echo pin to go low
		  while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
		  Value2 = __HAL_TIM_GET_COUNTER (&htim1);

		  Distance = (Value2-Value1)* 0.034/2;
		  HAL_Delay(50);

		  // Kraj naredbi za senzor

		  LCD_Clear();
		  if(NumUdaljenost == 0 && StupUdaljenost == 0){
			  LCD_SetCursor(0,0);
			  LCD_PrintStr("Senzor upaljen");
			  LCD_SetCursor(0,1);
			  LCD_PrintStr("Izaberite mod");
			  HAL_Delay(200);
		  }
		  else if(NumUdaljenost < StupUdaljenost){
			  signalizacijaBar();

		  }
		  else if(NumUdaljenost > StupUdaljenost){
			  sprintf(strUdaljenost,"%d cm",Distance);
			  signalizacijaNum();
		  }
	  }
	  else{
		  LCD_Clear();
		  LCD_SetCursor(0,0);
		  LCD_PrintStr("Upalite senzor!");
		  NumUdaljenost = 0;
		  StupUdaljenost = 0;
		  HAL_Delay(200);
	  }

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	global_gpio_pin = GPIO_Pin;
	if(global_gpio_pin == GPIO_PIN_13 || global_gpio_pin == GPIO_PIN_14 || global_gpio_pin == GPIO_PIN_15){
		__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
		HAL_TIM_Base_Start_IT(&htim7);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
		if(Distance<=7){
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
		}
		else if(Distance<15){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		}
		else if(Distance<30){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
		}
		else{
			HAL_TIM_Base_Stop_IT(&htim6);
		}
	}

	if(htim->Instance == TIM7){
		if(HAL_GPIO_ReadPin(GPIOB, global_gpio_pin)==0){
			if(global_gpio_pin == GPIO_PIN_13){
				TurnOff_On++;
			}
			else if(global_gpio_pin == GPIO_PIN_14){
				NumUdaljenost++;
				StupUdaljenost = 0;
			}
			else if(global_gpio_pin == GPIO_PIN_15){
				StupUdaljenost++;
				NumUdaljenost = 0;
			}
			else{
				HAL_TIM_Base_Stop_IT(&htim7);
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
