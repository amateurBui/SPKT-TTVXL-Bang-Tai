
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
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
static void MX_GPIO_Init(void);
void Clock_Init(void);
void sendPulse(void);
void delay(uint16_t time);
void button();
void pause();
void Conveyor();
void USART(void);
void send_char (char data);
void send_string (char *str);
void Nutpause();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


volatile unsigned int result;
volatile unsigned int ini;
volatile unsigned int ini_led;
int sensor = 0;
int detect = 0;
int detect1 = 0;
int detect2 = 0;
int auto_end = 0;
int state1 = 1;
int state2 = 1;
int state3 = 1;
int state4 = 0;
int buttonStart = 1;
int buttonStop = 1;
int buttonRe = 1;
int state5 = 0;
int state6 = 0;
int timer_bangtai;
int timer_leddon;
int covat = 0;

int stateMotor = 1;
char auto_out[50];
char buffer[50];

char data_rx[10];
int count_rx = 0;

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	Clock_Init();
	USART();
	TIM2->DIER |= (1<<0);  //enable timer 2 interrupt
	NVIC->ISER[0] |= (1<<28);  //enable global interrupt for timer 2
	while (1)
		{
			/* USER CODE END WHILE */
			button();
			Conveyor();
			pause();
			Nutpause();
			/* USER CODE BEGIN 3 */
		}
  /* USER CODE END 3 */
}
void Clock_Init(void)
{
	RCC->AHB1ENR |= (1<<1) | (1<<4) | (1<<2) | (1<<3); //Enable GPIOB and GPIOE, GPIOC and GPIOD
	RCC->APB1ENR |= (1<<1);
	GPIOB->MODER &= ~(3<<6);  // EVERY F2 AND F4 NEED THIS LINE FOR RUNNING
//	GPIOB->MODER &= ~(3<<10);
//	GPIOB->MODER &= ~(3<<20);
	GPIOA->MODER |= (1<<14);
	GPIOB->MODER |= (1<<6) | (1<<10) | (1<<20) | (1<<0) | (1<<4);  //Enable B3 and B5 and B10 and B0 and B2 as output
	GPIOA->MODER &= ~(1<<30);  //Enable A15 as Input
	GPIOC->MODER &= ~(1<<12);  //Enable C6 as Input
	GPIOC->MODER &= ~(1<<14);  //Enable C7 as Input
	GPIOC->MODER &= ~(1<<16);  //Enable C8 as Input
	GPIOB->MODER &= ~(1<<8);   //Enable B4 as Input
	GPIOB->MODER &= ~(1<<10);  //Enable B5 as Input
	GPIOD->MODER &= ~(1<<22);  //Enable D11 as Input

	//GPIOB->OTYPER = 0; //Push-pull m;ode
	GPIOB->OSPEEDR = 0; //Low speed
	//GPIOB->PUPDR = 0; //No pull up, pull down
	GPIOB->ODR = 0xFFFF; //Reset all LEDs

//	GPIOE->MODER &= ~(3<<0);
//	GPIOE->MODER &= ~(3<<2);
	GPIOE->MODER |= (1<<4) | (1<<0); //Enable E0 and E2
	GPIOE->MODER |= (1<<2);   //Enable E1 as output
//	GPIOE->OTYPER = 0 ; //Push-pull mode
	GPIOA->OSPEEDR = 0; //Low speed
	GPIOB->OSPEEDR = 0; //Low speed
	GPIOC->OSPEEDR = 0; //Low speed
	GPIOD->OSPEEDR = 0; //Low speed
	GPIOE->OSPEEDR = 0; //Low speed
	//GPIOE->PUPDR = 0; //No pull up, pull down
	GPIOE->ODR &= ~(1<<2);

	//========Interrupt========//
	RCC->APB1ENR |= (1<<0);
	TIM2->PSC = 0;
	TIM2->ARR = 15999;  //1ms
	//TIMER3
	TIM3->PSC = 399;
	TIM3->ARR = 19999;
	TIM3->CR1 |= (1<<0);
	TIM3-> DIER |= (1<<0);
	NVIC->ISER[0] |= (1<<28);
}

void USART(void)
{
	GPIOA->MODER |= (1<<5); // PA2 - TX
	GPIOA->MODER |= (1<<7); // PA3  - RX
	GPIOA->AFR[0] |= (7<<8) | (7<<12); //enable AFRH/AFRL
	RCC->APB1ENR |= (1<<17); // clock USART2
	USART2->BRR = (104<<4) | (3<<0); // 9600-- 16MHz
	USART2->CR1 |= (1<<13) | (1<<3) | (1<<2) | (1<<5); //enable USART2 for working
	NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
	if(USART2->DR == ';')
	{
		if (strcmp(data_rx, "start") == 0)
		{
			stateMotor = 0;
		}

		if (strcmp(data_rx, "stop") == 0)
		{
				stateMotor = 1;
				timer_leddon = 2;
        TIM2->CR1 |= (1<<0);
		}
        sprintf(buffer, "Received command: %s \n", data_rx);//////////////////////////////////////////////
        send_string(buffer);
        count_rx = 0;
	}
	else
	{
		data_rx[count_rx] = USART2->DR;
		count_rx++;

	}
}

void send_char (char data)
{	while ((USART2->SR & (1<<6)) == 0)
	{}
	USART2->DR = data;
}

void send_string (char *str)
{
	while(*str) send_char(*str++);
}

//Function to make PB3 send a pulse
void PB3sendPulse(void)
{
	GPIOB->ODR |= (1<<3);
	GPIOB->ODR &= ~(1<<3);
}
void TIM2_IRQHandler (void)
{
	TIM2->SR &= ~(1<<0); // Clear interupt flat
	GPIOA->ODR ^= (1<<7); // toggle PA7
	GPIOB->ODR ^=(1<<2);  //toggle PB2
	ini++;
	ini_led++;

	if (timer_bangtai == 1){
		if (ini >= 8000){
			stateMotor = 0;
			timer_bangtai = 0;

			ini = 0;////////////////////////////////////////////////////////////////
			TIM2->CR1 &= ~(1<<0);/////////////////////////////////////////////////// Hoac set bien state7 = 1 va tat timer2 trong Conveyor
		}
	}
	if (timer_leddon ==2){
		if (ini_led >= 1000){
			GPIOB->ODR ^= (1<<2);
			ini_led = 0;
		}
	}
	if (timer_leddon ==1){
//		if (ini_led >= 500){
//			GPIOB->ODR ^= (1<<2);
//			GPIOA->ODR ^= (1<<7);
//			ini_led = 0;
//		}
	}
}
uint8_t pe11Pressed = 0;
uint8_t loopCounter = 0;
void TIM3_IRQHandler(void)
{
    TIM3->SR &= ~(1<<0);

    loopCounter++;
    if ((GPIOE->IDR & (1<<11)) == 0)
    {
        pe11Pressed = 1;
        TIM3->CR1 &= ~(1<<0);
        loopCounter = 0;
    }


    if (loopCounter >= 10)
    {
        if (!pe11Pressed)
        {
            GPIOB->ODR &= ~(1<<0);
            TIM3->CR1 &= ~(1<<0);
        }
        loopCounter = 0;
        pe11Pressed = 0;
    }
}
char spham=0;
void pause(){
	if ((GPIOD->IDR & (1<<13)) != 0){
		spham=1;
	}
	else if ((GPIOD->IDR & (1<<12)) != 0){
		spham=0;
	}
}
void Nutpause(){
	if ((GPIOC->IDR & (1<<7)) == 0){
		GPIOB->ODR |= (1<<0);
		if(spham==1){
			timer_bangtai=0;
		}
		else if (spham==0){
			timer_bangtai=1;
			TIM2->CR1 |= (1<<0);
			TIM3->CR1 |=(1<<0);

		}
	}
}
void button()
{
	//button START
	if (((GPIOD->IDR & (1<<11)) == 0) && state1 == 1)
	{
		buttonStart = 0;
		send_string("BT1 pressed \n");
		state1 = 0;
	}
	else if (((GPIOD->IDR & (1<<11)) == 0) && state1 == 0)
	{
		buttonStart = 1;
		send_string("BT1 not pressed \n");
		state1 = 1;
	}
	//button STOP
	if (((GPIOC->IDR & (1<<7)) == 0) && state2 == 1)
	{
		buttonStop = 0;
		send_string("BT2 pressed \n");
		state2 = 0;
	}
	else if (((GPIOC->IDR & (1<<7)) != 0) && state2 == 0)
	{
		buttonStop = 1;
		send_string("BT2 not pressed \n");
		state2 = 1;
	}
	//button REACTIVE
	if (((GPIOE->IDR & (1<<14)) == 0) && state3 == 1)
	{
		buttonRe = 0;
		send_string("BT3 pressed \n");
		state3 = 0;
	}
	else if (((GPIOE->IDR & (1<<14)) != 0) && state3 == 0)
	{
		buttonRe = 1;
		send_string("BT3 not pressed \n");
		state3 = 1;
	}
}

//Motor Part
void Conveyor()
{
	// Running Part
	GPIOB->ODR &= ~(1<<2); //turn on LED 2
	GPIOA->ODR |= (1<<7);  // turn off LED 1

	if ((buttonStop == 0) && (covat == 0))// PAUSE ko co vat
	{
		stateMotor = 1;
		send_string("PAUSED VINH VIEN\n");
	}
	else if ((buttonStop == 0) && (covat == 1) && (state6 == 0))// PAUSE co vat
	{
		stateMotor = 1;
		timer_bangtai = 1;
		timer_leddon = 1;
		TIM2->CR1 |= (1<<0);
		send_string("PAUSED \n");
		state6 = 1;
	}
//	else if (((GPIOC->IDR & (1<<7)) == 0) && detect == 1)
//	{
//		stateMotor = 1;
//		send_string("PAUSED \n");
//		detect = 0;
//	}
	else if (((GPIOE->IDR & (1<<14)) != 0) && (buttonStop == 0))//////////////////////////////////////////////////////////////////////////////////
	{
		stateMotor = 0;
		send_string("REACTIVE \n");
		buttonStop = 1;
		state2 = 1;
		state6 = 0;
		TIM2->CR1 &= ~(1<<0);
	}

	if(stateMotor == 0)
	{
		GPIOB->ODR &= ~(1<<0);
	}
	else if (stateMotor == 1)
	{
		GPIOB->ODR |= (1<<0);
	}

//	// Counting Part
	if (((GPIOD->IDR & (1<<13)) != 0) && (buttonStart == 0))
	{
		sensor = 1;
		stateMotor = 0;
		detect1 = 1;
		state5 = 1;
		covat = 1;
		TIM2->CR1 &= ~ (1<<0);
	}
	else if (((GPIOD->IDR & (1<<13)) == 0) && (detect1 == 1))
	{
		detect1 = 0;
	}
	else if (((GPIOD->IDR & (1<<12)) != 0) && (state5 == 1))
	{
		stateMotor = 1;
		state5 = 0;
		covat = 0;
		auto_end++;
		sprintf(auto_out, "Number:  %d\n", auto_end); // int to string
		send_string(auto_out);
		timer_leddon = 1;
		TIM2->CR1 |= (1<<0);
	}


}


void delay(uint16_t time)
{
	while(time>0) time--;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
