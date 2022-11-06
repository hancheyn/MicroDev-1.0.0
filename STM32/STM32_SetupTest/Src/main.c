/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: Nathan Hanchey
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
#include "setup.h"


/* Private includes ----------------------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);

// SERIAL
int command_read(unsigned char data[]);
int command_write(unsigned int pin, unsigned int result, unsigned int test);
int crc_encode(unsigned char data[], unsigned int pin, unsigned int result, unsigned int test);
int crc_decode(unsigned char data[]);


//IO SIGNALS
struct pin PINS_[64];
int analogRead(int pin);
int digitalRead(int pin);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int logic);
void pinMode(int pin, int mode);


//FIX AFTER ARDUINO PORTION
void run_tests(unsigned char data[]);
int sleepmode(int mode);

//https://thekurks.net/blog/2018/1/24/guide-to-arduino-sleep-mode
//void attachInterrupt();
//void set_sleep_mode();
//void sleep_cpu();

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();

	//IO INITs
	init_pins(PINS_);
	// pinMode(0, OUTPUT);
	// digitalWrite(0, HIGH);

	sleepmode(3);

  	//GPIOC Input
  	RCC->AHB1ENR |= 4; //GPIOC clock
  	GPIOC->MODER &= ~0x0C000000;

    while (1) {
  	  if(GPIOC->IDR & 0x2000) {
  		  //GPIOA->ODR |= 0x00000020; //turn on
  		  //digitalWrite(0, LOW);
  		  //delay(500);

  		    uint8_t RMSG[4] = {'\0'};

  		    //Write Test
  			command_read(RMSG);
  			delay(50);	//delay is important

  			if(crc_decode(RMSG)){
  				//GPIOA->ODR &= ~0x00000020;
  				//digitalWrite(0, LOW);

  				//Interpret Instructions
  				run_tests(RMSG);

  				//Send Back Results
  				command_write(RMSG[0], RMSG[1], RMSG[2]);
  			}
  	  }
    }

  /* USER CODE END 3 */
}


/********************************************************/
//Tests
/********************************************************/
void run_tests(unsigned char data[]) {

	unsigned char pin = data[0];
	unsigned char instruction = data[1];

	// Test #1
	if(data[2] == 1) {
		if(instruction == 1) {
			pinMode(pin, OUTPUT);
			digitalWrite(pin, 1);
		}
		else if(instruction == 0) {
			pinMode(pin, OUTPUT);
			digitalWrite(pin, 0);
		}

	}
	else if(data[2] == 2) {

	}
	else if(data[2] == 8) {

	}

}



/********************************************************/
//IO PINS

//
int sleepmode(int mode) {

	//SLEEP == 1
	if(mode == 1) {
		HAL_SuspendTick();
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		HAL_PWR_EnableSleepOnExit();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
	}
	//STOP == 2
	else if(mode == 2) {
		HAL_SuspendTick();
		HAL_PWR_EnableSleepOnExit();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
	}
	//STANDBY == 4
	else if(mode == 4) {
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		//__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
		HAL_PWR_EnterSTANDBYMode();
	}

	return 0;
}


int analogRead(int pin) { //!IN PROGRESS [NEED TO DEVELOP DEBUGGING METHOD]

	int result;
	//SET REGESTERS p213

	//Pin Setup
	RCC->AHB1ENR |= PINS_[pin].clock;  //clock for pin
	PINS_[pin].GPIO->MODER |= (0x03 << (PINS_[pin].pin * 2)); // Set to ADC mode

	//ADC1
	RCC->APB2ENR |= 0x00000100; /* Enable ADC1 clock */
	/* Setup for ACD1 */
	ADC1->CR2 = 0;
	ADC1->SQR3 = 1;
	ADC1->SQR1 = 0;
	ADC1->CR2 = 1;

	//FIND VALUE in ADC1
	ADC1->CR2 |= 0x40000000;
	while(!(ADC1->SR & 2)) {}
	result = ADC1->DR;

	return result;
}

void pinMode(int pin, int mode) {

	if(mode == INPUT_PULLUP) {
		RCC->AHB1ENR |= PINS_[pin].clock;
		PINS_[pin].GPIO->MODER &= ~(0x03 << (PINS_[pin].pin * 2)); 		/* Clear Mode to Input */
		PINS_[pin].GPIO->PUPDR &= ~(0x03 << (PINS_[pin].pin * 2)); 	/* Enable Pull-up resister by setting bit 0x01 */
		PINS_[pin].GPIO->PUPDR |= (0x01 << (PINS_[pin].pin * 2));
	}
	else if(mode == INPUT_PULLDOWN) {
		RCC->AHB1ENR |= PINS_[pin].clock;
		PINS_[pin].GPIO->MODER &= ~(0x03 << (PINS_[pin].pin * 2)); 		/* Clear Mode to Input */
		PINS_[pin].GPIO->PUPDR &= ~(0x03 << (PINS_[pin].pin * 2)); 	/* Enable Pull-down resister by setting bit 0x02 */
		PINS_[pin].GPIO->PUPDR |= (0x02 << (PINS_[pin].pin * 2));
	}
	else if(mode == OUTPUT) {
		RCC->AHB1ENR |= PINS_[pin].clock; 							/* Enable Port Clock */
		(PINS_[pin].GPIO)->MODER &= ~(0x03 << (PINS_[pin].pin * 2)); 		/* Clear Mode */
		(PINS_[pin].GPIO)->MODER |= (0x01 << (PINS_[pin].pin * 2));			/* Set Mode to Output */
	}
	else {
		RCC->AHB1ENR |= PINS_[pin].clock;
		PINS_[pin].GPIO->MODER &= ~(0x03 << (PINS_[pin].pin * 2)); 		/* Clear Mode to Input */
		PINS_[pin].GPIO->PUPDR &= ~(0x03 << (PINS_[pin].pin * 2)); 	/* Enable Floating by setting bit 0x01 */
	}

}

int digitalRead(int pin_num) {

	//RCC->AHB1ENR |= PINS_[pin_num].clock;
	//PINS_[pin_num].GPIO->MODER &= ~PINS_[pin_num].pin_clear_mode; 	/* Clear Mode to Input */
	//PINS_[pin_num].GPIO->PUPDR |= 0x00 << (PINS_[pin_num].pin * 2); 	/* Enable Pull-up resister by setting bit 0x01 */

	int out = 0;

 	if(PINS_[pin_num].GPIO->IDR & PINS_[pin_num].pin) {
 		out = 1;
 	}

	return out;
}


void digitalWrite(int pin, int logic) {
	//RCC->AHB1ENR |= PINS_[pin].clock; 						/* Enable Port Clock */
	//PINS_[pin].GPIO->MODER &= ~PINS_[pin].pin_clear_mode; 	/* Clear Mode */
	//PINS_[pin].GPIO->MODER |= PINS_[pin].pin_out_mode;		/* Set Mode to Output */

	if(logic) {
		PINS_[pin].GPIO->BSRR |= 0x01 << PINS_[pin].pin;
	}
	else {
		PINS_[pin].GPIO->BSRR |= 0x01 << (PINS_[pin].pin + 16);
	}

}


/********************************************************/
//SERIAL
int command_read(unsigned char data[]) {
	HAL_UART_Receive(&huart2, data, 3, 10000);
	return 0;
}

int command_write(unsigned int pin, unsigned int result, unsigned int test) {

	//Write
	unsigned char data[3];

	crc_encode(data, pin, result, test);

	HAL_UART_Transmit(&huart2, data, 3, 100);

	return 0;
}

int crc_encode(unsigned char data[], unsigned int pin, unsigned int result, unsigned int test) {
	// Find the data
	unsigned long int crc_packet = ((pin << 16) & 0xFF0000) + ((result << 8) & 0xFF00) + ((test << 4) & 0xF0);

	// Find
	unsigned int remainder = crc_packet % 5;
	unsigned int crc = 5 - remainder;

	crc_packet += crc;

	data[0] = ((crc_packet >> 16)) & 0xFF;
	data[1] = ((crc_packet >> 8)) & 0xFF;
	data[2] = ((crc_packet) & 0xFF);

	return 0;
}


int crc_decode(unsigned char data[]) {

	// Find the data
	unsigned long int crc_packet = (((unsigned long int)data[0] << 16) & 0xFF0000)
			+ (((unsigned long int)data[1] << 8) & 0xFF00) + (((unsigned long int)data[2]));

	if(crc_packet % 5) {
		return 0;
	}

	data[2] = (data[2] & 0xF0) >> 4;

	return 1;
}


/***********************************************************/
/********* BASIC AUTO-GENERATED STM CONFIGURATIONS *********/
/***********************************************************/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}


void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

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

