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
  * References:
  * STM32 Arm Programming For Embedded Systems
  * Authors: Muhammad Ali Mazidi | Shujen Chen | Eshragh Ghaemi
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "setup.h"
#include <string.h>

#define CRC_KEY 5


/* Private includes ----------------------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);

// SERIAL
int command_read(unsigned char data[]);
int command_write(unsigned int pin, unsigned int result, unsigned int test);
int crc_encode(unsigned char data[], unsigned int pin, unsigned int result, unsigned int test);
int crc_decode(unsigned char data[]);

/* Config Function Prototypes */
void configure_output(unsigned int pin, unsigned int logic);
int configure_input(unsigned int pin);
void configure_input_pullup(unsigned int pin);
int configure_analog_input(unsigned int pin);
void configure_input_pulldown(unsigned int pin);
void configure_sleep_mode(unsigned int sleepmode, unsigned int interruptPin);
void wakeUp();

//IO SIGNALS
struct pin PINS_[64];
int analogRead(int pin);
int digitalRead(int pin);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int logic);
void pinMode(int pin, int mode);

//TESTS
void run_tests(unsigned char data[]);
void reset_pins();

//SLEEPMODES
int sleepmode(int mode);

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();

	//IO INITs
	init_pins(PINS_);

    while (1) {

    	//USART2->;
  	  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {

  		    //Write Test
    		uint8_t RMSG[3] = {0};
  			command_read(RMSG);
  			delay(50);	//delay is important

  			if(crc_decode(RMSG) && RMSG[0] > 0){

  				//Interpret Instructions
  				if (RMSG[1] < 128) {
  					run_tests(RMSG);
  				}
  				else {
  					RMSG[0] = PINS_[RMSG[0]].pin;
  				}

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
		// Reconfigure Previous Pins Setup
		reset_pins();

		configure_output(pin, instruction);
		command_write(data[0], data[1], data[2]);
	}
	else if(data[2] == 2) {
		// Reconfigure Previous Pins Setup
		reset_pins();

		configure_output(pin, instruction);
		command_write(data[0], data[1], data[2]);
	}
	else if(data[2] == 3) {
		// Reconfigure Previous Pins Setup
		reset_pins();

		configure_input_pullup(pin);
		command_write(data[0], data[1], data[2]);
	}
	else if(data[2] == 4) {
		// Reconfigure Previous Pins Setup
		reset_pins();

		configure_input_pulldown(pin);
		command_write(data[0], data[1], data[2]);

	}
	else if(data[2] == 5) {
		data[1] = configure_input(pin);
		command_write(data[0], data[1], data[2]);

		// Reconfigure Pin Setup

	}
	else if(data[2] == 6) {
		data[1] = configure_analog_input(pin);
		command_write(data[0], data[1], data[2]);

		// Reconfigure Pin Setup

	}
	else if(data[2] == 7) {
		if(instruction == 1) {
			configure_sleep_mode(1, pin);
		}
		else if(instruction == 2) {
			configure_sleep_mode(2, pin);
		}
		else if(instruction == 3) {
			configure_sleep_mode(4, pin);
		}

	}
	else {
		command_write(data[0], data[1], data[2]);
	}

}


/********************************************************/
// Value Getters
/********************************************************/
/*
 * Description: Configures GPIO pin as OUTPUT and turns the output to HIGH. Used for testing GPIO output voltage under load sourcing.
 * Accepts: unsigned int pin - the pin number to configure as OUTPUT
 *          unsigned int logic - HIGH or LOW logic
 * Returns: void
 */
void configure_output(unsigned int pin, unsigned int logic) {
    pinMode(pin, OUTPUT);
    if(logic) {
      digitalWrite(pin, HIGH);
    }
    else {
      digitalWrite(pin, LOW);
    }
    return;
}

/*
 * Description: Configures GPIO pin as an INPUT. Used for testing input logic levels. The input pin cannot be a pullup,
 * as that would allow the pin to act as a current source and could damage the testing device's DAC.
 * Accepts: unsigned int pin - the pin number to configure as INPUT
 * Returns: int - 0 or 1 depending on input voltage of the pin (LOGIC LOW OR HIGH)
 */
int configure_input(unsigned int pin) {
    pinMode(pin, INPUT);
    return digitalRead(pin);
}

/*
 * Description: Configures GPIO pin as INPUT_PULLUP. Used for testing the pin's unloaded pullup voltage and internal
 * resistance value.
 * Accepts: unsigned int pin - the pin number to configure as INPUT_PULLUP
 * Returns: void
 */
void configure_input_pullup(unsigned int pin) {
    pinMode(pin,INPUT_PULLUP);
    return;
}

/*
 * Description: Configures GPIO pin as INPUT_PULLDOWN. Used for testing the pin's unloaded pullup voltage and internal
 * resistance value.
 * Accepts: unsigned int pin - the pin number to configure as INPUT_PULLDOWN
 * Returns: void
 */
void configure_input_pulldown(unsigned int pin) {
    pinMode(pin,INPUT_PULLDOWN);
    return;
}


/*
 * Description: Returns the analog reading of the selected analog pin (A0, A1, ..., A5). Used for testing the Arduino's
 * ADC.
 * Accepts: unsigned int analogPin - the analog pin number to read
 * Returns: int - 0 to 1023, depending on the voltage reading of the ADC. (0 = GND, 1023 = 5V)
 */
int configure_analog_input(unsigned int analogPin) {
	pinMode(analogPin, INPUT);
   return (analogRead(analogPin) >> 4); //returns a value 0- (0=GND,  = 3V3)
}




/********************************************************/
//IO  Sleep Modes / PINS
/********************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
	SystemClock_Config();
	HAL_PWR_DisableSleepOnExit();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_ResumeTick();
}

//
void configure_sleep_mode(unsigned int mode, unsigned int interruptPin) {

	// SLEEP == 1
	MX_GPIO_Init();
	SystemClock_Config();
	// Wake Up = ?PA0 | D13
	if(mode == 1) {
		HAL_PWR_EnableSEVOnPend();
		wakeUp(interruptPin);
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		HAL_ResumeTick();
	}
	// STOP == 2
	// Wake Up = Reset
	else if(mode == 2) {
		HAL_PWR_EnableSEVOnPend();
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		//wakeUp(interruptPin);
		HAL_SuspendTick();
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

		HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
	}
	// STANDBY == 4
	// Wake up Pin1 = PA0 | Pin2 = PC13
	else if(mode == 4) {
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		if(interruptPin == 1) {
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
		}
		//else if (interruptPin == 2) {
		//	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
		//}
		else {
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
		}
		HAL_PWR_EnterSTANDBYMode();
	}

}

// EXTI Interrupt Function
void wakeUp(int pin) {
		__disable_irq();
		// Pin A0
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		/*Configure GPIO pin : PA0 */
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* EXTI interrupt init*/
		HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		__enable_irq();
}


void EXTI15_10_IRQHandler(void) {
	if(EXTI->PR == 0x2000) {
		HAL_ResumeTick();
		EXTI->PR = 0x2000;
	}
	else if(EXTI->PR == 0x0001) {
		HAL_ResumeTick();
		EXTI->PR = 0x0001;
	}
	SystemClock_Config();
	HAL_PWR_DisableSleepOnExit();
	HAL_ResumeTick();
	EXTI->PR = 0x0001;
}


/********************************************************/
// Read & Write Functions
/********************************************************/

int analogRead(int pin) { //!IN PROGRESS [NEED TO DEVELOP DEBUGGING METHOD]

	int result, channel = 0;
	//SET REGESTERS p213

	//ANALOG MAPPING
	if(PINS_[pin].clock == 1) {
		channel = PINS_[pin].pin;
	}
	else if(PINS_[pin].clock == 2) {
		channel = PINS_[pin].pin + 8;
	}
	else if(PINS_[pin].clock == 4) {
		channel = PINS_[pin].pin + 10;
	}

	//Pin Setup
	RCC->AHB1ENR |= PINS_[pin].clock;  //clock for pin
	PINS_[pin].GPIO->MODER |= (0x03 << (PINS_[pin].pin * 2)); // Set to ADC mode

	//ADC1
	RCC->APB2ENR |= 0x00000100; /* Enable ADC1 clock */
	/* Setup for ACD1 */
	ADC1->CR2 = 0;
	ADC1->SQR3 = channel;
	ADC1->SQR1 = 0;
	ADC1->CR2 |= 1;

	//FIND VALUE in ADC1
	ADC1->CR2 |= 0x40000000;
	while(!(ADC1->SR & 2)) {}
	result = ADC1->DR;

	return result;
}

void pinMode(int pin, int mode) {

	if(mode == INPUT_PULLUP) { //P
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

 	if(PINS_[pin_num].GPIO->IDR & (0x01 << PINS_[pin_num].pin)) {
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

void reset_pins() {
	int i;
	for(i = 1; i < 63; i++) {
		configure_input(i);
	}
}

/********************************************************/
//SERIAL
/********************************************************/

int command_read(unsigned char data[]) {
	HAL_UART_Receive(&huart2, data, 3, 1000); //Changed from 10000 & while loop
	return 0;
}

int command_write(unsigned int pin, unsigned int result, unsigned int test) {
	//Write
	unsigned char data[3];
	crc_encode(data, pin, result, test);
	HAL_UART_Transmit(&huart2, data, 3, 1000); //changed from 100
	return 0;
}

int crc_encode(unsigned char data[], unsigned int pin, unsigned int result, unsigned int test) {
	// Find the data
	unsigned long int crc_packet = ((pin << 16) & 0xFF0000) + ((result << 8) & 0xFF00) + ((test << 4) & 0xF0);

	// Calculate CRC Number
	unsigned int remainder = crc_packet % CRC_KEY;
	unsigned int crc = CRC_KEY - remainder;

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

	if(crc_packet % CRC_KEY) {
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
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

	  /*Configure GPIO pin : PA0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : LD2_Pin */
	  GPIO_InitStruct.Pin = LD2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;	  /*Configure GPIO pin : PA0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : LD2_Pin */
	  GPIO_InitStruct.Pin = LD2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);


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
