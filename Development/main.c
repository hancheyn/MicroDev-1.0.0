/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: Your Name
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

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
void reset_pins();

// IO SIGNALS
struct pin PINS_[64];
int analogRead(int pin);
int digitalRead(int pin);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int logic);
void pinMode(int pin, int mode);
void configure_sleep_mode(unsigned int sleepmode, unsigned int interruptPin);
void wakeUp();

//FIX AFTER ARDUINO PORTION
void run_tests(unsigned char data[]);


/* Private variables unique to Development Board ---------------------------------------------*/




/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

	/* FIX: Reset of all peripherals, Initializes the Flash interface and the Systick. */

	/* Initialize all configured peripherals */


	//IO INITs
	init_pins(PINS_);


    while (1) {

       
        if( /* FIX!! Serial Available */) {

            //Write Test
            uint8_t RMSG[3] = {0};
            command_read(RMSG);
            delay(50);	//delay is important

            if(crc_decode(RMSG) && RMSG[0] > 0){

                //Interpret Instructions
                if (RMSG[1] < 128) {
                    /* Begin Test */
                    run_tests(RMSG);
                }
                else {
                    /* Greater then 128 indicates facade test */
                    RMSG[0] = PINS_[RMSG[0]].pin;
                }

            }

        }
    */
    }

  /* USER CODE END */
}


/********************************************************/
//Tests
/********************************************************/
void run_tests(unsigned char data[]) {

	unsigned char pin = data[0];
	unsigned char instruction = data[1];

	// Test #1
	if(data[2] == 1) {

		reset_pins();
		configure_output(pin, instruction);
		command_write(data[0], data[1], data[2]);

	}
	else if(data[2] == 2) {
		reset_pins();
		configure_output(pin, instruction);
		command_write(data[0], data[1], data[2]);
	}
	else if(data[2] == 3) {
		reset_pins();
		configure_input_pullup(pin);
		command_write(data[0], data[1], data[2]);
	}
	else if(data[2] == 4) {
		reset_pins();
		configure_input_pulldown(pin);
		command_write(data[0], data[1], data[2]);
	}
	else if(data[2] == 5) {
		data[1] = configure_input(pin);
		command_write(data[0], data[1], data[2]);
	}
	else if(data[2] == 6) {
		data[1] = configure_analog_input(pin);
		command_write(data[0], data[1], data[2]);
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

/*FIX The digitalREAD!
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


/* FIX
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
void configure_sleep_mode(unsigned int mode, unsigned int interruptPin) {

    /* FIX: Establish Mode Value for each power mode to be tested */
	// SLEEP == 1
	// FIX: Establish Wake Up Pin = ?PA0 | ?D13
	if(mode == 1) {
        /* FIX: Configure Power Mode and Wakeup */
	}
	// STOP == 2
	else if(mode == 2) {
		/* FIX: Configure Power Mode and Wakeup  */
	}
	// STANDBY == 4
	else if(mode == 4) {
		/* FIX: Configure Power Mode and Wakeup */
	}

}

// EXTI Interrupt Function
void wakeUp(int pin) {
		__disable_irq();

		/* Initialize Wakeup if Helpful */
		__enable_irq();
}



/********************************************************/
// Read & Write Functions
/********************************************************/

int analogRead(int pin) {

	int result, channel = 0;
    
    /* Configure ADC based on pin value */

	return result;
}

void pinMode(int pin, int mode) {

	if(mode == INPUT_PULLUP) {
        /* FIX: Configure the specified GPIO Input with Pullup Resistance */
		
	}
	else if(mode == INPUT_PULLDOWN) {
		/* FIX: Configure the specified GPIO Input with Pulldown Resistance */

	}
	else if(mode == OUTPUT) {
		/* FIX: Configure the specified GPIO as an Output*/

	}
	else {
		/* FIX: Configure the specified GPIO Input as Floating */

	}

}

int digitalRead(int pin_num) {

	int out = 0;
    
    /* FIX: Configure a GPIO Input and read the value */
 	if( /* PINS_[pin_num].GPIO->IDR & (0x01 << PINS_[pin_num].pin) */ ) {
 		out = 1;
 	}

	return out;
}


void digitalWrite(int pin, int logic) {

	if(logic) {
		//PINS_[pin].GPIO->BSRR |= 0x01 << PINS_[pin].pin;
        /* FIX: Configure Logic High on specified pin*/
	}
	else {
		//PINS_[pin].GPIO->BSRR |= 0x01 << (PINS_[pin].pin + 16);
        /* FIX: Configure Logic Low on specified pin */
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
	//HAL_UART_Receive(&huart2, data, 3, 1000); 
    /* Configure Serial Read Command for 3 bytes */
	return 0;
}

int command_write(unsigned int pin, unsigned int result, unsigned int test) {
	//Write
	unsigned char data[3];
	crc_encode(data, pin, result, test);
	//HAL_UART_Transmit(&huart2, data, 3, 1000); 
    /* Configure Serial Write Command for 3 bytes */
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
/********* BASIC AUTO-GENERATED CONFIGURATIONS BELOW *******/
/***********************************************************/

