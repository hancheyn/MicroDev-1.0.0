/*
 * setup.c
 *
 *  Created on: May 2, 2022
 *      Author: nathan
 */
#include "main.h"
#include "setup.h"


//Basic Delay
void delay(int n) {
	int i;
	for(; n > 0; n--) {
		for(i =0; i < 3195; i++);
	}

}

struct pin pin_set(uint32_t pin, uint32_t clock, GPIO_TypeDef * gpio,  uint8_t pin_id) {

	struct pin P;
	P.pin = pin;
	P.pin_id = pin_id;

	P.clock = clock;
	P.GPIO = gpio;


	return P;
}


//Initialize pin struct array
void init_pins(struct pin pins[]) {

	//Pin 0 Example | PA5
	pins[0] = pin_set(0x05, 0x01, GPIOA, 0);

	//Pin 1 | PD2
	pins[1] = pin_set(0x02, 0x08, GPIOD, 1);

	//Pin 2 | PC12
	pins[2] = pin_set(0x0C, 0x04, GPIOC, 2);

	//Pin 3 | PC11
	pins[3] = pin_set(0x0B, 0x04, GPIOC, 3);

	//Pin 4 | PC10
	pins[4] = pin_set(0x0A, 0x04, GPIOC, 4);

	//Pin 5 | PB8
	pins[5] = pin_set(0x08, 0x02, GPIOB, 5);

	//Pin 6 | PC6
	pins[6] = pin_set(0x06, 0x04, GPIOC, 6);

	//Pin 7 | PC9
	pins[7] = pin_set(0x09, 0x04, GPIOC, 7);

	//Pin 8 | PC8
	pins[8] = pin_set(0x08, 0x04, GPIOC, 8);

	//Pin 9 | REF
	pins[9] = pin_set(0x00, 0x00, GPIOA, 9);

	//Pin 10 | BOOT0
	pins[10] = pin_set(0x00, 0x00, GPIOA, 10);

	//Pin 11 | E5V
	pins[11] = pin_set(0x00, 0x00, GPIOA, 11);

	//Pin 12 | VDD
	pins[12] = pin_set(0x00, 0x00, GPIOA, 12);

	//Pin 13 | AVDD
	pins[13] = pin_set(0x00, 0x00, GPIOA, 13);

	//Pin 14 | U5V
	pins[14] = pin_set(0x00, 0x00, GPIOA, 14);

	//Pin 15 | PB9
	pins[15] = pin_set(0x09, 0x04, GPIOB, 15);

	//Pin 16 | PC5
	pins[16] = pin_set(0x05, 0x04, GPIOC, 16);

	//Pin 17 | PA14
	pins[17] = pin_set(0x00, 0x01, GPIOA, 17);

	//Pin 18 | 3V3
	pins[18] = pin_set(0x00, 0x00, GPIOA, 18);

	//Pin 19 | PA13
	pins[19] = pin_set(0x00, 0x01, GPIOA, 19);


	//Pin 20 | RESET
	pins[20] = pin_set(0x00, 0x00, GPIOA, 20);

	//Pin 21 | PA6
	pins[21] = pin_set(0x06, 0x01, GPIOA, 21);

	//Pin 22 | PA11
	pins[22] = pin_set(0x0B, 0x01, GPIOA, 22);

	//Pin 23 | PA5
	pins[23] = pin_set(0x05, 0x01, GPIOA, 23);

	//Pin 24 | PA12
	pins[24] = pin_set(0x0C, 0x01, GPIOA, 24);

	//Pin 25 | PC13
	pins[25] = pin_set(0x0D, 0x04, GPIOC, 25);

	//Pin 26 | PB7
	pins[26] = pin_set(0x07, 0x02, GPIOB, 26);

	//Pin 27 | PA15
	pins[27] = pin_set(0x0F, 0x01, GPIOA, 27);

	//Pin 28 | 5V
	pins[28] = pin_set(0x00, 0x00, GPIOA, 28);

	//Pin 29 | PB2
	pins[29] = pin_set(0x02, 0x02, GPIOB, 29);

	//Pin 30 | PB6
	pins[30] = pin_set(0x06, 0x02, GPIOB, 30);

	//Pin 31 | PB12
	pins[31] = pin_set(0x0C, 0x02, GPIOB, 31);

	//Pin 32 | PA7
	pins[32] = pin_set(0x07, 0x01, GPIOA, 32);

	//Pin 33 | PC15
	pins[33] = pin_set(0x0F, 0x04, GPIOC, 33);

	//Pin 34 | PA0
	pins[34] = pin_set(0x00, 0x01, GPIOA, 34);

	//Pin 35 | PC14
	pins[35] = pin_set(0x0E, 0x04, GPIOC, 35);

	//Pin 36 | VIN
	pins[36] = pin_set(0x00, 0x00, GPIOA, 36);

	//Pin 37 | PA8
	pins[37] = pin_set(0x08, 0x01, GPIOA, 37);

	//Pin 38 | PB1
	pins[38] = pin_set(0x01, 0x02, GPIOB, 38);

	//Pin 39 | PA9
	pins[39] = pin_set(0x09, 0x01, GPIOA, 39);


	//Pin 40 | PC7
	pins[40] = pin_set(0x07, 0x04, GPIOC, 40);

	//Pin 41 | PH1
	pins[41] = pin_set(0x00, 0x00, GPIOA, 41);

	//Pin 42 | PA4
	pins[42] = pin_set(0x04, 0x01, GPIOA, 42);

	//Pin 43 | PH0
	pins[43] = pin_set(0x00, 0x00, GPIOA, 43);

	//Pin 44 | PA1
	pins[44] = pin_set(0x01, 0x01, GPIOA, 44);

	//Pin 45 | PB4
	pins[45] = pin_set(0x04, 0x02, GPIOB, 45);

	//Pin 46 | PB14
	pins[46] = pin_set(0x0E, 0x02, GPIOB, 46);

	//Pin 47 | PB10
	pins[47] = pin_set(0x0A, 0x02, GPIOB, 47);

	//Pin 48 | PB15
	pins[48] = pin_set(0x0F, 0x02, GPIOB, 48);

	//Pin 49 | PC2
	pins[49] = pin_set(0x02, 0x04, GPIOC, 49);

	//Pin 50 | PC1
	pins[50] = pin_set(0x01, 0x04, GPIOC, 50);

	//Pin 51 | VBAT
	pins[51] = pin_set(0x00, 0x00, GPIOA, 51);

	//Pin 52 | PB0
	pins[52] = pin_set(0x00, 0x02, GPIOB, 52);

	//Pin 53 | PB3
	pins[53] = pin_set(0x03, 0x02, GPIOB, 53);

	//Pin 54 | AGND
	pins[54] = pin_set(0x00, 0x00, GPIOA, 54);

	//Pin 55 | PB5
	pins[55] = pin_set(0x05, 0x02, GPIOB, 55);

	//Pin 56 | PB13
	pins[56] = pin_set(0x0D, 0x02, GPIOB, 56);

	//Pin 57 | PC3
	pins[57] = pin_set(0x03, 0x04, GPIOC, 57);

	//Pin 58 | PC0
	pins[58] = pin_set(0x00, 0x04, GPIOC, 58);

	//Pin 59 | PA3
	pins[59] = pin_set(0x00, 0x01, GPIOA, 59);

	//Pin 60 | PA2
	pins[60] = pin_set(0x00, 0x01, GPIOA, 60);

	//Pin 61 | PA10
	pins[61] = pin_set(0x0A, 0x01, GPIOA, 61);

	//Pin 62 | PC4
	pins[62] = pin_set(0x04, 0x04, GPIOC, 62);

}





void init_facade() {


}


void init_real() {


}
