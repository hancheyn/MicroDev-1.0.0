/*
 * setup.h
 *
 */

#ifndef SETUP_H_
#define SETUP_H_

#include "main.h"

//END OF TEST

void delay(int n);

#define HIGH 1
#define LOW 0
#define INPUT 0
#define INPUT_PULLUP 1
#define INPUT_PULLDOWN 2
#define OUTPUT 3


//PIN STRUCT
struct pin {
	uint32_t pin;
	uint32_t pin_clear_mode;
	uint32_t pin_out_mode;
	uint32_t clock;
	GPIO_TypeDef * GPIO;
	//= ((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)); | GPIOA
	uint8_t pin_id;
};

struct pin pin_set(uint32_t pin, uint32_t clock, GPIO_TypeDef * gpio,  uint8_t pin_id);

//SERIAL STRUCT
struct serial {
	uint8_t pin;
	uint8_t instruction;
	uint8_t test;
};


//INITS
void init_pins(struct pin pins[]);
void init_facade();
void init_real();


//RESULT TESTS
void set_all_gpio_output(struct pin pins[], int num_pins);
void set_all_gpio_pull_ups(struct pin pins[], int num_pins);
void set_all_gpio_pull_downs(struct pin pins[], int num_pins);
void set_all_gpio_inputs(struct pin pins[], int num_pins);
void set_all_adc_inputs(struct pin pins[], int num_pins);
//void set_power_mode(struct pin pins[], int num_pins, struct sleep_mode);


//FACADE TESTS
void run_gpio_output_loading_test(struct serial);
void run_gpio_input_resistance_test(struct serial);
void run_gpio_input_pull_up_test(struct serial);
void run_gpio_input_pull_down_test(struct serial);
void run_gpio_input_logic_level_test(struct serial);
void run_adc_test(struct serial);
void run_power_mode_test(struct serial);


#endif /* SETUP_H_ */
