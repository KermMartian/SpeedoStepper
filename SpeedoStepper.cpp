#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/pio.h"

#include "pico_stepper.hpp"

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// GPIO defines
// Example uses GPIO 2
#define GPIO PICO_DEFAULT_LED_PIN

const int MotorA1 = 14;
const int MotorA2 = 15;
const int MotorB1 = 16;
const int MotorB2 = 17;

int main()
{
    stdio_init_all();
    puts("Hello, world!");

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
	PicoStepperConf conf{.pin1 = MotorA1, .pin2 = MotorA2, .pin3 = MotorB1, .pin4 = MotorB2, .total_steps = 36, .initial_speed = 0.2};
	PicoStepper stepper(conf);
	while(1) {
		stepper.step(16);
		stepper.step(-16);
	}
    
    return 0;
}
