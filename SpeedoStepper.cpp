#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/pio.h"

#include "pico_stepper.hpp"

// GPIO defines
const int GPIO_AWS_BUTTON = 13;
const int MotorA1 = 14;
const int MotorA2 = 15;
const int MotorB1 = 16;
const int MotorB2 = 17;

int main() {
    stdio_init_all();

	// Debugging
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	// AWS button setup - pull up and look for gnd
	gpio_init(GPIO_AWS_BUTTON);
	gpio_set_dir(GPIO_AWS_BUTTON, GPIO_IN);
	gpio_pull_up(GPIO_AWS_BUTTON);
	
	// Stepper setup
	PicoStepperConf conf{.pin1 = MotorA1, .pin2 = MotorA2, .pin3 = MotorB1, .pin4 = MotorB2, .total_steps = 36, .initial_speed = 30};
	PicoStepper stepper(conf);
	stepper.step(-36 * stepper.getMicrosteps());
	stepper.setSpeed(60);
	
	// 13 mph per 4 major steps
	const float mphPerMicrostep = 13.16f / 4.f / (float)stepper.getMicrosteps();

	int curPos = 0;					// In microsteps - more precise than storing the current train speed
	float nextTrainSpeed = 0.0;
	char speedBuffer[10];
	char* speedBufferCur = &speedBuffer[0];
	
	while(1) {
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		if (0 == gpio_get(GPIO_AWS_BUTTON)) {
			gpio_put(PICO_DEFAULT_LED_PIN, 1);
			putc('A', stdout);
		}
		const int nextChar = getchar_timeout_us(0);
		if (PICO_ERROR_TIMEOUT != nextChar) {
			*speedBufferCur = (char)nextChar;
			if (';' == *speedBufferCur++) {
				// Parse the speed
				*speedBufferCur = '\0';
				speedBufferCur = &speedBuffer[0];
				sscanf(speedBuffer, "s%f;", &nextTrainSpeed);
				if (0.f <= nextTrainSpeed && 80.f >= nextTrainSpeed) {
					// Valid speed
					const int nextPos = (int)(nextTrainSpeed / mphPerMicrostep);
					if (nextPos != curPos) {
						stepper.step(nextPos - curPos);
						curPos = nextPos;
					}
				}
			}
		}
	}
    
    return 0;
}
