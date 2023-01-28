/*
 * PicoStepper - Microstepping stepper library for the Raspberry Pi Pico, Version 0.1
 * Copyright (C) Christopher Mitchell, Ph.D.
 *
 * Based on Stepper library for Raspberry Pi Pico - Version 0.1
 * Copyright (C) Beshr Kayali
 *
 * Based on Arduino Stepper Library
 * Copyright (C) Arduino LLC. Copyright (C) Sebastian Gassner. Copyright (c) Noah Shibley.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <cstdlib>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "pico_stepper.hpp"

#include "stepper.pio.h"

const int MICROSTEPS = 8;
static int microstepping_store[4][MICROSTEPS][2];
static const int count0Offset = 2;				// Must decrease phase0 by this to be accurate
static const int count1Offset = 2;				// Must decrease phase0 by this to be accurate
static const int count2Offset = 4;				// Must decrease phase0 by this to be accurate
static const int maxPWM = 142 + count1Offset;	// Max sum of components: ceil(100. * sin(pi/4)) + ceil(100. * cos(pi/4))

/* 8 microsteps
 = {
	{ { 0, 100 },{ 20, 98 },{ 38, 92 },{ 56, 83 },{ 71, 71 },{ 83, 56 },{ 92, 38 },{ 98, 20 }, },
	{ { 100, 0 },{ 98, -20 },{ 92, -38 },{ 83, -56 },{ 71, -71 },{ 56, -83 },{ 38, -92 },{ 20, -98 } },
	{ { -0, -100 },{ -20, -98 },{ -38, -92 },{ -56, -83 },{ -71, -71 },{ -83, -56 },{ -92, -38 },{ -98, -20 }, },
	{ { -100, 0 },{ -98, 20 },{ -92, 38 },{ -83, 56 },{ -71, 71 },{ -56, 83 },{ -38, 92 },{ -20, 98 } }
}; */

PicoStepper::PicoStepper(PicoStepperConf conf) {
	size_t increments = 0;
	for(size_t phase = 0; phase < 4; ++phase) {
		for(size_t microstep = 0; microstep < MICROSTEPS; ++microstep, ++increments) {
			double angle = increments * (M_PI / 2. / MICROSTEPS);
			microstepping_store[phase][microstep][0] = (int)(100. * std::sin(angle));
			microstepping_store[phase][microstep][1] = (int)(100. * std::cos(angle));
		}
	}
	microstepping_store[2][0][0] = -0;
	
  this->total_steps = conf.total_steps * MICROSTEPS;
  this->pin1 = conf.pin1;
  this->pin2 = conf.pin2;
  this->pin3 = conf.pin3;
  this->pin4 = conf.pin4;
  this->current_step = 0;
  this->current_microstep = 0;
  this->dir = 0;
  this->last_step_us_time = 0;
  setSpeed(conf.initial_speed);

    static const float pio_freq = 1000000;

    // Choose PIO instance (0 or 1)
    pio_ = pio0;

    // Get first free state machine in PIO 0
    sm_ = pio_claim_unused_sm(pio_, true /* required */);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    uint offset = pio_add_program(pio_, &stepper_program);
	
    // Calculate the PIO clock divider
    float div = (float)clock_get_hz(clk_sys) / pio_freq;

    // Initialize the program using the helper function in our .pio file
    stepper_program_init(pio_, sm_, offset, this->pin1, div);

    // Start running our PIO program in the state machine
    pio_sm_set_enabled(pio_, sm_, true);

}

int PicoStepper::getMicrosteps() {
	return MICROSTEPS;
}

void PicoStepper::setSpeed(double speed) {
  this->delay = (long)(60. * 1000. * 1000. / this->total_steps / speed);
}

void PicoStepper::step(int steps_to_move) {
  int steps_left = abs(steps_to_move);

  if (steps_to_move > 0) {
    this->dir = 1;
  } else {
    this->dir = 0;
  }

  while (steps_left > 0) {
    uint64_t now = to_us_since_boot(get_absolute_time());

    if (now - this->last_step_us_time >= this->delay) {
      this->last_step_us_time = now;

      if (this->dir == 1) {
		this->current_microstep++;
		
		if (this->current_microstep == MICROSTEPS) {
			this->current_step++;
			this->current_microstep = 0;

			if (this->current_step == this->total_steps) {
			  this->current_step = 0;
			}
		}
      } else {
		if (this->current_microstep == 0) {
			this->current_microstep = MICROSTEPS;

			if (this->current_step == 0) {
			  this->current_step = this->total_steps;
			}

			this->current_step--;
		}
		
		this->current_microstep--;
      }

      stepMotor(this->current_step % 4, this->current_microstep);
      steps_left--;
    }
  }
}

void PicoStepper::stepMotor(int step, int microstep) {

  int coil0 = microstepping_store[step][microstep][0];
  int coil2 = microstepping_store[step][microstep][1];
  int phase0 = 0;
  int count0 = (coil0 >= 0 ? coil0 : -coil0);
  count0 = (count0 > 0) ? (count0 - count0Offset) : 0;

  int phase2 = 0;
  int count2 = coil2 >= 0 ? coil2 : -coil2;
  count2 = (count2 > 0) ? (count2 - count2Offset) : 0;

	if (coil0 > 0) {
		phase0 = 0b1000;
	} else if (coil0 < 0) {
		phase0 = 0b0100;
	}
	if (coil2 > 0) {
		phase2 = 0b0010;
	} else if (coil2 < 0) {
		phase2 = 0b0001;
	}
	
	const int phase1 = phase0 | phase2;
	int count1 = maxPWM - count0 - count2 - count1Offset;

	pio_sm_put(pio_, sm_, (count2 << 25) | (phase2 << 21) | (count1 << 15) | (phase1 << 11) | (count0 << 4) | phase0);
}