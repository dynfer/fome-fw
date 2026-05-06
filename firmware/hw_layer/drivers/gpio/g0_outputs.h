/*
 * Atlas G0 extension output bridge.
 */

#pragma once

#include "gpio/gpio_ext.h"

#ifndef BOARD_G0_OUTPUT_COUNT
#define BOARD_G0_OUTPUT_COUNT 0
#endif

#define G0_OUTPUTS 4

int g0_outputs_add(brain_pin_e base, unsigned int index);

struct hardware_pwm;

hardware_pwm* g0_outputs_tryInitPwm(const char* msg, brain_pin_e pin, float frequencyHz, float duty);
