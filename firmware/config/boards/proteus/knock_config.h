/**
 * @file knock_config.h
 */

#pragma once

// Knock is on ADC3
#define KNOCK_ADC ADCD3

// knock 1 - pin PF4
#define KNOCK_ADC_CH1 ADC_CHANNEL_IN14
#define KNOCK_PIN_CH1 Gpio::F4

// knock 2 - pin PF5
#define KNOCK_HAS_CH2 true
#define KNOCK_ADC_CH2 ADC_CHANNEL_IN15
#define KNOCK_PIN_CH2 Gpio::F5

// knock 3 - pin PF3
#define KNOCK_HAS_CH3 true
#define KNOCK_ADC_CH3 ADC_CHANNEL_IN9
#define KNOCK_PIN_CH3 Gpio::F3

// knock 4 - pin PF6
#define KNOCK_HAS_CH4 true
#define KNOCK_ADC_CH4 ADC_CHANNEL_IN4
#define KNOCK_PIN_CH4 Gpio::F6

// Sample rate & time - depends on the exact MCU
#define KNOCK_SAMPLE_TIME ADC_SAMPLE_84
#define KNOCK_SAMPLE_RATE (STM32_PCLK2 / (4 * (84 + 12)))
