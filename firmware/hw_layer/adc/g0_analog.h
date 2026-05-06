#pragma once

#include "global.h"

#if HW_ATLAS && HAL_USE_ADC && HAL_USE_SPI

bool updateG0AnalogInputs();
void startG0AnalogInputs();
bool getG0AnalogInputAsAdc(adc_channel_e channel, adcsample_t& sample);

#endif // HW_ATLAS && HAL_USE_ADC && HAL_USE_SPI
