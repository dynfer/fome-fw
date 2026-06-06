#pragma once

#include <cstdint>

typedef struct hal_spi_driver SPIDriver;

struct G0DigitalState {
	uint8_t mode = 0;
	bool level = false;
	bool signalPresent = false;
	uint16_t frequencyHz = 0;
	uint16_t dutyPermille = 0;
};

struct G0OutputState {
	uint32_t frequencyHz = 0;
	uint16_t duty = 0;
};

using G0SpiOperation = bool (*)(SPIDriver&);

struct G0SentState {
	bool ready = false;
	bool valid = false;
	uint32_t data = 0;
	uint8_t status = 0;
	uint8_t crc = 0;
	uint8_t nibbleCount = 0;
};

void initG0Io();

bool g0ReadVersion(uint32_t& version);
bool g0RequestEnterUpdateMode();
bool g0ExecuteSpiOperation(G0SpiOperation operation);

void g0SuspendIo();
void g0ResumeIo();

bool g0SetInputMode(uint8_t input, uint8_t mode);
bool g0SetOutput(uint8_t output, uint32_t frequencyHz, uint16_t duty);
bool g0DisableOutput(uint8_t output);

bool g0GetDigitalState(uint8_t input, G0DigitalState& state);
bool g0GetOutputState(uint8_t output, G0OutputState& state);
bool g0ReadSentState(uint8_t input, G0SentState& state);
