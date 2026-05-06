#include "pch.h"

#include "gpio/g0_outputs.h"

#if HW_ATLAS && HAL_USE_SPI && (BOARD_G0_OUTPUT_COUNT > 0)

#include "console_io.h"
#include "hardware.h"
#include "mpu_util.h"

namespace {

#define DRIVER_NAME "g0"

static constexpr spi_device_e G0_SPI_DEVICE = SPI_DEVICE_5;
static constexpr uint8_t G0_APP_CMD_READ_ANALOG = 0x10;
static constexpr uint8_t G0_APP_CMD_SET_OUTPUT = 0x20;
static constexpr uint8_t G0_APP_CMD_DISABLE_OUTPUT = 0x21;
static constexpr uint16_t G0_OUTPUT_DUTY_MAX = 10000;
static constexpr uint32_t G0_OUTPUT_DEFAULT_FREQUENCY_HZ = 1000;
static constexpr size_t G0_APP_FRAME_SIZE = 36;
static constexpr size_t G0_APP_HEADER_SIZE = 4;
static constexpr uint8_t G0_APP_STATUS_INVALID = 0xff;
static constexpr uint8_t G0_APP_STATUS_READY = 0x00;
static constexpr uint8_t G0_APP_STATUS_UPDATE_MODE = 0x01;
static constexpr uint8_t G0_APP_RESULT_INVALID = 0xff;
static constexpr uint8_t G0_APP_RESULT_OK = 0x00;
static constexpr int G0_OUTPUT_RETRY_INTERVAL_MS = 1;

static NO_CACHE uint8_t g0OutputTxBuffer[G0_APP_FRAME_SIZE];
static NO_CACHE uint8_t g0OutputRxBuffer[G0_APP_FRAME_SIZE];
static SEMAPHORE_DECL(g0OutputWake, 0);
static THD_WORKING_AREA(g0OutputThreadStack, 512);
static bool g0OutputThreadStarted = false;

static const char* g0OutputPinNames[G0_OUTPUTS] = {
		"G0 Lowside 1",
		"G0 Lowside 2",
		"G0 Lowside 3",
		"G0 Lowside 4",
};

static void printG0Outputs();

static const char* statusName(uint8_t status) {
	switch (status) {
	case G0_APP_STATUS_READY:
		return "ready";
	case G0_APP_STATUS_UPDATE_MODE:
		return "update";
	case G0_APP_STATUS_INVALID:
		return "none";
	default:
		return "?";
	}
}

static const char* resultName(uint8_t result) {
	switch (result) {
	case G0_APP_RESULT_OK:
		return "ok";
	case 0x01:
		return "invalid";
	case 0x02:
		return "range";
	case 0x03:
		return "busy";
	case G0_APP_RESULT_INVALID:
		return "none";
	default:
		return "?";
	}
}

static void putU16(uint8_t* buffer, size_t offset, uint16_t value) {
	buffer[offset] = static_cast<uint8_t>(value);
	buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
}

static void putU32(uint8_t* buffer, size_t offset, uint32_t value) {
	buffer[offset] = static_cast<uint8_t>(value);
	buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
	buffer[offset + 2] = static_cast<uint8_t>(value >> 16);
	buffer[offset + 3] = static_cast<uint8_t>(value >> 24);
}

static uint16_t readU16(const uint8_t* buffer, size_t offset) {
	return static_cast<uint16_t>(buffer[offset]) | (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

static uint32_t readU32(const uint8_t* buffer, size_t offset) {
	return static_cast<uint32_t>(buffer[offset]) | (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
		   (static_cast<uint32_t>(buffer[offset + 2]) << 16) | (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}

static bool canSendSynchronously() {
	return !port_is_isr_context() && port_irq_enabled(port_get_irq_status());
}

class G0Outputs : public GpioChip {
public:
	int init() override {
		if (!g0OutputThreadStarted) {
			chThdCreateStatic(g0OutputThreadStack, sizeof(g0OutputThreadStack), PRIO_GPIOCHIP, g0OutputThread, nullptr);
			addConsoleAction("g0_outputs", printG0Outputs);
			g0OutputThreadStarted = true;
		}

		return 0;
	}

	int setPadMode(size_t pin, iomode_t mode) override {
		if (pin >= G0_OUTPUTS) {
			return -1;
		}

		if ((mode & PAL_STM32_MODE_MASK) == PAL_STM32_MODE_INPUT) {
			return writePad(pin, false);
		}

		return 0;
	}

	int writePad(size_t pin, int value) override {
		if (pin >= G0_OUTPUTS) {
			return -1;
		}

		{
			chibios_rt::CriticalSectionLocker csl;

			m_state[pin] = value != 0;
			m_pwmFrequency[pin] = G0_OUTPUT_DEFAULT_FREQUENCY_HZ;
			m_pwmDuty[pin] = value ? G0_OUTPUT_DUTY_MAX : 0;
		}

		if (canSendSynchronously()) {
			if (sendOutput(pin, G0_OUTPUT_DEFAULT_FREQUENCY_HZ, value ? G0_OUTPUT_DUTY_MAX : 0)) {
				return 0;
			}
		}

		markDirty(pin);
		return 0;
	}

	int readPad(size_t pin) override {
		if (pin >= G0_OUTPUTS) {
			return -1;
		}

		return m_state[pin] ? 1 : 0;
	}

	bool setPwm(size_t pin, uint32_t frequencyHz, float duty) {
		if (pin >= G0_OUTPUTS) {
			return false;
		}

		duty = clampF(0, duty, 1);
		const uint16_t scaledDuty = static_cast<uint16_t>(duty * G0_OUTPUT_DUTY_MAX);

		{
			chibios_rt::CriticalSectionLocker csl;

			m_state[pin] = scaledDuty != 0;
			m_pwmFrequency[pin] = frequencyHz;
			m_pwmDuty[pin] = scaledDuty;
		}

		if (canSendSynchronously()) {
			if (sendOutput(pin, frequencyHz, scaledDuty)) {
				return true;
			}
		}

		markDirty(pin);
		return true;
	}

	void printState(size_t chipIndex) {
		bool state[G0_OUTPUTS];
		uint8_t dirtyPins;
		uint8_t lastStatus;
		uint8_t lastResult;
		uint8_t lastCommand;
		uint8_t lastOutput;
		uint32_t lastFrequency;
		uint16_t lastDuty;
		uint32_t sendCount;
		uint32_t failCount;
		uint8_t lastRx0;
		uint8_t lastRx1;
		uint8_t lastRx2;
		uint8_t lastRx3;
		uint8_t lastRx4;
		uint8_t lastRx5;
		uint8_t lastRx6;
		uint8_t lastRx7;

		{
			chibios_rt::CriticalSectionLocker csl;

			memcpy(state, m_state, sizeof(state));
			dirtyPins = m_dirtyPins;
			lastStatus = m_lastStatus;
			lastResult = m_lastResult;
			lastCommand = m_lastCommand;
			lastOutput = m_lastOutput;
			lastFrequency = m_lastFrequency;
			lastDuty = m_lastDuty;
			sendCount = m_sendCount;
			failCount = m_failCount;
			lastRx0 = m_lastRx[0];
			lastRx1 = m_lastRx[1];
			lastRx2 = m_lastRx[2];
			lastRx3 = m_lastRx[3];
			lastRx4 = m_lastRx[4];
			lastRx5 = m_lastRx[5];
			lastRx6 = m_lastRx[6];
			lastRx7 = m_lastRx[7];
		}

		efiPrintf(
				"G0 outputs chip %d desired=%d%d%d%d dirty=0x%02x sent=%d failed=%d",
				static_cast<int>(chipIndex),
				state[0] ? 1 : 0,
				state[1] ? 1 : 0,
				state[2] ? 1 : 0,
				state[3] ? 1 : 0,
				dirtyPins,
				static_cast<int>(sendCount),
				static_cast<int>(failCount));
		efiPrintf(
				"G0 outputs last rsp status=%s(0x%02x) result=%s(0x%02x) cmd=0x%02x output=%d freq=%d duty=%d raw=%02x %02x %02x %02x %02x %02x %02x %02x",
				statusName(lastStatus),
				lastStatus,
				resultName(lastResult),
				lastResult,
				lastCommand,
				static_cast<int>(lastOutput),
				static_cast<int>(lastFrequency),
				static_cast<int>(lastDuty),
				lastRx0,
				lastRx1,
				lastRx2,
				lastRx3,
				lastRx4,
				lastRx5,
				lastRx6,
				lastRx7);
	}

	bool updateOutputs() {
		uint8_t dirtyPins;
		bool desiredState[G0_OUTPUTS];
		uint32_t desiredFrequency[G0_OUTPUTS];
		uint16_t desiredDuty[G0_OUTPUTS];

		{
			chibios_rt::CriticalSectionLocker csl;

			dirtyPins = m_dirtyPins;
			m_dirtyPins = 0;
			memcpy(desiredState, m_state, sizeof(desiredState));
			memcpy(desiredFrequency, m_pwmFrequency, sizeof(desiredFrequency));
			memcpy(desiredDuty, m_pwmDuty, sizeof(desiredDuty));
		}

		for (size_t pin = 0; pin < G0_OUTPUTS; pin++) {
			if (dirtyPins & (1 << pin)) {
				uint16_t duty = desiredState[pin] ? desiredDuty[pin] : 0;
				if (!sendOutput(pin, desiredFrequency[pin], duty)) {
					markDirty(pin);
				}
			}
		}

		return hasDirtyPins();
	}

	void markDirty(size_t pin) {
		chibios_rt::CriticalSectionLocker csl;

		m_dirtyPins |= 1 << pin;
		wakeDriver();
	}

	bool hasDirtyPins() {
		chibios_rt::CriticalSectionLocker csl;

		return m_dirtyPins != 0;
	}

	bool sendOutput(size_t pin, uint32_t frequencyHz, uint16_t duty) {
		SPIDriver* spi = getSpiDevice(G0_SPI_DEVICE);
		if (spi == nullptr || spi->state != SPI_READY) {
			recordSpiFailure();
			return false;
		}

		memset(g0OutputTxBuffer, 0, sizeof(g0OutputTxBuffer));
		memset(g0OutputRxBuffer, 0, sizeof(g0OutputRxBuffer));

		g0OutputTxBuffer[0] = duty != 0 ? G0_APP_CMD_SET_OUTPUT : G0_APP_CMD_DISABLE_OUTPUT;
		g0OutputTxBuffer[1] = static_cast<uint8_t>(pin + 1);

		if (duty != 0) {
			putU32(g0OutputTxBuffer, 2, frequencyHz);
			putU16(g0OutputTxBuffer, 6, duty);
		}

		spiAcquireBus(spi);
		spiSelect(spi);
		spiExchange(spi, G0_APP_FRAME_SIZE, g0OutputTxBuffer, g0OutputRxBuffer);

		// Prime the G0 response pipeline with the command the analog poller expects next.
		// This transaction returns the response to the output command above.
		memset(g0OutputTxBuffer, 0, sizeof(g0OutputTxBuffer));
		memset(g0OutputRxBuffer, 0, sizeof(g0OutputRxBuffer));
		g0OutputTxBuffer[0] = G0_APP_CMD_READ_ANALOG;
		spiExchange(spi, G0_APP_FRAME_SIZE, g0OutputTxBuffer, g0OutputRxBuffer);

		spiUnselect(spi);
		spiReleaseBus(spi);

		recordResponse(g0OutputRxBuffer);
		return true;
	}

	void recordSpiFailure() {
		chibios_rt::CriticalSectionLocker csl;

		m_failCount++;
	}

	void recordResponse(const uint8_t* rx) {
		chibios_rt::CriticalSectionLocker csl;

		memcpy(m_lastRx, rx, sizeof(m_lastRx));
		m_sendCount++;
		m_lastStatus = rx[0];
		m_lastResult = rx[1];
		m_lastCommand = rx[2];

		if (rx[3] >= 7) {
			m_lastOutput = rx[G0_APP_HEADER_SIZE];
			m_lastFrequency = readU32(rx, G0_APP_HEADER_SIZE + 1);
			m_lastDuty = readU16(rx, G0_APP_HEADER_SIZE + 5);
		} else {
			m_lastOutput = 0;
			m_lastFrequency = 0;
			m_lastDuty = 0;
		}

		if (rx[1] != G0_APP_RESULT_OK) {
			m_failCount++;
		}
	}

	bool m_state[G0_OUTPUTS] = {};
	uint32_t m_pwmFrequency[G0_OUTPUTS] = {
			G0_OUTPUT_DEFAULT_FREQUENCY_HZ,
			G0_OUTPUT_DEFAULT_FREQUENCY_HZ,
			G0_OUTPUT_DEFAULT_FREQUENCY_HZ,
			G0_OUTPUT_DEFAULT_FREQUENCY_HZ,
	};
	uint16_t m_pwmDuty[G0_OUTPUTS] = {};
	uint8_t m_lastRx[G0_APP_FRAME_SIZE] = {};
	uint8_t m_dirtyPins = 0;
	uint8_t m_lastStatus = G0_APP_STATUS_INVALID;
	uint8_t m_lastResult = G0_APP_RESULT_INVALID;
	uint8_t m_lastCommand = 0;
	uint8_t m_lastOutput = 0;
	uint32_t m_lastFrequency = 0;
	uint16_t m_lastDuty = 0;
	uint32_t m_sendCount = 0;
	uint32_t m_failCount = 0;

	static THD_FUNCTION(g0OutputThread, arg);
	static void wakeDriver() {
		chibios_rt::CriticalSectionLocker csl;

		chSemSignalI(&g0OutputWake);
		if (!port_is_isr_context()) {
			chSchRescheduleS();
		}
	}
};

static G0Outputs chips[BOARD_G0_OUTPUT_COUNT];

class G0HardwarePwm : public hardware_pwm {
public:
	bool init(brain_pin_e brainPin, float frequencyHz, float duty) {
		const int offset = static_cast<int>(brainPin) - static_cast<int>(Gpio::G0_LS_1);
		if (offset < 0 || offset >= G0_OUTPUTS) {
			return false;
		}

		if (frequencyHz < 1) {
			return false;
		}

		m_pin = static_cast<size_t>(offset);
		m_frequencyHz = static_cast<uint32_t>(frequencyHz);

		return chips[0].setPwm(m_pin, m_frequencyHz, duty);
	}

	void setDuty(float duty) override {
		chips[0].setPwm(m_pin, m_frequencyHz, duty);
	}

	bool matches(brain_pin_e brainPin) const {
		const int offset = static_cast<int>(brainPin) - static_cast<int>(Gpio::G0_LS_1);
		return offset >= 0 && offset < G0_OUTPUTS && static_cast<size_t>(offset) == m_pin;
	}

	bool isUsed() const {
		return m_pin < G0_OUTPUTS;
	}

private:
	size_t m_pin = G0_OUTPUTS;
	uint32_t m_frequencyHz = G0_OUTPUT_DEFAULT_FREQUENCY_HZ;
};

static G0HardwarePwm g0HardPwms[G0_OUTPUTS];

THD_FUNCTION(G0Outputs::g0OutputThread, arg) {
	(void)arg;
	chRegSetThreadName(DRIVER_NAME " outputs");

	while (true) {
		(void)chSemWait(&g0OutputWake);

		while (true) {
			bool hasMoreWork = false;

			for (auto& chip : chips) {
				hasMoreWork |= chip.updateOutputs();
			}

			if (!hasMoreWork) {
				break;
			}

			chThdSleepMilliseconds(G0_OUTPUT_RETRY_INTERVAL_MS);
		}
	}
}

static void printG0Outputs() {
	for (size_t i = 0; i < BOARD_G0_OUTPUT_COUNT; i++) {
		chips[i].printState(i);
	}
}

} // namespace

int g0_outputs_add(brain_pin_e base, unsigned int index) {
	if (index >= BOARD_G0_OUTPUT_COUNT) {
		return -1;
	}

	int result = gpiochip_register(base, DRIVER_NAME, chips[index], G0_OUTPUTS);
	if (result < 0) {
		return result;
	}

	gpiochips_setPinNames(static_cast<brain_pin_e>(result), g0OutputPinNames);

	return result;
}

hardware_pwm* g0_outputs_tryInitPwm(const char* msg, brain_pin_e pin, float frequencyHz, float duty) {
	(void)msg;

	const int offset = static_cast<int>(pin) - static_cast<int>(Gpio::G0_LS_1);
	if (offset < 0 || offset >= G0_OUTPUTS) {
		return nullptr;
	}

	for (auto& pwm : g0HardPwms) {
		if (pwm.isUsed() && pwm.matches(pin)) {
			if (pwm.init(pin, frequencyHz, duty)) {
				return &pwm;
			}

			return nullptr;
		}
	}

	for (auto& pwm : g0HardPwms) {
		if (!pwm.isUsed()) {
			if (pwm.init(pin, frequencyHz, duty)) {
				return &pwm;
			}

			return nullptr;
		}
	}

	firmwareError("Run out of G0 hardware PWM devices!");
	return nullptr;
}

#else /* HW_ATLAS && HAL_USE_SPI && (BOARD_G0_OUTPUT_COUNT > 0) */

int g0_outputs_add(brain_pin_e base, unsigned int index) {
	(void)base;
	(void)index;
	return -1;
}

hardware_pwm* g0_outputs_tryInitPwm(const char*, brain_pin_e, float, float) {
	return nullptr;
}

#endif
