#include "pch.h"

#include "gpio/g0_outputs.h"

#if HW_ATLAS && HAL_USE_SPI && (BOARD_G0_OUTPUT_COUNT > 0)

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
static constexpr int G0_OUTPUT_POLL_INTERVAL_MS = 100;

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

class G0Outputs : public GpioChip {
public:
	int init() override {
		if (!g0OutputThreadStarted) {
			chThdCreateStatic(g0OutputThreadStack, sizeof(g0OutputThreadStack), PRIO_GPIOCHIP, g0OutputThread, nullptr);
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
			m_dirtyPins |= 1 << pin;
		}

		wakeDriver();
		return 0;
	}

	int readPad(size_t pin) override {
		if (pin >= G0_OUTPUTS) {
			return -1;
		}

		return m_state[pin] ? 1 : 0;
	}

private:
	void updateOutputs() {
		uint8_t dirtyPins;
		bool desiredState[G0_OUTPUTS];

		{
			chibios_rt::CriticalSectionLocker csl;

			dirtyPins = m_dirtyPins;
			m_dirtyPins = 0;
			memcpy(desiredState, m_state, sizeof(desiredState));
		}

		for (size_t pin = 0; pin < G0_OUTPUTS; pin++) {
			if (dirtyPins & (1 << pin)) {
				if (!sendOutput(pin, desiredState[pin])) {
					markDirty(pin);
				}
			}
		}
	}

	void markDirty(size_t pin) {
		chibios_rt::CriticalSectionLocker csl;

		m_dirtyPins |= 1 << pin;
	}

	bool sendOutput(size_t pin, bool value) {
		SPIDriver* spi = getSpiDevice(G0_SPI_DEVICE);
		if (spi == nullptr || spi->state != SPI_READY) {
			return false;
		}

		memset(g0OutputTxBuffer, 0, sizeof(g0OutputTxBuffer));
		memset(g0OutputRxBuffer, 0, sizeof(g0OutputRxBuffer));

		g0OutputTxBuffer[0] = value ? G0_APP_CMD_SET_OUTPUT : G0_APP_CMD_DISABLE_OUTPUT;
		g0OutputTxBuffer[1] = static_cast<uint8_t>(pin + 1);

		if (value) {
			putU32(g0OutputTxBuffer, 2, G0_OUTPUT_DEFAULT_FREQUENCY_HZ);
			putU16(g0OutputTxBuffer, 6, G0_OUTPUT_DUTY_MAX);
		}

		spiAcquireBus(spi);
		spiSelect(spi);
		spiExchange(spi, G0_APP_FRAME_SIZE, g0OutputTxBuffer, g0OutputRxBuffer);

		// Prime the G0 response pipeline with the command the analog poller expects next.
		memset(g0OutputTxBuffer, 0, sizeof(g0OutputTxBuffer));
		memset(g0OutputRxBuffer, 0, sizeof(g0OutputRxBuffer));
		g0OutputTxBuffer[0] = G0_APP_CMD_READ_ANALOG;
		spiExchange(spi, G0_APP_FRAME_SIZE, g0OutputTxBuffer, g0OutputRxBuffer);

		spiUnselect(spi);
		spiReleaseBus(spi);

		return true;
	}

	bool m_state[G0_OUTPUTS] = {};
	uint8_t m_dirtyPins = 0;

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

THD_FUNCTION(G0Outputs::g0OutputThread, arg) {
	(void)arg;
	chRegSetThreadName(DRIVER_NAME " outputs");

	while (true) {
		(void)chSemWaitTimeout(&g0OutputWake, TIME_MS2I(G0_OUTPUT_POLL_INTERVAL_MS));

		for (auto& chip : chips) {
			chip.updateOutputs();
		}
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

#else /* HW_ATLAS && HAL_USE_SPI && (BOARD_G0_OUTPUT_COUNT > 0) */

int g0_outputs_add(brain_pin_e base, unsigned int index) {
	(void)base;
	(void)index;
	return -1;
}

#endif
