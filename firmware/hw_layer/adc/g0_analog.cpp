#include "pch.h"

#if HW_ATLAS && HAL_USE_ADC && HAL_USE_SPI

#include "g0_analog.h"

#include "hardware.h"
#include "mpu_util.h"

namespace {

static constexpr spi_device_e G0_SPI_DEVICE = SPI_DEVICE_5;

static constexpr uint8_t G0_APP_CMD_READ_ANALOG = 0x10;

static constexpr uint8_t G0_APP_STATUS_READY = 0x00;
static constexpr uint8_t G0_APP_STATUS_UPDATE_MODE = 0x01;
static constexpr uint8_t G0_APP_RESULT_OK = 0x00;
static constexpr size_t G0_APP_FRAME_SIZE = 36;
static constexpr size_t G0_APP_HEADER_SIZE = 4;
static constexpr size_t G0_APP_MAX_PAYLOAD_SIZE = G0_APP_FRAME_SIZE - G0_APP_HEADER_SIZE;

static constexpr size_t G0_ANALOG_INPUT_COUNT = 12;
static constexpr adc_channel_e G0_FIRST_ADC_CHANNEL = EFI_ADC_20;
static constexpr int G0_ANALOG_POLL_INTERVAL_MS = 20;
static constexpr size_t G0_ANALOG_PRINT_INTERVAL = 50;

static NO_CACHE uint8_t g0SpiTxBuffer[G0_APP_FRAME_SIZE];
static NO_CACHE uint8_t g0SpiRxBuffer[G0_APP_FRAME_SIZE];
static THD_WORKING_AREA(g0AnalogThreadStack, 512);

static volatile uint16_t g0AnalogMillivolts[G0_ANALOG_INPUT_COUNT];
static volatile bool g0AnalogReady = false;
static bool g0AnalogThreadStarted = false;

class PrintThrottle {
public:
	bool shouldPrint() {
		if (++m_counter < G0_ANALOG_PRINT_INTERVAL) {
			return false;
		}

		m_counter = 0;
		return true;
	}

private:
	size_t m_counter = 0;
};

struct G0AppResponse {
	const uint8_t* frame = nullptr;
	size_t offset = 0;
	size_t payloadLength = 0;

	const uint8_t* payload() const {
		return &frame[G0_APP_HEADER_SIZE];
	}

	bool isShifted() const {
		return offset != 0;
	}
};

static PrintThrottle g0AnalogValuePrintThrottle;
static PrintThrottle g0AnalogFailurePrintThrottle;
static PrintThrottle g0AnalogAlignmentPrintThrottle;

static uint16_t readLe16(const uint8_t* data) {
	return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

static bool isKnownStatus(uint8_t status) {
	return status == G0_APP_STATUS_READY || status == G0_APP_STATUS_UPDATE_MODE;
}

static void exchangeG0AppFrame(SPIDriver* spi, uint8_t command, uint8_t* rx) {
	memset(g0SpiTxBuffer, 0, sizeof(g0SpiTxBuffer));
	g0SpiTxBuffer[0] = command;
	memset(g0SpiRxBuffer, 0, sizeof(g0SpiRxBuffer));
	memset(rx, 0, G0_APP_FRAME_SIZE);

	spiAcquireBus(spi);
	spiSelect(spi);
	spiExchange(spi, G0_APP_FRAME_SIZE, g0SpiTxBuffer, g0SpiRxBuffer);
	spiUnselect(spi);
	spiReleaseBus(spi);

	memcpy(rx, g0SpiRxBuffer, G0_APP_FRAME_SIZE);
}

static bool isG0AppResponseAt(const uint8_t* frame, size_t availableSize, uint8_t expectedCommand) {
	if (availableSize < G0_APP_HEADER_SIZE) {
		return false;
	}

	const size_t payloadLength = frame[3];
	const size_t responseLength = G0_APP_HEADER_SIZE + payloadLength;

	return isKnownStatus(frame[0]) && frame[1] == G0_APP_RESULT_OK && frame[2] == expectedCommand &&
		   payloadLength <= G0_APP_MAX_PAYLOAD_SIZE && responseLength <= availableSize;
}

static bool findG0AppResponse(const uint8_t* rx, size_t rxSize, uint8_t expectedCommand, G0AppResponse& response) {
	for (size_t offset = 0; offset + G0_APP_HEADER_SIZE <= rxSize; offset++) {
		if (isG0AppResponseAt(&rx[offset], rxSize - offset, expectedCommand)) {
			response.frame = &rx[offset];
			response.offset = offset;
			response.payloadLength = rx[offset + 3];
			return true;
		}
	}

	return false;
}

static bool isG0AnalogChannel(adc_channel_e channel) {
	const int channelIndex = static_cast<int>(channel) - static_cast<int>(G0_FIRST_ADC_CHANNEL);
	return channelIndex >= 0 && channelIndex < static_cast<int>(G0_ANALOG_INPUT_COUNT);
}

static adcsample_t millivoltsToAdc(uint16_t millivolts) {
	const float volts = millivolts / 1000.0f;
	const float adc = voltsToAdc(volts);

	return static_cast<adcsample_t>(clampF(0, adc, ADC_MAX_VALUE));
}

static void debugPrintG0AnalogValues() {
	if (!g0AnalogValuePrintThrottle.shouldPrint()) {
		return;
	}

	efiPrintf(
		"G0 analog rx mV: %d %d %d %d %d %d %d %d %d %d %d %d",
		static_cast<int>(g0AnalogMillivolts[0]),
		static_cast<int>(g0AnalogMillivolts[1]),
		static_cast<int>(g0AnalogMillivolts[2]),
		static_cast<int>(g0AnalogMillivolts[3]),
		static_cast<int>(g0AnalogMillivolts[4]),
		static_cast<int>(g0AnalogMillivolts[5]),
		static_cast<int>(g0AnalogMillivolts[6]),
		static_cast<int>(g0AnalogMillivolts[7]),
		static_cast<int>(g0AnalogMillivolts[8]),
		static_cast<int>(g0AnalogMillivolts[9]),
		static_cast<int>(g0AnalogMillivolts[10]),
		static_cast<int>(g0AnalogMillivolts[11]));
}

static void debugPrintG0AnalogFailure(const char* reason, const uint8_t* rx = nullptr) {
	if (!g0AnalogFailurePrintThrottle.shouldPrint()) {
		return;
	}

	if (rx == nullptr) {
		efiPrintf("G0 analog rx failed: %s", reason);
		return;
	}

	efiPrintf(
		"G0 analog rx failed: %s raw=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		reason,
		rx[0],
		rx[1],
		rx[2],
		rx[3],
		rx[4],
		rx[5],
		rx[6],
		rx[7],
		rx[8],
		rx[9],
		rx[10],
		rx[11]);
}

static void debugPrintG0AnalogAlignment(size_t responseOffset) {
	if (!g0AnalogAlignmentPrintThrottle.shouldPrint()) {
		return;
	}

	efiPrintf("G0 analog rx aligned at offset %d", static_cast<int>(responseOffset));
}

static bool parseG0AnalogResponse(const uint8_t* rx, size_t rxSize) {
	G0AppResponse response;

	if (!findG0AppResponse(rx, rxSize, G0_APP_CMD_READ_ANALOG, response)) {
		debugPrintG0AnalogFailure("bad response", rx);
		return false;
	}

	if (response.isShifted()) {
		debugPrintG0AnalogAlignment(response.offset);
	}

	if (response.payloadLength < 2) {
		debugPrintG0AnalogFailure("short payload", rx);
		return false;
	}

	const uint8_t* payload = response.payload();
	const bool samplesReady = payload[0] != 0;
	const size_t reportedCount = payload[1];
	const size_t count =
			static_cast<size_t>(minI(static_cast<int>(G0_ANALOG_INPUT_COUNT), static_cast<int>(reportedCount)));

	if (!samplesReady || count == 0) {
		debugPrintG0AnalogFailure("samples not ready", rx);
		return false;
	}

	if (response.payloadLength < 2 + (count * sizeof(uint16_t))) {
		debugPrintG0AnalogFailure("short payload", rx);
		return false;
	}

	for (size_t i = 0; i < count; i++) {
		g0AnalogMillivolts[i] = readLe16(&payload[2 + (i * 2)]);
	}

	g0AnalogReady = true;
	debugPrintG0AnalogValues();
	return true;
}

static THD_FUNCTION(g0AnalogThread, arg) {
	(void)arg;
	chRegSetThreadName("G0 Analog");

	while (true) {
		updateG0AnalogInputs();
		chThdSleepMilliseconds(G0_ANALOG_POLL_INTERVAL_MS);
	}
}

} // namespace

bool updateG0AnalogInputs() {
	SPIDriver* spi = getSpiDevice(G0_SPI_DEVICE);
	uint8_t rx[G0_APP_FRAME_SIZE];

	if (spi->state != SPI_READY) {
		debugPrintG0AnalogFailure("SPI not ready");
		return false;
	}

	exchangeG0AppFrame(spi, G0_APP_CMD_READ_ANALOG, rx);
	return parseG0AnalogResponse(rx, sizeof(rx));
}

void startG0AnalogInputs() {
	if (g0AnalogThreadStarted) {
		return;
	}

	g0AnalogThreadStarted = true;
	chThdCreateStatic(g0AnalogThreadStack, sizeof(g0AnalogThreadStack), NORMALPRIO, g0AnalogThread, nullptr);
}

bool getG0AnalogInputAsAdc(adc_channel_e channel, adcsample_t& sample) {
	if (!isG0AnalogChannel(channel)) {
		return false;
	}

	if (!g0AnalogReady) {
		sample = 0;
		return true;
	}

	const size_t index = channel - G0_FIRST_ADC_CHANNEL;
	sample = millivoltsToAdc(g0AnalogMillivolts[index]);
	return true;
}

#endif // HW_ATLAS && HAL_USE_ADC && HAL_USE_SPI
