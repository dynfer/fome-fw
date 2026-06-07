#include "pch.h"

#include "g0_io.h"

#if HW_ATLAS && HAL_USE_SPI

#include "adc_provider.h"
#include "eficonsole.h"
#include "hardware.h"
#include "mpu_util.h"
#include "spi_thread.h"
#include "../ext/g0_firmware/for_fome/g0_spi_protocol.h"

#include <atomic>

namespace protocol = g0_spi_protocol;

namespace {
static constexpr spi_device_e G0_SPI_DEVICE = SPI_DEVICE_5;
static constexpr brain_pin_e G0_SPI_CS_PIN = Gpio::F6;
static constexpr brain_pin_e G0_RESET_PIN = Gpio::B14;
static constexpr brain_pin_e G0_BOOT_PIN = Gpio::B15;

static constexpr size_t G0_ADC_FIRST_CHANNEL_INDEX = 20;
static constexpr int G0_SPI_THREAD_ACTIVE_PERIOD_MS = 1;
static constexpr int G0_SPI_THREAD_IDLE_PERIOD_MS = 60 * 60 * 1000;
static constexpr int G0_SPI_THREAD_POLL_PERIOD_MS = 20;
static constexpr int G0_SPI_THREAD_TIMEOUT_MS = 250;
static constexpr int G0_SPI_APP_RESPONSE_DELAY_US = 250;

static NO_CACHE uint8_t g0SpiTxBuffer[protocol::appFrameSize];
static NO_CACHE uint8_t g0SpiRxBuffer[protocol::appFrameSize];

static SPIConfig g0SpiConfig = {
		.circular = false,
		.end_cb = nullptr,
		.ssport = nullptr,
		.sspad = 0,
		.cfg1 = 7 | SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
		.cfg2 = 0};

enum class G0SpiRequest : uint8_t {
	None,
	ReadVersion,
	EnterUpdateMode,
	ExecuteCallback,
	SetInputMode,
	SetOutput,
	DisableOutput,
	ReadOutput,
	ReadSent,
};

static void setPin(brain_pin_e pin, bool value) {
	palWritePad(getHwPort("g0", pin), getHwPin("g0", pin), value);
}

static void driveG0RunModePins() {
	efiSetPadModeWithoutOwnershipAcquisition("G0 BOOT", G0_BOOT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	efiSetPadModeWithoutOwnershipAcquisition("G0 RESET", G0_RESET_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	setPin(G0_BOOT_PIN, false);
	setPin(G0_RESET_PIN, true);
}

static bool isValidDigitalInput(uint8_t input) {
	return input >= 1 && input <= protocol::digitalInputCount;
}

static bool isValidAnalogIndex(size_t idx) {
	return idx < protocol::analogChannelCount;
}

static bool isValidOutput(uint8_t output) {
	return output >= 1 && output <= protocol::outputCount;
}

static void exchangeFrame(SPIDriver& spi, const protocol::AppFrame& tx, protocol::AppFrame& rx) {
	memcpy(g0SpiTxBuffer, tx.bytes, sizeof(tx.bytes));
	memset(g0SpiRxBuffer, 0, sizeof(g0SpiRxBuffer));

	spiSelect(&spi);
	spiExchange(&spi, protocol::appFrameSize, g0SpiTxBuffer, g0SpiRxBuffer);
	spiUnselect(&spi);

	memcpy(rx.bytes, g0SpiRxBuffer, sizeof(rx.bytes));
}

static bool responseLooksValid(const protocol::AppFrame& response, uint8_t expectedCommand) {
	const auto& header = response.responseHeader;
	const bool knownStatus = (header.status == protocol::statusReady) || (header.status == protocol::statusUpdateMode);

	return knownStatus && header.command == expectedCommand;
}

static bool transact(SPIDriver& spi, const protocol::AppFrame& request, protocol::AppFrame& response) {
	protocol::AppFrame scratch = {};
	protocol::AppFrame nop = {};
	nop.commandRequest.command = protocol::cmdNop;

	exchangeFrame(spi, request, scratch);
	chThdSleepMicroseconds(G0_SPI_APP_RESPONSE_DELAY_US);
	exchangeFrame(spi, nop, response);

	return true;
}

class G0AdcProvider final : public AdcProvider {
public:
	const char* name() const override {
		return "G0";
	}

	bool enable(const char*, size_t idx) override {
		return isValidAnalogIndex(idx);
	}

	void disable(size_t) override {}

	float get(size_t idx) const override {
		if (!isValidAnalogIndex(idx) || !m_ready.load(std::memory_order_relaxed)) {
			return 0;
		}

		return m_millivolts[idx].load(std::memory_order_relaxed) / 1000.0f;
	}

	void setSample(size_t idx, uint16_t millivolts) {
		if (isValidAnalogIndex(idx)) {
			m_millivolts[idx].store(millivolts, std::memory_order_relaxed);
		}
	}

	void setReady(bool ready) {
		m_ready.store(ready, std::memory_order_relaxed);
	}

private:
	std::atomic<uint16_t> m_millivolts[protocol::analogChannelCount] = {};
	std::atomic<bool> m_ready{false};
};

static G0AdcProvider& adcProvider() {
	static G0AdcProvider instance;
	return instance;
}

class G0SpiIoDevice final : public BackgroundSpiDevice {
public:
	bool ensureReady() {
		if (m_spi) {
			driveG0RunModePins();
			return true;
		}

		driveG0RunModePins();
		turnOnSpi(G0_SPI_DEVICE);

		m_spi = getSpiDevice(G0_SPI_DEVICE);
		if (!m_spi) {
			return false;
		}

		initSpiCs(&g0SpiConfig, G0_SPI_CS_PIN);
		palSetPad(g0SpiConfig.ssport, g0SpiConfig.sspad);
		return true;
	}

	bool ensureRegistered() {
		if (!ensureReady()) {
			return false;
		}

		if (!m_registered) {
			m_registered = registerBackgroundSpiDevice(*this);
		}

		return m_registered;
	}

	void enablePolling() {
		driveG0RunModePins();
		m_pollingEnabled.store(true, std::memory_order_relaxed);
		updatePeriodForState();
	}

	void suspend() {
		m_suspended.store(true, std::memory_order_relaxed);
		updatePeriodForState();
	}

	void resume() {
		driveG0RunModePins();
		m_suspended.store(false, std::memory_order_relaxed);
		updatePeriodForState();
	}

	SPIDriver* spiDriver() const override {
		return m_spi;
	}

	const SPIConfig& config() override {
		return g0SpiConfig;
	}

	int getSpiThreadPeriodMs() const override {
		return m_periodMs.load(std::memory_order_relaxed);
	}

	bool shouldStartTransfer() const override {
		if (m_request.load(std::memory_order_acquire) != G0SpiRequest::None) {
			return true;
		}

		if (m_exclusiveMode.load(std::memory_order_acquire)) {
			return false;
		}

		return m_pollingEnabled.load(std::memory_order_relaxed) && !m_suspended.load(std::memory_order_relaxed);
	}

	void onTransferStarted() override {
		m_transferActive.store(true, std::memory_order_release);
	}

	void onTransferFinished() override {
		m_transferActive.store(false, std::memory_order_release);
	}

	void performTransfer(SPIDriver& driver) override {
		const auto request = m_request.exchange(G0SpiRequest::None, std::memory_order_acq_rel);

		if (request != G0SpiRequest::None) {
			const bool ok = handleRequest(driver, request);
			m_result = ok;
			updatePeriodForState();
			m_completion.signal();
			return;
		}

		if (m_suspended.load(std::memory_order_relaxed) || !m_pollingEnabled.load(std::memory_order_relaxed)) {
			updatePeriodForState();
			return;
		}

		poll(driver);
		updatePeriodForState();
	}

	bool readVersion(uint32_t& version) {
		if (!submitRequest(G0SpiRequest::ReadVersion)) {
			return false;
		}

		version = m_versionResult;
		return true;
	}

	bool requestEnterUpdateMode() {
		return submitRequest(G0SpiRequest::EnterUpdateMode);
	}

	bool executeCallback(G0SpiOperation callback) {
		if (!callback || !ensureReady()) {
			return false;
		}

		m_exclusiveMode.store(true, std::memory_order_release);
		updatePeriodForState();

		const bool idle = waitUntilIdle();
		if (!idle) {
			m_exclusiveMode.store(false, std::memory_order_release);
			updatePeriodForState();
			return false;
		}

		spiAcquireBus(m_spi);
		spiStart(m_spi, &config());
		const bool ok = callback(*m_spi);
		spiStop(m_spi);
		spiReleaseBus(m_spi);

		m_exclusiveMode.store(false, std::memory_order_release);
		updatePeriodForState();
		return ok;
	}

	bool setInputMode(uint8_t input, uint8_t mode) {
		if (!isValidDigitalInput(input)) {
			return false;
		}

		m_requestInput.store(input, std::memory_order_relaxed);
		m_requestMode.store(mode, std::memory_order_relaxed);
		return submitRequest(G0SpiRequest::SetInputMode);
	}

	bool setOutput(uint8_t output, uint32_t frequencyHz, uint16_t duty) {
		if (!isValidOutput(output)) {
			return false;
		}

		m_requestOutput.store(output, std::memory_order_relaxed);
		m_requestFrequency.store(frequencyHz, std::memory_order_relaxed);
		m_requestDuty.store(duty, std::memory_order_relaxed);
		return submitRequest(G0SpiRequest::SetOutput);
	}

	bool disableOutput(uint8_t output) {
		if (!isValidOutput(output)) {
			return false;
		}

		m_requestOutput.store(output, std::memory_order_relaxed);
		return submitRequest(G0SpiRequest::DisableOutput);
	}

	bool readSent(uint8_t input, G0SentState& state) {
		if (!isValidDigitalInput(input)) {
			return false;
		}

		m_requestInput.store(input, std::memory_order_relaxed);
		if (!submitRequest(G0SpiRequest::ReadSent)) {
			return false;
		}

		state = m_sentState;
		return true;
	}

	bool getDigitalState(uint8_t input, G0DigitalState& state) const {
		if (!isValidDigitalInput(input)) {
			return false;
		}

		const auto idx = input - 1;
		const uint8_t flags = m_digitalFlags[idx].load(std::memory_order_relaxed);
		state.mode = m_digitalModes[idx].load(std::memory_order_relaxed);
		state.level = (flags & 0x1) != 0;
		state.signalPresent = (flags & 0x2) != 0;
		state.frequencyHz = m_digitalFrequency[idx].load(std::memory_order_relaxed);
		state.dutyPermille = m_digitalDuty[idx].load(std::memory_order_relaxed);
		return true;
	}

	bool getOutputState(uint8_t output, G0OutputState& state) const {
		if (!isValidOutput(output)) {
			return false;
		}

		const auto idx = output - 1;
		state.frequencyHz = m_outputFrequency[idx].load(std::memory_order_relaxed);
		state.duty = m_outputDuty[idx].load(std::memory_order_relaxed);
		return true;
	}

	void printInfo() const {
		efiPrintf(
				"G0 SPI: polling=%d suspended=%d",
				m_pollingEnabled.load(std::memory_order_relaxed),
				m_suspended.load(std::memory_order_relaxed));

		for (size_t i = 0; i < protocol::analogChannelCount; i++) {
			efiPrintf("G0 analog %d: %d mV", static_cast<int>(i + 1), static_cast<int>(adcProvider().get(i) * 1000.0f));
		}

		for (uint8_t input = 1; input <= protocol::digitalInputCount; input++) {
			G0DigitalState state;
			if (getDigitalState(input, state)) {
				efiPrintf(
						"G0 digital %d: mode=%d level=%d signal=%d freq=%d duty=%d",
						static_cast<int>(input),
						static_cast<int>(state.mode),
						state.level,
						state.signalPresent,
						static_cast<int>(state.frequencyHz),
						static_cast<int>(state.dutyPermille));
			}
		}

		for (uint8_t output = 1; output <= protocol::outputCount; output++) {
			G0OutputState state;
			if (getOutputState(output, state)) {
				efiPrintf(
						"G0 output %d: freq=%d duty=%d",
						static_cast<int>(output),
						static_cast<int>(state.frequencyHz),
						static_cast<int>(state.duty));
			}
		}
	}

private:
	void updatePeriodForState() {
		if (m_request.load(std::memory_order_relaxed) != G0SpiRequest::None) {
			m_periodMs.store(G0_SPI_THREAD_ACTIVE_PERIOD_MS, std::memory_order_relaxed);
			return;
		}

		if (!m_pollingEnabled.load(std::memory_order_relaxed) ||
			m_suspended.load(std::memory_order_relaxed) ||
			m_exclusiveMode.load(std::memory_order_relaxed)) {
			m_periodMs.store(G0_SPI_THREAD_IDLE_PERIOD_MS, std::memory_order_relaxed);
			return;
		}

		m_periodMs.store(G0_SPI_THREAD_POLL_PERIOD_MS, std::memory_order_relaxed);
	}

	bool submitRequest(G0SpiRequest request) {
		if (m_exclusiveMode.load(std::memory_order_acquire)) {
			return false;
		}

		if (!ensureRegistered()) {
			return false;
		}

		m_completion.reset(true);
		m_result = false;
		m_request.store(request, std::memory_order_release);
		m_periodMs.store(G0_SPI_THREAD_ACTIVE_PERIOD_MS, std::memory_order_relaxed);

		const auto waitResult = m_completion.wait(TIME_MS2I(G0_SPI_THREAD_TIMEOUT_MS));
		if (waitResult != MSG_OK) {
			m_request.store(G0SpiRequest::None, std::memory_order_release);
			updatePeriodForState();
			return false;
		}

		return m_result;
	}

	bool waitUntilIdle() const {
		for (int i = 0; i < G0_SPI_THREAD_TIMEOUT_MS; i++) {
			const bool requestPending = m_request.load(std::memory_order_acquire) != G0SpiRequest::None;
			const bool transferActive = m_transferActive.load(std::memory_order_acquire);

			if (!requestPending && !transferActive) {
				return true;
			}

			chThdSleepMilliseconds(1);
		}

		return false;
	}

	bool handleRequest(SPIDriver& driver, G0SpiRequest request) {
		switch (request) {
			case G0SpiRequest::ReadVersion:
				return requestReadVersion(driver);
			case G0SpiRequest::EnterUpdateMode:
				return requestUpdateMode(driver);
			case G0SpiRequest::ExecuteCallback:
				return requestExecuteCallback(driver);
			case G0SpiRequest::SetInputMode:
				return requestSetInputMode(driver);
			case G0SpiRequest::SetOutput:
				return requestSetOutput(driver);
			case G0SpiRequest::DisableOutput:
				return requestDisableOutput(driver);
			case G0SpiRequest::ReadOutput:
				return requestReadOutput(driver, m_requestOutput.load(std::memory_order_relaxed));
			case G0SpiRequest::ReadSent:
				return requestReadSent(driver);
			case G0SpiRequest::None:
				return true;
		}

		return false;
	}

	void poll(SPIDriver& driver) {
		if (m_pollAnalogNext) {
			requestReadAnalog(driver);
		} else {
			requestReadDigitalAll(driver);
		}

		m_pollAnalogNext = !m_pollAnalogNext;
	}

	bool requestReadVersion(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.commandRequest.command = protocol::cmdReadVersion;

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdReadVersion)) {
			return false;
		}

		const auto& header = response.responseHeader;
		if (header.result != protocol::resultOk || header.payloadLength != protocol::versionPayloadLength) {
			return false;
		}

		m_versionResult = response.versionResponse.version;
		return true;
	}

	bool requestUpdateMode(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.commandRequest.command = protocol::cmdEnterUpdate;

		return transact(driver, request, response) && responseLooksValid(response, protocol::cmdEnterUpdate) &&
			   response.responseHeader.result == protocol::resultOk;
	}

	bool requestExecuteCallback(SPIDriver& driver) {
		const auto callback = m_callback;
		return callback ? callback(driver) : false;
	}

	bool requestReadAnalog(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.commandRequest.command = protocol::cmdReadAnalog;

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdReadAnalog)) {
			return false;
		}

		const auto& header = response.responseHeader;
		if (header.result != protocol::resultOk || header.payloadLength != protocol::analogPayloadLength) {
			return false;
		}

		const auto& analog = response.analogResponse;
		const auto count = minI(static_cast<int>(analog.channelCount), static_cast<int>(protocol::analogChannelCount));
		for (int i = 0; i < count; i++) {
			adcProvider().setSample(i, analog.millivolts[i]);
		}

		adcProvider().setReady(analog.ready != 0);
		return true;
	}

	bool requestReadDigitalAll(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.commandRequest.command = protocol::cmdReadDigitalAll;

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdReadDigitalAll)) {
			return false;
		}

		const auto& header = response.responseHeader;
		if (header.result != protocol::resultOk || header.payloadLength != protocol::digitalAllPayloadLength) {
			return false;
		}

		const auto& digital = response.digitalAllResponse;
		const auto count = minI(static_cast<int>(digital.inputCount), static_cast<int>(protocol::digitalInputCount));
		for (int i = 0; i < count; i++) {
			m_digitalModes[i].store(digital.inputs[i].mode, std::memory_order_relaxed);
			m_digitalFlags[i].store(digital.inputs[i].flags, std::memory_order_relaxed);
			m_digitalFrequency[i].store(digital.inputs[i].frequencyHz, std::memory_order_relaxed);
			m_digitalDuty[i].store(digital.inputs[i].dutyPermille, std::memory_order_relaxed);
		}

		return true;
	}

	bool requestSetInputMode(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.setInputModeRequest.command = protocol::cmdSetInputMode;
		request.setInputModeRequest.input = m_requestInput.load(std::memory_order_relaxed);
		request.setInputModeRequest.mode = m_requestMode.load(std::memory_order_relaxed);

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdSetInputMode)) {
			return false;
		}

		return response.responseHeader.result == protocol::resultOk;
	}

	bool requestSetOutput(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.setOutputRequest.command = protocol::cmdSetOutput;
		request.setOutputRequest.output = m_requestOutput.load(std::memory_order_relaxed);
		request.setOutputRequest.frequencyHz = m_requestFrequency.load(std::memory_order_relaxed);
		request.setOutputRequest.duty = m_requestDuty.load(std::memory_order_relaxed);

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdSetOutput)) {
			return false;
		}

		if (response.responseHeader.result != protocol::resultOk) {
			return false;
		}

		return requestReadOutput(driver, request.setOutputRequest.output);
	}

	bool requestDisableOutput(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.indexedRequest.command = protocol::cmdDisableOutput;
		request.indexedRequest.index = m_requestOutput.load(std::memory_order_relaxed);

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdDisableOutput)) {
			return false;
		}

		if (response.responseHeader.result != protocol::resultOk) {
			return false;
		}

		return requestReadOutput(driver, request.indexedRequest.index);
	}

	bool requestReadOutput(SPIDriver& driver, uint8_t output) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.indexedRequest.command = protocol::cmdReadOutput;
		request.indexedRequest.index = output;

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdReadOutput)) {
			return false;
		}

		const auto& header = response.responseHeader;
		if (header.result != protocol::resultOk || header.payloadLength != protocol::outputPayloadLength) {
			return false;
		}

		const auto outputIndex = output - 1;
		m_outputFrequency[outputIndex].store(response.outputResponse.frequencyHz, std::memory_order_relaxed);
		m_outputDuty[outputIndex].store(response.outputResponse.duty, std::memory_order_relaxed);
		return true;
	}

	bool requestReadSent(SPIDriver& driver) {
		protocol::AppFrame request = {};
		protocol::AppFrame response = {};
		request.indexedRequest.command = protocol::cmdReadSent;
		request.indexedRequest.index = m_requestInput.load(std::memory_order_relaxed);

		if (!transact(driver, request, response) || !responseLooksValid(response, protocol::cmdReadSent)) {
			return false;
		}

		const auto& header = response.responseHeader;
		if (header.result != protocol::resultOk || header.payloadLength != protocol::sentPayloadLength) {
			return false;
		}

		m_sentState = {
				.ready = (response.sentResponse.flags & 0x1) != 0,
				.valid = (response.sentResponse.flags & 0x2) != 0,
				.data = response.sentResponse.data,
				.status = response.sentResponse.status,
				.crc = response.sentResponse.crc,
				.nibbleCount = response.sentResponse.nibbleCount,
		};
		return true;
	}

	SPIDriver* m_spi = nullptr;
	bool m_registered = false;
	bool m_pollAnalogNext = true;
	bool m_result = false;
	uint32_t m_versionResult = 0;
	G0SpiOperation m_callback = nullptr;
	G0SentState m_sentState = {};
	chibios_rt::BinarySemaphore m_completion{true};
	std::atomic<int> m_periodMs{G0_SPI_THREAD_IDLE_PERIOD_MS};
	std::atomic<bool> m_pollingEnabled{false};
	std::atomic<bool> m_suspended{false};
	std::atomic<bool> m_exclusiveMode{false};
	std::atomic<bool> m_transferActive{false};
	std::atomic<G0SpiRequest> m_request{G0SpiRequest::None};
	std::atomic<uint8_t> m_requestInput{0};
	std::atomic<uint8_t> m_requestMode{0};
	std::atomic<uint8_t> m_requestOutput{0};
	std::atomic<uint32_t> m_requestFrequency{0};
	std::atomic<uint16_t> m_requestDuty{0};
	std::atomic<uint8_t> m_digitalModes[protocol::digitalInputCount] = {};
	std::atomic<uint8_t> m_digitalFlags[protocol::digitalInputCount] = {};
	std::atomic<uint16_t> m_digitalFrequency[protocol::digitalInputCount] = {};
	std::atomic<uint16_t> m_digitalDuty[protocol::digitalInputCount] = {};
	std::atomic<uint32_t> m_outputFrequency[protocol::outputCount] = {};
	std::atomic<uint16_t> m_outputDuty[protocol::outputCount] = {};
};

static G0SpiIoDevice g0SpiIoDevice;
static bool g0AdcProviderRegistered = false;
static bool g0ConsoleRegistered = false;

static void registerConsoleActions() {
	if (g0ConsoleRegistered) {
		return;
	}

	addConsoleAction("g0info", []() { g0SpiIoDevice.printInfo(); });
	g0ConsoleRegistered = true;
}
} // namespace

void initG0Io() {
	if (!g0SpiIoDevice.ensureRegistered()) {
		efiPrintf("G0 I/O ERROR: SPI5 is not available");
		return;
	}

	if (!g0AdcProviderRegistered) {
		registerAdcProvider(adcProvider(), G0_ADC_FIRST_CHANNEL_INDEX, protocol::analogChannelCount);
		g0AdcProviderRegistered = true;
	}

	registerConsoleActions();
	g0SpiIoDevice.enablePolling();
}

bool g0ReadVersion(uint32_t& version) {
	return g0SpiIoDevice.readVersion(version);
}

bool g0RequestEnterUpdateMode() {
	return g0SpiIoDevice.requestEnterUpdateMode();
}

bool g0ExecuteSpiOperation(G0SpiOperation operation) {
	return g0SpiIoDevice.executeCallback(operation);
}

void g0SuspendIo() {
	g0SpiIoDevice.suspend();
}

void g0ResumeIo() {
	g0SpiIoDevice.resume();
}

bool g0SetInputMode(uint8_t input, uint8_t mode) {
	return g0SpiIoDevice.setInputMode(input, mode);
}

bool g0SetOutput(uint8_t output, uint32_t frequencyHz, uint16_t duty) {
	return g0SpiIoDevice.setOutput(output, frequencyHz, duty);
}

bool g0DisableOutput(uint8_t output) {
	return g0SpiIoDevice.disableOutput(output);
}

bool g0GetDigitalState(uint8_t input, G0DigitalState& state) {
	return g0SpiIoDevice.getDigitalState(input, state);
}

bool g0GetOutputState(uint8_t output, G0OutputState& state) {
	return g0SpiIoDevice.getOutputState(output, state);
}

bool g0ReadSentState(uint8_t input, G0SentState& state) {
	return g0SpiIoDevice.readSent(input, state);
}

#else

void initG0Io() {}

bool g0ReadVersion(uint32_t&) {
	return false;
}

bool g0RequestEnterUpdateMode() {
	return false;
}

bool g0ExecuteSpiOperation(G0SpiOperation) {
	return false;
}

void g0SuspendIo() {}

void g0ResumeIo() {}

bool g0SetInputMode(uint8_t, uint8_t) {
	return false;
}

bool g0SetOutput(uint8_t, uint32_t, uint16_t) {
	return false;
}

bool g0DisableOutput(uint8_t) {
	return false;
}

bool g0GetDigitalState(uint8_t, G0DigitalState&) {
	return false;
}

bool g0GetOutputState(uint8_t, G0OutputState&) {
	return false;
}

bool g0ReadSentState(uint8_t, G0SentState&) {
	return false;
}

#endif
