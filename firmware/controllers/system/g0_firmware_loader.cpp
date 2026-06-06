#include "pch.h"

#if HW_ATLAS && HAL_USE_SPI

#include "g0_firmware_loader.h"

#include "g0_io.h"
#include "hardware.h"
#include "mpu_util.h"

#if __has_include("../../ext/g0_firmware/for_fome/g0_firmware_image.h")
#include "../../ext/g0_firmware/for_fome/g0_firmware_image.h"
#define G0_FIRMWARE_IMAGE_AVAILABLE TRUE
#else
#define G0_FIRMWARE_IMAGE_AVAILABLE FALSE
#endif

#if G0_FIRMWARE_IMAGE_AVAILABLE

static constexpr brain_pin_e G0_RESET_PIN = Gpio::B14;
static constexpr brain_pin_e G0_BOOT_PIN = Gpio::B15;
static constexpr uint32_t G0_FLASH_BASE = 0x08000000;

static constexpr uint8_t STM_BL_ACK = 0x79;
static constexpr uint8_t STM_BL_NACK = 0x1F;
static constexpr uint8_t STM_BL_SYNC = 0x5A;
static constexpr uint8_t STM_BL_EXTENDED_ERASE = 0x44;
static constexpr uint8_t STM_BL_WRITE_MEMORY = 0x31;
static constexpr uint8_t STM_BL_GO = 0x21;
static constexpr int STM_BL_INTER_BYTE_DELAY_US = 20;
static constexpr int G0_FLASH_MAX_ATTEMPTS = 2;
static constexpr size_t G0_SPI_DMA_BUFFER_SIZE = 258;

static NO_CACHE uint8_t g0SpiTxBuffer[G0_SPI_DMA_BUFFER_SIZE];
static NO_CACHE uint8_t g0SpiRxBuffer[G0_SPI_DMA_BUFFER_SIZE];

static void setPin(brain_pin_e pin, bool value) {
	palWritePad(getHwPort("g0", pin), getHwPin("g0", pin), value);
}

static void initControlPins() {
	efiSetPadModeWithoutOwnerShipAcquisition("G0 BOOT", G0_BOOT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	efiSetPadModeWithoutOwnerShipAcquisition("G0 RESET", G0_RESET_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	setPin(G0_BOOT_PIN, false);
	setPin(G0_RESET_PIN, true);
}

static void releaseControlPins() {
	setPin(G0_BOOT_PIN, false);
	setPin(G0_RESET_PIN, true);

	efiSetPadModeWithoutOwnerShipAcquisition("G0 BOOT", G0_BOOT_PIN, PAL_MODE_INPUT);
	efiSetPadModeWithoutOwnerShipAcquisition("G0 RESET", G0_RESET_PIN, PAL_MODE_INPUT);
}

static void resetG0(bool bootloaderMode) {
	setPin(G0_BOOT_PIN, bootloaderMode);
	chThdSleepMilliseconds(5);

	setPin(G0_RESET_PIN, false);
	chThdSleepMilliseconds(20);

	setPin(G0_RESET_PIN, true);
	chThdSleepMilliseconds(100);
}

static uint8_t spiByte(SPIDriver* spi, uint8_t tx) {
	g0SpiTxBuffer[0] = tx;
	g0SpiRxBuffer[0] = 0;
	spiExchange(spi, 1, g0SpiTxBuffer, g0SpiRxBuffer);
	return g0SpiRxBuffer[0];
}

static uint8_t bootloaderSpiByte(SPIDriver* spi, uint8_t tx) {
	const auto rx = spiByte(spi, tx);

	// AN4286 requires at least 15us between bytes for the STM32 ROM SPI bootloader.
	chThdSleepMicroseconds(STM_BL_INTER_BYTE_DELAY_US);

	return rx;
}

static bool handleBootloaderAck(SPIDriver* spi, uint8_t response) {
	if (response == STM_BL_ACK) {
		bootloaderSpiByte(spi, STM_BL_ACK);
		return true;
	}

	return false;
}

static bool waitAck(SPIDriver* spi, int attempts = 200) {
	while (attempts-- > 0) {
		const auto response = bootloaderSpiByte(spi, 0x00);

		if (handleBootloaderAck(spi, response)) {
			return true;
		}

		if (response == STM_BL_NACK) {
			return false;
		}

		chThdSleepMilliseconds(1);
	}

	return false;
}

static bool sendBytesAndWaitAck(SPIDriver* spi, const uint8_t* data, size_t size) {
	for (size_t i = 0; i < size; i++) {
		const auto response = bootloaderSpiByte(spi, data[i]);

		if (handleBootloaderAck(spi, response)) {
			return true;
		}

		if (response == STM_BL_NACK) {
			return false;
		}
	}

	return waitAck(spi);
}

static bool sendCommand(SPIDriver* spi, uint8_t command) {
	const uint8_t bytes[] = {STM_BL_SYNC, command, static_cast<uint8_t>(command ^ 0xFF)};
	return sendBytesAndWaitAck(spi, bytes, sizeof(bytes));
}

static bool sendAddress(SPIDriver* spi, uint32_t address) {
	uint8_t bytes[5] = {
			static_cast<uint8_t>(address >> 24),
			static_cast<uint8_t>(address >> 16),
			static_cast<uint8_t>(address >> 8),
			static_cast<uint8_t>(address),
			0};

	bytes[4] = bytes[0] ^ bytes[1] ^ bytes[2] ^ bytes[3];

	return sendBytesAndWaitAck(spi, bytes, sizeof(bytes));
}

static bool massErase(SPIDriver* spi) {
	if (!sendCommand(spi, STM_BL_EXTENDED_ERASE)) {
		return false;
	}

	const uint8_t globalMassErase[] = {0xFF, 0xFF, 0x00};
	return sendBytesAndWaitAck(spi, globalMassErase, sizeof(globalMassErase));
}

static bool writeBlock(SPIDriver* spi, uint32_t address, const uint8_t* data, size_t size) {
	if (size == 0 || size > 256) {
		return false;
	}

	if (!sendCommand(spi, STM_BL_WRITE_MEMORY) || !sendAddress(spi, address)) {
		return false;
	}

	uint8_t buffer[258];
	buffer[0] = static_cast<uint8_t>(size - 1);
	uint8_t checksum = buffer[0];

	for (size_t i = 0; i < size; i++) {
		buffer[1 + i] = data[i];
		checksum ^= data[i];
	}

	buffer[1 + size] = checksum;

	return sendBytesAndWaitAck(spi, buffer, size + 2);
}

static bool go(SPIDriver* spi, uint32_t address) {
	return sendCommand(spi, STM_BL_GO) && sendAddress(spi, address);
}

static bool injectG0Firmware(SPIDriver& spi) {
	const auto totalSize = sizeof(build_g0_extension_bin);

	if (totalSize == 0) {
		efiPrintf("G0 firmware load ERROR: firmware image header is missing or empty");
		return false;
	}

	efiPrintf("G0 firmware load: entering SPI bootloader");
	resetG0(true);

	spiSelect(&spi);

	bootloaderSpiByte(&spi, STM_BL_SYNC);
	if (!waitAck(&spi)) {
		spiUnselect(&spi);
		efiPrintf("G0 firmware load ERROR: no ACK from SPI bootloader");
		return false;
	}

	efiPrintf("G0 firmware load: erasing flash");
	if (!massErase(&spi)) {
		spiUnselect(&spi);
		efiPrintf("G0 firmware load ERROR: flash erase failed");
		return false;
	}

	efiPrintf("G0 firmware load: writing %d bytes", static_cast<int>(totalSize));
	for (size_t offset = 0; offset < totalSize; offset += 256) {
		uint8_t block[256];
		const auto blockSize = minI(256, totalSize - offset);
		memset(block, 0xFF, sizeof(block));
		memcpy(block, build_g0_extension_bin + offset, blockSize);

		if (!writeBlock(&spi, G0_FLASH_BASE + offset, block, sizeof(block))) {
			spiUnselect(&spi);
			efiPrintf("G0 firmware load ERROR: write failed at offset %d", static_cast<int>(offset));
			return false;
		}
	}

	efiPrintf("G0 firmware load: starting firmware");
	const bool ok = go(&spi, G0_FLASH_BASE);
	spiUnselect(&spi);

	resetG0(false);
	return ok;
}

static bool injectG0FirmwareWithRetry(SPIDriver& spi) {
	for (int attempt = 1; attempt <= G0_FLASH_MAX_ATTEMPTS; attempt++) {
		efiPrintf("G0 firmware load: flash attempt %d/%d", attempt, G0_FLASH_MAX_ATTEMPTS);

		if (injectG0Firmware(spi)) {
			return true;
		}

		resetG0(false);
	}

	efiPrintf("G0 firmware load ERROR: aborting G0 flash after %d failed attempts", G0_FLASH_MAX_ATTEMPTS);
	return false;
}

#endif // G0_FIRMWARE_IMAGE_AVAILABLE

bool loadG0Firmware(bool forceUpdate) {
#if G0_FIRMWARE_IMAGE_AVAILABLE
	initControlPins();
	g0SuspendIo();

	resetG0(false);

	uint32_t currentVersion = 0;
	bool ok = true;

	if (g0ReadVersion(currentVersion)) {
		efiPrintf(
				"G0 firmware load: app version %d, bundled version %d",
				static_cast<int>(currentVersion),
				static_cast<int>(build_g0_extension_version));

		if (!forceUpdate && currentVersion == build_g0_extension_version) {
			efiPrintf("G0 firmware load: firmware is already current");
		} else {
			efiPrintf("G0 firmware load: requesting app update mode%s", forceUpdate ? " (forced)" : "");

			if (!g0RequestEnterUpdateMode()) {
				efiPrintf("G0 firmware load: update-mode request timed out, forcing bootloader update");
			}

			ok = g0ExecuteSpiOperation(injectG0FirmwareWithRetry);
			releaseControlPins();
			g0ResumeIo();
			if (ok) {
				efiPrintf("G0 firmware load: complete");
			} else {
				efiPrintf("G0 firmware load: failed");
			}
			return ok;
		}
	} else {
		efiPrintf("G0 firmware load: no app version response, forcing update");
		ok = g0ExecuteSpiOperation(injectG0FirmwareWithRetry);
		releaseControlPins();
		g0ResumeIo();
		if (ok) {
			efiPrintf("G0 firmware load: complete");
		} else {
			efiPrintf("G0 firmware load: failed");
		}
		return ok;
	}

	releaseControlPins();
	g0ResumeIo();

	if (ok) {
		efiPrintf("G0 firmware load: complete");
	} else {
		efiPrintf("G0 firmware load: failed");
	}

	return ok;
#else
	efiPrintf("G0 firmware load ERROR: run `make -C firmware/ext/g0_firmware for_fome_image` first");
	return false;
#endif
}

#else

bool loadG0Firmware(bool) {
	efiPrintf("G0 firmware load is only available on Atlas with SPI enabled");
	return false;
}

#endif
