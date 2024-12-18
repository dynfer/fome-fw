/*
 * @file knock_logic.c
 *
 * @date Apr 04, 2021
 * @author Andrey Gusakov
 */

#include "pch.h"
#include "knock_logic.h"

int getCylinderKnockBank(uint8_t cylinderNumber) {
	return engineConfiguration->knockBankCyl[cylinderNumber];
}

bool KnockControllerBase::onKnockSenseCompleted(uint8_t cylinderNumber, float dbv, efitick_t lastKnockTime) {
	bool isKnock = dbv > m_knockThreshold;

	// Per-cylinder peak detector
	float cylPeak = peakDetectors[cylinderNumber].detect(dbv, lastKnockTime);
	m_knockCyl[cylinderNumber] = roundf(cylPeak);

	// All-cylinders peak detector
	m_knockLevel = allCylinderPeakDetector.detect(dbv, lastKnockTime);

	if (isKnock) {
		m_knockCount++;

		auto baseTiming = engine->engineState.timingAdvance[cylinderNumber];

		// TODO: 20 configurable? Better explanation why 20?
		auto distToMinimum = baseTiming - (-20);

		// percent -> ratio = divide by 100
		auto retardFraction = engineConfiguration->knockRetardAggression * 0.01f;
		auto retardAmount = distToMinimum * retardFraction;

		{
			// Adjust knock retard under lock
			chibios_rt::CriticalSectionLocker csl;
			auto newRetard = m_knockRetard + retardAmount;
			m_knockRetard = clampF(0, newRetard, m_maximumRetard);
		}
	}

	return isKnock;
}

float KnockControllerBase::getKnockRetard() const {
	return m_knockRetard;
}

uint32_t KnockControllerBase::getKnockCount() const {
	return m_knockCount;
}

void KnockControllerBase::onFastCallback() {
	m_knockThreshold = getKnockThreshold();
	m_maximumRetard = getMaximumRetard();

	constexpr auto callbackPeriodSeconds = FAST_CALLBACK_PERIOD_MS / 1000.0f;

	auto applyAmount = engineConfiguration->knockRetardReapplyRate * callbackPeriodSeconds;

	{
		// Adjust knock retard under lock
		chibios_rt::CriticalSectionLocker csl;

		// Reduce knock retard at the requested rate
		float newRetard = m_knockRetard - applyAmount;

		// don't allow retard to go negative
		if (newRetard < 0) {
			m_knockRetard = 0;
		} else {
			m_knockRetard = newRetard;
		}
	}
}

float KnockController::getKnockThreshold() const {
	return interpolate2d(
		Sensor::getOrZero(SensorType::Rpm),
		config->knockNoiseRpmBins,
		config->knockBaseNoise
	);
}

float KnockController::getMaximumRetard() const {
	return
		interpolate3d(
			config->maxKnockRetardTable,
			config->maxKnockRetardLoadBins, getIgnitionLoad(),
			config->maxKnockRetardRpmBins, Sensor::getOrZero(SensorType::Rpm)
		);
}

// This callback is to be implemented by the knock sense driver
__attribute__((weak)) void onStartKnockSampling(uint8_t cylinderNumber, float samplingTimeSeconds, uint8_t channelIdx) {
	UNUSED(cylinderNumber);
	UNUSED(samplingTimeSeconds);
	UNUSED(channelIdx);
}

static uint8_t cylinderNumberCopy;

// Called when its time to start listening for knock
// Does some math, then hands off to the driver to start any sampling hardware
static void startKnockSampling(void*) {
	if (!engine->rpmCalculator.isRunning()) {
		return;
	}

	// Convert sampling angle to time
	float samplingSeconds = engine->rpmCalculator.oneDegreeUs * engineConfiguration->knockSamplingDuration / US_PER_SECOND_F;

	// Look up which channel this cylinder uses
	auto channel = getCylinderKnockBank(cylinderNumberCopy);

	// Call the driver to begin sampling
	onStartKnockSampling(cylinderNumberCopy, samplingSeconds, channel);
}

void Engine::onSparkFireKnockSense(uint8_t cylinderNumber, efitick_t nowNt) {
	cylinderNumberCopy = cylinderNumber;

#if EFI_SOFTWARE_KNOCK
	scheduleByAngle(nullptr, nowNt,
			/*angle*/engineConfiguration->knockDetectionWindowStart, startKnockSampling);
#else
	UNUSED(nowNt);
#endif
}
