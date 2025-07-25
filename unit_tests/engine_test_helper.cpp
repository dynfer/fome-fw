/**
 * @file	engine_test_helper.cpp
 *
 * @date Jun 26, 2014
 * @author Andrey Belomutskiy, (c) 2012-2014
 */

#include "pch.h"

#include "trigger_decoder.h"
#include "speed_density.h"
#include "fuel_math.h"
#include "accel_enrichment.h"
#include "logicdata.h"
#include "hardware.h"

#if EFI_ENGINE_SNIFFER
#include "engine_sniffer.h"
extern WaveChart waveChart;
#endif /* EFI_ENGINE_SNIFFER */

extern WarningCodeState unitTestWarningCodeState;
extern engine_configuration_s activeConfiguration;
extern bool printTriggerDebug;
extern bool printTriggerTrace;
extern bool printFuelDebug;
extern int minCrankingRpm;

EngineTestHelperBase::EngineTestHelperBase(Engine * eng, engine_configuration_s * econfig, persistent_config_s * pers) {
	// todo: make this not a global variable, we need currentTimeProvider interface on engine
	setTimeNowUs(0);
	minCrankingRpm = 0;
	EnableToothLogger();
	if (engine || engineConfiguration || config) {
		firmwareError(ObdCode::OBD_PCM_Processor_Fault,
			      "Engine configuration not cleaned up by previous test");
	}
	engine = eng;
	engineConfiguration = econfig;
	config = pers;
}

EngineTestHelperBase::~EngineTestHelperBase() {
	engine = nullptr;
	engineConfiguration = nullptr;
	config = nullptr;
}

EngineTestHelper::EngineTestHelper(engine_type_e engineType)
	: EngineTestHelper(engineType, &emptyCallbackWithConfiguration) {
}

EngineTestHelper::EngineTestHelper(engine_type_e engineType, configuration_callback_t configurationCallback)
	: EngineTestHelper(engineType, configurationCallback, {}) {
}

EngineTestHelper::EngineTestHelper(engine_type_e engineType, const std::unordered_map<SensorType, float>& sensorValues)
	: EngineTestHelper(engineType, &emptyCallbackWithConfiguration, sensorValues) {
}

warningBuffer_t *EngineTestHelper::recentWarnings() {
	return &unitTestWarningCodeState.recentWarnings;
}

int EngineTestHelper::getWarningCounter() {
	return unitTestWarningCodeState.warningCounter;
}

EngineTestHelper::EngineTestHelper(engine_type_e engineType, configuration_callback_t configurationCallback, const std::unordered_map<SensorType, float>& sensorValues) :
	EngineTestHelperBase(&engine, &persistentConfig.engineConfiguration, &persistentConfig)
{
	efi::clear(persistentConfig);

	Sensor::setMockValue(SensorType::Clt, 70);
	Sensor::setMockValue(SensorType::Iat, 30);

	for (const auto& [s, v] : sensorValues) {
		Sensor::setMockValue(s, v);
	}

	unitTestWarningCodeState.clear();

	efi::clear(activeConfiguration);

	enginePins.reset();
	enginePins.unregisterPins();

	waveChart.init();

	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, -40, 1.5);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, -30, 1.5);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, -20, 1.42);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, -10, 1.36);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 0, 1.28);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 10, 1.19);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 20, 1.12);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 30, 1.10);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 40, 1.06);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 50, 1.06);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 60, 1.03);
	setCurveValue(config->cltFuelCorrBins, config->cltFuelCorr, CLT_CURVE_SIZE, 70, 1.01);

	initDataStructures();

	resetConfigurationExt(configurationCallback, engineType);
	engineConfiguration->isMapAveragingEnabled = false;

	validateConfig();

	enginePins.startPins();

	commonInitEngineController();

	// this is needed to have valid CLT and IAT.
//todo: reuse 	initPeriodicEvents() method
	engine.periodicSlowCallback();

	extern bool hasInitGtest;
	if (hasInitGtest) {
		// Setup running in mock airmass mode if running actual tests
		engineConfiguration->fuelAlgorithm = LM_MOCK;

		mockAirmass = std::make_unique<::testing::NiceMock<MockAirmass>>();
		engine.mockAirmassModel = mockAirmass.get();
	}

	memset(mockPinStates, 0, sizeof(mockPinStates));

	initHardware();
	rememberCurrentConfiguration();
}

EngineTestHelper::~EngineTestHelper() {
	// Write history to file
	extern bool hasInitGtest;
	if (hasInitGtest) {
    	std::stringstream filePath;
    	filePath << "unittest_" << ::testing::UnitTest::GetInstance()->current_test_info()->name() << ".logicdata";
	    writeEvents(filePath.str().c_str());
	}

	// Cleanup
	enginePins.reset();
	enginePins.unregisterPins();
	Sensor::resetRegistry();
	memset(mockPinStates, 0, sizeof(mockPinStates));
}

void EngineTestHelper::writeEvents(const char *fileName) {
	const auto& events = getCompositeEvents();
	if (events.size() < 2) {
		printf("Not enough data for %s\n", fileName);
		return;
	}
	printf("Writing %d records to %s\n", events.size(), fileName);
	writeFile(fileName, events);
}

/**
 * mock a change of time and fire single RISE front event
 * DEPRECATED many usages should be migrated to
 */
void EngineTestHelper::fireRise(float delayMs) {
	moveTimeForwardUs(MS2US(delayMs));
	firePrimaryTriggerRise();
}

void EngineTestHelper::smartFireRise(float delayMs) {
	moveTimeForwardAndInvokeEventsUs(MS2US(delayMs));
	firePrimaryTriggerRise();
}

void EngineTestHelper::fireFall(float delayMs) {
	moveTimeForwardUs(MS2US(delayMs));
	firePrimaryTriggerFall();
}

void EngineTestHelper::smartFireFall(float delayMs) {
	moveTimeForwardAndInvokeEventsUs(MS2US(delayMs));
	firePrimaryTriggerFall();
}

/**
 * fire single RISE front event
 */
void EngineTestHelper::firePrimaryTriggerRise() {
	efitick_t nowNt = getTimeNowNt();
	LogTriggerTooth(TriggerEvent::PrimaryRising, nowNt);
	handleShaftSignal(0, true, nowNt);
}

void EngineTestHelper::firePrimaryTriggerFall() {
	efitick_t nowNt = getTimeNowNt();
	LogTriggerTooth(TriggerEvent::PrimaryFalling, nowNt);
	handleShaftSignal(0, false, nowNt);
}

void EngineTestHelper::fireTriggerEventsWithDuration(float durationMs) {
	fireTriggerEvents2(/*count*/1, durationMs);
}

/**
 * Sends specified number of rise/fall trigger events, with specified amount of time between those.
 *
 * This is helpful for trigger_type_e::TT_ONE trigger wheel decoder and probably other decoders as well.
 */
void EngineTestHelper::fireTriggerEvents2(int count, float durationMs) {
	for (int i = 0; i < count; i++) {
		fireRise(durationMs);
		fireFall(durationMs);
	}
}

void EngineTestHelper::smartFireTriggerEvents2(int count, float durationMs) {
	for (int i = 0; i < count; i++) {
		smartFireRise(durationMs);
		smartFireFall(durationMs);
	}
}

void EngineTestHelper::clearQueue() {
	engine.scheduler.executeAll(99999999); // this is needed to clear 'isScheduled' flag
	ASSERT_EQ( 0,  engine.scheduler.size()) << "Failed to clearQueue";
}

int EngineTestHelper::executeActions() {
	return engine.scheduler.executeAll(getTimeNowUs());
}

void EngineTestHelper::moveTimeForwardMs(float deltaTimeMs) {
	moveTimeForwardUs(MS2US(deltaTimeMs));
}

void EngineTestHelper::moveTimeForwardSec(float deltaTimeSec) {
	moveTimeForwardUs(MS2US(1000 * deltaTimeSec));
}

void EngineTestHelper::moveTimeForwardUs(int deltaTimeUs) {
	if (printTriggerDebug || printFuelDebug) {
		printf("moveTimeForwardUs %.1fms\r\n", deltaTimeUs / 1000.0);
	}
	advanceTimeUs(deltaTimeUs);
}

void EngineTestHelper::moveTimeForwardAndInvokeEventsSec(int deltaTimeSeconds) {
	moveTimeForwardAndInvokeEventsUs(MS2US(1000 * deltaTimeSeconds));
}

/**
 * this method executes all pending events while moving time forward
 */
void EngineTestHelper::moveTimeForwardAndInvokeEventsUs(int deltaTimeUs) {
	if (printTriggerDebug || printFuelDebug) {
		printf("moveTimeForwardAndInvokeEventsUs %.1fms\r\n", deltaTimeUs / 1000.0);
	}
	setTimeAndInvokeEventsUs(getTimeNowUs() + deltaTimeUs);
}

void EngineTestHelper::setTimeAndInvokeEventsUs(int targetTime) {
	while (true) {
		scheduling_s* nextScheduledEvent = engine.scheduler.getHead();
		if (nextScheduledEvent == nullptr) {
			// nothing pending - we are done here
			break;
		}
		int nextEventTime = nextScheduledEvent->momentX;
		if (nextEventTime > targetTime) {
			// next event is too far in the future
			break;
		}
		setTimeNowUs(nextEventTime);
		engine.scheduler.executeAll(getTimeNowUs());
	}

	setTimeNowUs(targetTime);
}

void EngineTestHelper::fireTriggerEvents(int count) {
	fireTriggerEvents2(count, 5); // 5ms
}

void EngineTestHelper::assertInjectorUpEvent(const char *msg, int eventIndex, efitimeus_t momentX, int injectorIndex) {
	assertEvent(msg, eventIndex, (void*)startInjection, momentX, injectorIndex);
}

void EngineTestHelper::assertInjectorDownEvent(const char *msg, int eventIndex, efitimeus_t momentX, int injectorIndex) {
	assertEvent(msg, eventIndex, (void*)endInjection, momentX, injectorIndex);
}

scheduling_s * EngineTestHelper::assertEvent5(const char *msg, int index, void *callback, efitimeus_t expectedTimestamp) {
	TestExecutor *executor = &engine.scheduler;
	EXPECT_TRUE(executor->size() > index) << msg << " valid index";
	scheduling_s *event = executor->getForUnitTest(index);
	EXPECT_NEAR_M4((void*)event->action.getCallback() == (void*) callback, 1) << msg << " callback up/down";
	efitimeus_t start = getTimeNowUs();
	EXPECT_NEAR_M4(expectedTimestamp, event->momentX - start) << msg << " timestamp";
	return event;
}

const AngleBasedEvent * EngineTestHelper::assertTriggerEvent(const char *msg,
		int index, AngleBasedEvent *expected,
		void *callback,
		angle_t enginePhase) {
	auto event = engine.module<TriggerScheduler>()->getElementAtIndexForUnitTest(index);

	if (callback) {
		EXPECT_NEAR_M4((void*)event->action.getCallback() == (void*) callback, 1) << msg << " callback up/down";
	}

	EXPECT_NEAR_M4(enginePhase, event->eventPhase.angle) << msg << " angle";
	return event;
}

scheduling_s * EngineTestHelper::assertScheduling(const char *msg, int index, scheduling_s *expected, void *callback, efitimeus_t expectedTimestamp) {
	scheduling_s * actual = assertEvent5(msg, index, callback, expectedTimestamp);
	return actual;
}

void EngineTestHelper::assertEvent(const char *msg, int index, void *callback, efitimeus_t momentX, int injectorIndex) {
	scheduling_s *event = assertEvent5(msg, index, callback, momentX);

	InjectorContext ctx = bit_cast<InjectorContext>(event->action.getArgument());
	ASSERT_EQ(ctx.eventIndex, injectorIndex);
}

void EngineTestHelper::applyTriggerWaveform() {
	engine.updateTriggerWaveform();

	incrementGlobalConfigurationVersion();
}

// todo: open question if this is worth a helper method or should be inlined?
void EngineTestHelper::assertRpm(int expectedRpm, const char *msg) {
	EXPECT_EQ(expectedRpm, Sensor::getOrZero(SensorType::Rpm)) << msg;
}

void setupSimpleTestEngineWithMaf(EngineTestHelper *eth, injection_mode_e injectionMode,
		trigger_type_e trigger) {
	engineConfiguration->isIgnitionEnabled = false; // let's focus on injection
	engineConfiguration->cylindersCount = 4;
	// a bit of flexibility - the mode may be changed by some tests
	engineConfiguration->injectionMode = injectionMode;
	// set cranking mode (it's used by getCurrentInjectionMode())
	engineConfiguration->crankingInjectionMode = IM_SIMULTANEOUS;

	setArrayValues(config->cltFuelCorrBins, 1.0f);
	setArrayValues(engineConfiguration->injector.battLagCorr, 0.0f);
	// this is needed to update injectorLag
	engine->updateSlowSensors();

	ASSERT_EQ( 0,  engine->triggerCentral.isTriggerConfigChanged()) << "trigger #1";
	eth->setTriggerType(trigger);
}

void EngineTestHelper::setTriggerType(trigger_type_e trigger) {
	engineConfiguration->trigger.type = trigger;
	incrementGlobalConfigurationVersion();
	ASSERT_EQ( 1, engine.triggerCentral.isTriggerConfigChanged()) << "trigger #2";
	applyTriggerWaveform();
}

void setupSimpleTestEngineWithMafAndTT_ONE_trigger(EngineTestHelper *eth, injection_mode_e injectionMode) {
	setCamOperationMode();
	setupSimpleTestEngineWithMaf(eth, injectionMode, trigger_type_e::TT_ONE);
}

void setVerboseTrigger(bool isEnabled) {
	printTriggerDebug = isEnabled;
	printTriggerTrace = isEnabled;
}
