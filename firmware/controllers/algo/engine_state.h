/**
 * @file engine_state.h
 * @brief One header which acts as gateway to current engine state
 *
 * @date Dec 20, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "global.h"
#include "engine_parts.h"
#include "engine_state_generated.h"

class EngineState : public engine_state_s {
public:
	EngineState();
	void periodicFastCallback();
	void updateTChargeK(float rpm, float tps);

	/**
	 * always 360 or 720, never zero
	 */
	angle_t engineCycle;

	bool useOddFireWastedSpark = false;

	float injectionStage2Fraction = 0;

	Timer crankingTimer;

	WarningCodeState warnings;

	float auxValveStart = 0;
	float auxValveEnd = 0;

	/**
	 * MAP averaging angle start, in relation to 'mapAveragingSchedulingAtIndex' trigger index index
	 */
	angle_t mapAveragingStart[MAX_CYLINDER_COUNT];

	// Angle between firing the main (primary) spark and the secondary (trailing) spark
	angle_t trailingSparkAngle = 0;

	Timer timeSinceLastTChargeK;

	float currentVe = 0;

	/**
	 * Raw fuel injection duration produced by current fuel algorithm, without any correction
	 */
	floatms_t baseFuel = 0;

	/**
	 * TPS acceleration: extra fuel amount
	 */
	floatms_t tpsAccelEnrich = 0;

	/**
	 * Each individual fuel injection duration for current engine cycle, without wall wetting
	 * including everything including injector lag, both cranking and running
	 * @see getInjectionDuration()
	 */
	floatms_t injectionDuration = 0;
	floatms_t injectionDurationStage2 = 0;

	angle_t injectionOffset = 0;

	multispark_state multispark;

	bool shouldUpdateInjectionTiming = true;

	void updateSplitInjection();

	Timer splitInjectionTimer;
	bool requestSplitInjection = false;

	void updateMapCylinderOffsets();
	float mapCylinderBalance[MAX_CYLINDER_COUNT] = {0};
};

EngineState * getEngineState();
