/**
 * @file	event_registry.h
 *
 * @date Nov 27, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "global.h"
#include "efi_gpio.h"
#include "scheduler.h"
#include "fl_stack.h"
#include "trigger_structure.h"

struct AngleBasedEvent {
	scheduling_s scheduling;
	action_s action;
	/**
	 * Trigger-based scheduler maintains a linked list of all pending tooth-based events.
	 */
	AngleBasedEvent *nextToothEvent = nullptr;

	TrgPhase eventPhase;

	void setAngle(EngPhase angle);

	bool shouldSchedule(const EnginePhaseInfo& phase) const;
	float getAngleFromNow(const EnginePhaseInfo& phase) const;
};

#define MAX_OUTPUTS_FOR_IGNITION 2

class IgnitionEvent {
public:
	uint16_t calculateIgnitionOutputMask() const;

	angle_t calculateSparkAngle() const;

	scheduling_s dwellStartTimer;
	AngleBasedEvent sparkEvent;

	scheduling_s trailingSparkCharge;
	scheduling_s trailingSparkFire;

	// Track whether coil charge was intentionally skipped (spark limiter)
	bool wasSparkLimited = false;

	floatms_t sparkDwell = 0;

	// this timer allows us to measure actual dwell time
	Timer actualDwellTimer;

	float dwellAngle = 0;

	/**
	 * [0, cylindersCount)
	 */
	int cylinderIndex = 0;
	int8_t cylinderNumber = 0;
	char *name = nullptr;

	ignition_mode_e m_ignitionMode = IM_INDIVIDUAL_COILS;
};

class IgnitionEventList {
public:
	/**
	 * ignition events, per cylinder
	 */
	IgnitionEvent elements[MAX_CYLINDER_COUNT];
	bool isReady = false;
};

class AuxActor {
public:
	int phaseIndex;
	int valveIndex;
	angle_t extra;

	AngleBasedEvent open;
	AngleBasedEvent close;
};


IgnitionEventList *getIgnitionEvents();
