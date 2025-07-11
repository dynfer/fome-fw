#include "pch.h"

#include "init.h"
#include "adc_subscription.h"
#include "functional_sensor.h"
#include "table_func.h"
#include "func_chain.h"

static FunctionalSensor maf (SensorType::Maf , /* timeout = */ MS2NT(50));
static FunctionalSensor maf2(SensorType::Maf2, /* timeout = */ MS2NT(50));

// This function converts volts -> kg/h
static TableFunc mafCurve(config->mafDecodingBins, config->mafDecoding);

// grumble grumble func_chain doesn't do constructor parameters so we need an adapter
struct MafTable : public SensorConverter {
	SensorResult convert(float input) const override {
		return mafCurve.convert(input);
	}
};

struct MafFilter final : public SensorConverter {
	SensorResult convert(float input) const override {
		float param = engineConfiguration->mafFilterParameter;
		if (param == 0) {
			return input;
		}

		float rpm = Sensor::getOrZero(SensorType::Rpm);

		float invTimeConstant = rpm / param;
		float alpha = (1e-3 * FAST_CALLBACK_PERIOD_MS) * invTimeConstant;

		// for alpha < 0.005 (stopped engine)
		// or alpha > 0.98 (engine very fast and/or small manifold)
		//     -> disable filtering entirely
		if (alpha < 0.005f || alpha > 0.98f) {
			m_lastValue = input;
			return input;
		}

		m_lastValue = alpha * input + (1 - alpha) * m_lastValue;

		return m_lastValue;
	}

	mutable float m_lastValue = 0;
};

static FuncChain<MafTable, MafFilter> mafFunction;

static void initMaf(adc_channel_e channel, FunctionalSensor& m) {
	if (!isAdcChannelValid(channel)) {
		return;
	}

	m.setFunction(mafFunction);

	AdcSubscription::SubscribeSensor(m, channel, /*lowpassCutoff =*/ 50);
	m.Register();
}

void initMaf() {
	initMaf(engineConfiguration->mafAdcChannel, maf);
	initMaf(engineConfiguration->maf2AdcChannel, maf2);
}
