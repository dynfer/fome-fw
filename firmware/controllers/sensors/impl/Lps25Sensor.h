#pragma once

#include "stored_value_sensor.h"
#include "lps25.h"

class Lps25Sensor : public StoredValueSensor {
public:
	explicit Lps25Sensor(Lps25& sensor);
	void update();

	void showInfo(const char* sensorName) const override;

private:
	Lps25* m_sensor;
};

#ifdef STM32H7XX
class Lps25TempSensor : public StoredValueSensor {
	public:
		explicit Lps25TempSensor(Lps25& sensor);
		void update();
	
		void showInfo(const char* sensorName) const override;
	
	private:
		Lps25* m_sensor;
};
#endif