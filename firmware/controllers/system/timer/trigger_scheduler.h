#pragma once

class TriggerScheduler : public EngineModule {
public:
	void schedule(AngleBasedEvent* event, EngPhase angle, action_s action);

	bool scheduleOrQueue(AngleBasedEvent *event,
							EngPhase angle,
							action_s action,
							const EnginePhaseInfo& phase);

	void onEnginePhase(float rpm, const EnginePhaseInfo& phase) override;

	// For unit tests
	AngleBasedEvent * getElementAtIndexForUnitTest(int index);

private:
	void schedule(AngleBasedEvent* event, action_s action);

	bool assertNotInList(AngleBasedEvent *head, AngleBasedEvent *element);

	/**
	 * That's the linked list of pending events scheduled in relation to trigger
	 * At the moment we iterate over the whole list while looking for events for specific
	 * trigger index We can make it an array of lists per trigger index, but that would take
	 * some RAM and probably not needed yet.
	 */
	AngleBasedEvent *m_angleBasedEventsHead = nullptr;
};
