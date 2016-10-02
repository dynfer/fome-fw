/**
 * @file	signal_executor_sleep.cpp
 * @brief   Asynchronous output signal code
 *
 * Here we have the simplest, thread-based implementation of signal executor.
 * TODO: https://sourceforge.net/p/rusefi/tickets/6/
 *
 * @date Feb 10, 2013
 * @author Andrey Belomutskiy, (c) 2012-2016
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "main.h"
#include "scheduler.h"
#include "signal_executor.h"
#include "main_trigger_callback.h"

#if EFI_SIMULATOR || defined(__DOXYGEN__)
// this is about debugging
#include "efiGpio.h"
#endif /* EFI_SIMULATOR */

#if EFI_SIGNAL_EXECUTOR_SLEEP || defined(__DOXYGEN__)

void scheduleByTime(const bool monitorReuse, const char *prefix, scheduling_s *scheduling, efitimeus_t time, schfunc_t callback, void *param) {
	scheduleTask(monitorReuse, prefix, scheduling, time - getTimeNowUs(), callback, param);
}

static void timerCallback(scheduling_s *scheduling) {
#if EFI_PRINTF_FUEL_DETAILS || defined(__DOXYGEN__)
	if (scheduling->callback == (schfunc_t)&seTurnPinLow) {
		printf("executing cb=seTurnPinLow p=%d sch=%d now=%d\r\n", (int)scheduling->param, (int)scheduling,
				(int)getTimeNowUs());
	} else {
//		printf("exec cb=%d p=%d\r\n", (int)scheduling->callback, (int)scheduling->param);
	}

#endif /* EFI_SIMULATOR */
		scheduling->callback(scheduling->param);

}

void scheduleTask(const bool monitorReuse, const char *prefix, scheduling_s *scheduling, int delayUs, schfunc_t callback, void *param) {
	int delaySt = MY_US2ST(delayUs);
	if (delaySt <= 0) {
		/**
		 * in case of zero delay, we should invoke the callback
		 */
		callback(param);
		return;
	}

	lockAnyContext();
	scheduling->callback = callback;
	scheduling->param = param;
	int isArmed = chVTIsArmedI(&scheduling->timer);
	if (isArmed) {
#if EFI_SIMULATOR || defined(__DOXYGEN__)
		if (monitorReuse) {
//		for (int i = 0;i<100;i++)
			printf("%s: isArmed? why? sch=%d cb=%d p=%d\r\n", prefix, (int) scheduling, (int)callback, (int)param);
			firmwareError("armored\r\n");
		}
#endif /* EFI_SIMULATOR */
		chVTResetI(&scheduling->timer);
	}

#if EFI_SIMULATOR || defined(__DOXYGEN__)
	if (callback == (schfunc_t)&seTurnPinLow) {
		printf("setTime cb=seTurnPinLow p=%d\r\n", (int)param);
	} else {
//		printf("setTime cb=%d p=%d\r\n", (int)callback, (int)param);
	}
#endif /* EFI_SIMULATOR */

	chVTSetI(&scheduling->timer, delaySt, (vtfunc_t)timerCallback, scheduling);
	unlockAnyContext();
}

void initSignalExecutorImpl(void) {

}

#endif /* EFI_SIGNAL_EXECUTOR_SLEEP */
