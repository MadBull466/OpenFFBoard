/*
 * ChainScheduler.cpp: see ChainScheduler.h for the model and the order of the steps.
 */
#include "ChainScheduler.h"

#include "main.h"

#include <vector>

constexpr const char *ChainScheduler::kThreadNames[];

void ChainScheduler::reset() {
	stats_ = Stats();
	tim13_ = TimerPhase();
	tim6_ = TimerPhase();
	lastUs_ = HostPlatform::nowUs();
}

void ChainScheduler::adoptThreads() {
	for (const char *name : kThreadNames) {
		TaskHandle_t task = HostRtos::findTask(name);
		if (task != nullptr && !HostRtos::isResumable(task)) {
			HostRtos::makeResumable(task);
		}
	}
}

uint64_t ChainScheduler::periodCycles(const TIM_HandleTypeDef &h) {
	return (static_cast<uint64_t>(h.Instance->ARR) + 1) * (static_cast<uint64_t>(h.Instance->PSC) + 1);
}

void ChainScheduler::runCommandThreads() {
	for (const char *name : {"CDCCMD", "CMD_MAIN"}) {
		TaskHandle_t task = HostRtos::findTask(name);
		if (task != nullptr) {
			if (!HostRtos::isResumable(task)) {
				HostRtos::makeResumable(task);
			}
			HostRtos::runUntilBlocked(task);
		}
	}
	adoptThreads(); // a command (axis.0.drvtype) may have created the "TMC" thread
}

void ChainScheduler::pumpUsb() {
	adoptThreads();
	TaskHandle_t usb = HostRtos::findTask("USB");
	if (usb != nullptr) {
		HostRtos::runUntilBlocked(usb);
	}
	runCommandThreads();
}

void ChainScheduler::runTmcThread() {
	TaskHandle_t tmc = HostRtos::findTask("TMC");
	if (tmc == nullptr) {
		return;
	}
	std::vector<TaskHandle_t> suspended;
	auto suspend = [&suspended, tmc](TaskHandle_t task) {
		if (task != nullptr && task != tmc && !HostRtos::isSuspended(task)) {
			vTaskSuspend(task);
			suspended.push_back(task);
		}
	};
	for (const char *name : kThreadNames) {
		suspend(HostRtos::findTask(name));
	}
	suspend(HostRtos::timerServiceTask());
	const HostRtos::ResumeStats saved = HostRtos::resumeStats();
	for (uint32_t guard = 0; guard < 1000; guard++) {
		if (HostRtos::runUntilIdle(1) == 0) {
			break;
		}
		stats_.tmcActivations++;
		stats_.tmcWakeUs.push_back(HostPlatform::nowUs());
	}
	for (TaskHandle_t task : suspended) {
		vTaskResume(task);
	}
	HostRtos::resumeStats() = saved;
}

void ChainScheduler::completeHspi1() {
	if (HostTarget::spiTransferPending(&hspi1) && HostTarget::spiComplete(&hspi1)) {
		stats_.spiCompletions++;
	}
}

void ChainScheduler::activateTmcenc() {
	TaskHandle_t task = HostRtos::findTask("TMCENC");
	if (task != nullptr) {
		if (!HostRtos::isResumable(task)) {
			HostRtos::makeResumable(task);
		}
		HostRtos::runUntilBlocked(task);
		stats_.tmcencActivations++;
	}
	completeHspi1();
}

uint32_t ChainScheduler::runTimer(TIM_HandleTypeDef &h, TimerPhase &phase, uint64_t elapsedUs, bool isTim6) {
	if (!HostTarget::timerRunning(&h) || !HostTarget::timerUpdateIrqEnabled(&h)) {
		phase.armed = false;
		return 0;
	}
	if (!phase.armed) {
		phase.armed = true;
		phase.cycles = 0;
		phase.pscLatched = h.Instance->PSC;
	}
	phase.cycles += elapsedUs * (kTimerClockHz / 1000000);
	uint32_t raised = 0;
	while (raised < 100000) {
		const uint64_t period = (static_cast<uint64_t>(h.Instance->ARR) + 1) * (static_cast<uint64_t>(phase.pscLatched) + 1);
		if (phase.cycles < period) {
			break;
		}
		phase.cycles -= period;
		phase.pscLatched = h.Instance->PSC; // PSC changes from the next update on
		if (!HostTarget::timerUpdate(&h)) {
			break;
		}
		raised++;
		if (isTim6) {
			stats_.tim6Updates++;
			activateTmcenc();
		}
	}
	return raised;
}

void ChainScheduler::advanceMs(uint32_t n) {
	for (uint32_t i = 0; i < n; i++) {
		stats_.steps++;
		// (a) USB
		pumpUsb();
		// (b) time, timers, ADC
		HostPlatform::advanceMs(1);
		const uint64_t now = HostPlatform::nowUs();
		const uint64_t elapsedUs = now > lastUs_ ? now - lastUs_ : 0;
		lastUs_ = now;
		stats_.ffbUpdates += runTimer(htim13, tim13_, elapsedUs, false);
		runTimer(htim6, tim6_, elapsedUs, true);
		for (ADC_HandleTypeDef *adc : {&hadc1, &hadc2}) {
			if (HostTarget::adcConversionComplete(adc)) {
				stats_.adcConversions++;
			}
		}
		// (c) FFBMAIN: one cycle per activation that finds a notification (ulTaskNotifyTake clears the count)
		TaskHandle_t ffb = HostRtos::findTask("FFBMAIN");
		if (ffb != nullptr) {
			if (!HostRtos::isResumable(ffb)) {
				HostRtos::makeResumable(ffb);
			}
			if (HostRtos::pendingNotifications(ffb) > 0) {
				stats_.ffbCycles++;
			}
			HostRtos::runUntilBlocked(ffb);
		}
		// (d) the "TMC" thread when its Delay expired, counted per task handle: every other known task is suspended
		//     (vTaskSuspend from the test thread) while runUntilIdle runs, so only the TMC thread can be activated; the suspend
		//     / resume calls are not counted in HostRtos::resumeStats. Then every other ready task, highest priority first, with
		//     adoptThreads() between the activations (a task created by an activation starts in the restart model otherwise).
		adoptThreads();
		runTmcThread();
		for (uint32_t guard = 0; guard < 1000; guard++) {
			adoptThreads();
			if (HostRtos::runUntilIdle(1) == 0) {
				break;
			}
			stats_.readyActivations++;
		}
		completeHspi1();
		// (e) command threads
		runCommandThreads();
		// (f) plant
		if (plantHook) {
			plantHook(HostPlatform::nowUs());
		}
		// (g) observers
		if (stepEndHook) {
			stepEndHook();
		}
	}
}
