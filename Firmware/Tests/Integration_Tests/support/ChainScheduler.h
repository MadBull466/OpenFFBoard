/*
 * ChainScheduler.h
 *
 * Deterministic scheduler of the integration executable (ffb_integration_tests, plan section 3.3 step 4). The fake kernel
 * has no preemption and the fake timers raise nothing by themselves (audit C, G1), so this class plays the target's
 * interrupts and task switches, one simulated millisecond at a time, in ONE fixed order:
 *
 *   (a) USB: the "USB" thread (tud_task) until blocked, then the command threads (see (e)): the events the simulated
 *       host queued are processed. (UsbHost's pump does exactly this step, without time.)
 *   (b) time: HostPlatform::advanceMs(1) (RTOS tick hooks run, HAL_GetTick, micros()), then every timer whose period
 *       elapsed since the last step, derived from the timer registers the firmware wrote (see below), fires
 *       HostTarget::timerUpdate: TIM13 (htim13, FFB tick, notifies "FFBMAIN") and TIM6 (htim6, TMC external encoder,
 *       notifies "TMCENC") only while the firmware has started them (CEN and UIE set). Each TIM6 update is followed at once
 *       by an activation of "TMCENC" and the completion of the pending hspi1 interrupt transfer (audit B, G1): the
 *       target runs TMCENC (priority 33) before FFBMAIN (30) and the TMC thread (25), so a sub-millisecond period
 *       (TIM6: about 257 us) gives as many activations as updates, in order. Then both ADCs convert once (audit C, G2:
 *       the fake ADC converts only when told to; the inputs are persistent, so the firmware sees a fresh DMA buffer every ms).
 *   (c) "FFBMAIN" until blocked (one FFB cycle per activation that found a notification).
 *   (d) the "TMC" thread when its Delay has expired (the fake kernel wakes a delayed resumable task by tick count),
 *       counted by task handle (Stats::tmcActivations, tmcWakeUs): every other known task is suspended while it runs;
 *       then every other ready task, highest priority first (HostRtos::runUntilIdle(1) in a loop with adoptThreads() between
 *       the activations, so a task created by an activation is made resumable before it first runs; Stats::readyActivations).
 *       Then a pending hspi1 interrupt transfer is completed. (The TMC thread runs before the others of this step: the order
 *       among ready tasks of one step is not observable by the cases.)
 *   (e) the command threads "CDCCMD" and "CMD_MAIN" until blocked (CDC command interface and command thread).
 *   (f) plant hook: plantHook(nowUs), set by IntegrationFixture::attachPlant to WheelPlant::stepToUs (one plant step per
 *       scheduler step, with the simulated time elapsed since the previous one). The slot is shared by WheelPlant and
 *       PositionScript (IntegrationFixture::attachPlant / attachPositionScript): one client at a time.
 *   (g) observer hook: stepEndHook(), after (f) (WO I1-1: TorqueTrace samples here; it sees the chip registers of the end of the
 *       step, i.e. including what the plant or the position script wrote in (f)).
 *
 * Order option (WO I1-2, for F1-H6 only): setUsbAfterFfb(true) moves step (a) after step (c): the order of a step is then (b), (c), (d),
 * (e) with (a) executed between (c) and (d) (the USB thread and the command threads of (a) run once more in (e)). A report the host
 * put on the OUT endpoint with FfbHost::queue is then processed AFTER the FFB cycle of the step instead of before it. Default false.
 *
 * Host-vs-target: on the target these preempt by priority (USB 40 > TMCENC 33 > CMD 32 > FFBMAIN 30 > TMC 25) and a long
 * activation (e.g. the 500 ms ADC calibration of the TMC thread, which busy-waits on HAL_GetTick) is interrupted by the
 * timers; here an activation runs to its next blocking call, and the timer updates that elapsed meanwhile are all raised
 * afterwards at the next step (a task notification coalesces, so FFBMAIN runs once for them). The fixed order is the
 * order of a quiet target where each thread finishes within its tick.
 *
 * Timer period (RM0090 timers, audit C G1): f_timer = 84 MHz (APB1 timers, Targets/F407VG/Core/Src/main.c clock tree:
 * SystemCoreClock 168 MHz, APB1 prescaler 4, timer clock x2); period = (ARR + 1) * (PSC + 1) timer clock cycles. ARR
 * applies at once (ARPE = 0, MX_TIM13_Init / MX_TIM6_Init AutoReloadPreload disabled), a new PSC from the next update
 * (the value is latched at each update). The phase of a timer starts at 0 when the scheduler first sees it running.
 * A timer that is first seen running is credited with the whole elapsed time of that step as its first phase (it was started
 * somewhere inside the step): exact when frames take 0 us, up to a few us early at 1 us per SPI frame. TIM7 (the HAL tick and
 * micros()) needs no step: HostPlatform keeps HAL_GetTick and the TIM7 counter consistent with the simulated time (audit C).
 * A new ARR below the running count is not modelled (RM0090 does not say; the phase just wraps). The time base is the
 * elapsed simulated time (HostPlatform::nowUs) since the last step, not a counted 1 ms, so frames that take simulated
 * time (FakeTmc4671::setMicrosPerTransfer) move the timers too.
 *
 * Threads (resumable, adopted by name as soon as they exist, before the first activation): see kThreadNames. The restart
 * model of the fake kernel cannot express a Delay, and since WO I0-3 a Delay in a restart-model task is a kernel fault.
 */
#ifndef HOSTTEST_INTEGRATION_CHAINSCHEDULER_H_
#define HOSTTEST_INTEGRATION_CHAINSCHEDULER_H_

#include "HostPlatform.h"
#include "HostRtos.h"
#include "HostRtosExt.h"
#include "HostTarget.h"

#include <cstdint>
#include <functional>
#include <vector>

class ChainScheduler {
public:
	/** Counters since reset() */
	struct Stats {
		uint64_t steps = 0;			   //!< advanceMs steps (1 ms each)
		uint32_t ffbUpdates = 0;	   //!< TIM13 update events raised (HostTarget::timerUpdate returned true)
		uint32_t ffbCycles = 0;		   //!< activations of "FFBMAIN" that found a notification (one updateControl each)
		uint32_t tim6Updates = 0;	   //!< TIM6 update events raised
		uint32_t tmcencActivations = 0; //!< activations of "TMCENC" after a TIM6 update
		uint32_t tmcActivations = 0;   //!< step (d): activations of the "TMC" task handle only
		std::vector<uint64_t> tmcWakeUs; //!< simulated time (us) of every TMC activation, in order
		uint32_t readyActivations = 0; //!< step (d): activations of every other ready task (not the TMC thread)
		uint32_t spiCompletions = 0;   //!< hspi1 interrupt transfers completed by the scheduler
		uint32_t adcConversions = 0;   //!< ADC conversions raised (both ADCs, once per step)
	};

	/** Threads the scheduler makes resumable (every one that delays or waits in the middle of its loop), by name */
	static constexpr const char *kThreadNames[] = {"USB", "CDCCMD", "CMD_MAIN", "FFBMAIN", "TMC", "TMCENC", "HIDCMD", "CANCMD", "UARTCMD", "FXCalc", "errprint"};

	/** The timer clock of TIM6 / TIM13 in Hz (APB1 timers: 84 MHz) */
	static constexpr uint32_t kTimerClockHz = 84000000;

	/** Plant hook of step (f): called at the end of every step with the simulated time in microseconds. Empty when no plant is attached */
	std::function<void(uint64_t nowUs)> plantHook;

	/**
	 * USB feed hook of step (a) (WO I1-2, FfbHost::queue): called by the scheduler's own step (a) (advanceMs only, NOT by pumpUsb(): the host's NAK retries and transfers pump without feeding) before every run of the "USB" thread; returns true if it put
	 * one OUT packet on the bus. step (a) repeats (hook, USB thread, command threads) until the hook returns false, so a queue of reports
	 * is taken in order by ONE USB step (the OUT endpoint is re-armed by the USB thread after each packet, so a second packet can only be
	 * sent after the thread ran). Empty when nobody queues. Not cleared by reset()
	 */
	std::function<bool()> usbFeed;

	/** Observer hook of step (g): called at the very end of every step, after the plant hook. Empty when nobody observes. Not cleared by reset() */
	std::function<void()> stepEndHook;

	ChainScheduler() { reset(); }

	/** Clears the counters and the timer phases (the next step starts the phase of a running timer at 0) */
	void reset();

	/** F1-H6: true runs step (a) (USB) after step (c) (FFB) in each step, see the order option in the file comment. Default false. Cleared by reset() */
	void setUsbAfterFfb(bool after) { usbAfterFfb_ = after; }
	bool usbAfterFfb() const { return usbAfterFfb_; }

	/** Runs n steps of 1 ms in the order above */
	void advanceMs(uint32_t n);

	/** Step (a) without time: the "USB" thread and the command threads. This is what the simulated host's pump does */
	void pumpUsb();
	/** Step (a) of advanceMs: the feed hook (usbFeed) and pumpUsb() until the hook has nothing more */
	void usbStep();

	/** Makes every existing thread of kThreadNames resumable (idempotent). Called by every step and after thread creation */
	void adoptThreads();

	/** Period of a timer in timer clock cycles from its registers: (ARR + 1) * (PSC + 1) */
	static uint64_t periodCycles(const TIM_HandleTypeDef &h);

	const Stats &stats() const { return stats_; }

private:
	struct TimerPhase {
		bool armed = false;		//!< running (CEN and UIE) at the last step
		uint64_t cycles = 0;	//!< timer clock cycles since the last update (the counter, in clock cycles)
		uint32_t pscLatched = 0; //!< prescaler in effect for the current period
	};
	/** Raises the updates of one timer that elapsed in `elapsedUs`. @return number raised */
	uint32_t runTimer(TIM_HandleTypeDef &h, TimerPhase &phase, uint64_t elapsedUs, bool isTim6);
	void runCommandThreads();
	void completeHspi1();
	void runTmcThread();
	void activateTmcenc();

	Stats stats_;
	TimerPhase tim13_, tim6_;
	uint64_t lastUs_ = 0;
	bool usbAfterFfb_ = false;
};

#endif
