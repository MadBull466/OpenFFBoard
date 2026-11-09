/*
 * host_freertos_ext.inc
 *
 * Fake FreeRTOS API F11 for ffb_rtos_tests (the cpp_freertos wrappers, tier A vendor code). Included at the end of
 * host_freertos.cpp when HOSTTEST_RTOS_EXT is defined (only ffb_rtos_tests), so it shares the fake kernel's internal
 * state (tasks, resumable tasks, semaphores, kernel faults). Not a separate translation unit: CMake does not list it.
 * The other executables never see this code; host_freertos.cpp leaves out the functions replaced here
 * (#ifndef HOSTTEST_RTOS_EXT). Test control: support/HostRtosExt.h (the model is described there).
 *
 * Every function models the FreeRTOS V10.3.1 function of the F407 target
 * (Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source, below "FreeRTOS/"), cited with file:line. Conditions on
 * which FreeRTOS stops in configASSERT are recorded as kernel faults ("configASSERT: ...") and the call returns its
 * failure value; calls the fake can not model are recorded as "unmodelled: ...".
 */
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "HostRtosExt.h"

#include <array>
#include <cstring>
#include <deque>
#include <list>

#if (configUSE_TICK_HOOK == 1)
extern "C" void vApplicationTickHook(void); // ctickhook.cpp (FreeRTOS/tasks.c:2840 calls it on every tick)
#endif

/** A queue (FreeRTOS/queue.c Queue_t): items are copied in and out by value */
struct HostQueue {
	UBaseType_t length = 0;
	UBaseType_t itemSize = 0;
	std::deque<std::vector<uint8_t>> items; //!< front = next item to receive
	uint64_t generation = 0;
	bool internal = false; //!< The command queue of the timer service (not counted by HostRtos::queueCount)
};

/** A software timer (FreeRTOS/timers.c Timer_t) */
struct HostTimer {
	const char *name = nullptr; //!< pcTimerName: the pointer passed to xTimerCreate (not copied, like FreeRTOS)
	TickType_t period = 0;
	bool autoReload = false;
	void *id = nullptr;
	TimerCallbackFunction_t callback = nullptr;
	bool active = false; //!< tmrSTATUS_IS_ACTIVE
	bool inList = false; //!< In the active timer list
	TickType_t expiry = 0;
	uint64_t generation = 0;
};

/** A task blocked in xEventGroupWaitBits / xEventGroupSync (the event list item of FreeRTOS/event_groups.c) */
struct EventGroupWaiter {
	HostTask *task = nullptr;
	EventBits_t waitFor = 0;
	bool clearOnExit = false;
	bool waitAll = false;
	bool unblocked = false; //!< Set by xEventGroupSetBits / vEventGroupDelete (eventUNBLOCKED_DUE_TO_BIT_SET)
	EventBits_t result = 0; //!< Event bits stored in the waiter when it was unblocked
};

/** An event group (FreeRTOS/event_groups.c EventGroup_t) */
struct HostEventGroup {
	EventBits_t bits = 0;
	bool isStatic = false;
	std::list<EventGroupWaiter *> waiters; //!< xTasksWaitingForBits, in the order the tasks blocked
	uint64_t generation = 0;
};

namespace {
namespace rtosext {

using HostRtos::RtosObject;

// event_groups.c:58, 60 (configUSE_16_BIT_TICKS == 0). The clear-on-exit / wait-for-all control bits are kept in
// EventGroupWaiter instead of the event list item value
constexpr EventBits_t eventUNBLOCKED_DUE_TO_BIT_SET = 0x02000000UL;
constexpr EventBits_t eventEVENT_BITS_CONTROL_BYTES = 0xff000000UL;
// FreeRTOSConfig.h:161 of the F407: configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5 << (8 - 4 priority bits)
constexpr uint32_t maxSyscallBasepri = 0x50;
constexpr uint32_t sysTickIpsr = 15; //!< Exception number of SysTick (IPSR while the tick hook runs)

/** Message of the timer service queue (timers.c DaemonTaskMessage_t) */
struct TimerMessage {
	BaseType_t id;
	TickType_t value;
	HostTimer *timer;
	PendedFunction_t function;
	void *parameter1;
	uint32_t parameter2;
};

// Registries as function-local statics: firmware objects with static storage (Thread::StartGuardLock) create kernel
// objects during the static initialisation, possibly before this translation unit is initialised. They are allocated
// and never destroyed: objects with static storage may use them while the program ends, and the leak sanitizer (which
// checks after the static destructors ran) must still see the blocks they reference (heap blocks of a MemoryPool).
std::map<const HostQueue *, uint64_t> &queues() {
	static auto *live = new std::map<const HostQueue *, uint64_t>();
	return *live;
}
std::map<const HostTimer *, uint64_t> &timers() { //!< Created, delete command not processed yet
	static auto *live = new std::map<const HostTimer *, uint64_t>();
	return *live;
}
std::vector<HostTimer *> &zombieTimers() { //!< Deleted by the timer service; freed by reset (detects later use)
	static auto *list = new std::vector<HostTimer *>();
	return *list;
}
std::map<const HostEventGroup *, uint64_t> &eventGroups() {
	static auto *live = new std::map<const HostEventGroup *, uint64_t>();
	return *live;
}
std::map<void *, size_t> &heapBlocks() { //!< pvPortMalloc blocks not freed yet (kept across resets)
	static auto *live = new std::map<void *, size_t>();
	return *live;
}
std::list<HostTimer *> &activeTimers() { //!< pxCurrentTimerList: sorted by expiry time, equal times in insertion order
	static auto *list = new std::list<HostTimer *>();
	return *list;
}

struct Injection {
	uint32_t calls = 0;
	uint32_t failFrom = 0;
	uint32_t failCount = 0;
};
std::array<Injection, (size_t)RtosObject::Count> injections{};
uint64_t objectGenerations = 0;
HostRtos::CriticalStats critical{};
uint32_t schedulerSuspended = 0;
uint32_t ticks = 0;
uint32_t heapFailed = 0;
HostQueue *timerQueue = nullptr;  //!< xTimerQueue
HostTask *timerService = nullptr; //!< xTimerTaskHandle
TickType_t lastSampleTime = 0;	  //!< prvSampleTimeNow's xLastTime
HostRtos::TimerServiceStats serviceStats{};

/** True if the next creation of `kind` is injected to fail. Counts the call */
bool createFails(RtosObject kind) {
	Injection &inj = injections[(size_t)kind];
	const uint32_t n = inj.calls++;
	return n >= inj.failFrom && n - inj.failFrom < inj.failCount;
}

void assertFault(const std::string &what) { kernelFault("configASSERT: " + what); }
void unmodelled(const std::string &what) { kernelFault("unmodelled: " + what); }

/** Priority of the code that runs now (an interrupt interrupts it): the current task, the test thread counts as 0 */
UBaseType_t runningPriority() { return currentTask != nullptr ? currentTask->priority : 0; }

/**
 * Non-FromISR API functions enter a critical section (taskENTER_CRITICAL). vPortEnterCritical asserts when the
 * outermost one is entered in an interrupt (FreeRTOS/portable/GCC/ARM_CM4F/port.c:413-416).
 */
void checkTaskApi(const char *api) {
	if (__get_IPSR() != 0U && critical.nesting == 0) {
		assertFault(std::string(api) + " called in interrupt context; only ...FromISR functions are allowed (port.c:415)");
	}
}

/**
 * configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) && ( xTicksToWait != 0 ) ) ) on entry of
 * xQueueGenericSend (queue.c:751), xQueueReceive (:1293), xQueueSemaphoreTake (:1438), xQueuePeek (:1653),
 * xEventGroupSync (event_groups.c:202) and xEventGroupWaitBits (:325): checked whether the call would block or not
 */
void checkSuspendedBlockTime(const char *api, TickType_t ticksToWait, const char *where) {
	if (schedulerSuspended > 0 && ticksToWait != 0) {
		assertFault(std::string(api) + " with a block time while the scheduler is suspended (" + where + ")");
	}
}

/** A call that really blocks inside a critical section: the context switch would be deferred (not modelled) */
void checkMayBlock(const char *api) {
	if (critical.nesting > 0 || critical.basepri != 0) {
		unmodelled(std::string(api) + " blocks inside a critical section / with interrupts masked (the context switch is deferred)");
	}
}

/** Marks a resumable task as waiting for `object` while it is parked (isWaiting, pxHigherPriorityTaskWoken) */
struct WaitMark {
	HostTask *task;
	WaitMark(HostTask *task, const void *object, std::function<bool()> wake) : task(task) {
		task->extWaitObject = object;
		task->extWake = std::move(wake);
	}
	~WaitMark() {
		task->extWaitObject = nullptr;
		task->extWake = nullptr;
	}
};

/**
 * Blocks the caller until `ready` holds or `ticksToWait` ticks have passed (see HostRtosExt.h for the three kinds of
 * caller). `ready` must only capture values (handles + generation), not references to the caller's locals: for a
 * task with the restart model it outlives the call. Returns true if `ready` holds at the end. A resumable task that is
 * deleted while it waits is unwound (parkCurrentTask throws), or returns false while an exception propagates.
 */
bool waitUntil(const char *api, const void *object, const std::function<bool()> &ready, TickType_t ticksToWait) {
	if (ready()) {
		return true;
	}
	if (ticksToWait == 0) {
		return false;
	}
	checkMayBlock(api);
	const bool forever = ticksToWait == portMAX_DELAY;
	if (HostTask *self = resumableCurrent()) {
		const TickType_t start = xTaskGetTickCount();
		auto expired = [forever, start, ticksToWait] { return !forever && (TickType_t)(xTaskGetTickCount() - start) >= ticksToWait; };
		auto wake = [&ready, &expired] { return ready() || expired(); };
		WaitMark mark(self, object, wake);
		while (true) {
			if (!parkCurrentTask(wake)) {
				return false;
			}
			if (ready()) {
				return true;
			}
			if (expired()) {
				return false;
			}
		}
	}
	if (currentTask != nullptr) {
		// Restart model: leave the task function; runUntilIdle starts it again when the condition holds
		currentTask->extWake = ready;
		currentTask->extWaitObject = object;
		throw HostRtos::TaskBlocked();
	}
	if (forever) {
		kernelFault(std::string("would block forever: ") + api + "(portMAX_DELAY) outside a task");
		return false;
	}
	for (TickType_t i = 0; i < ticksToWait; i++) {
		HostPlatform::advanceMs(1); // Nobody else runs while the test thread waits; tick hooks do
		if (ready()) {
			return true;
		}
	}
	return false;
}

/** pxHigherPriorityTaskWoken: set to pdTRUE (never to pdFALSE) if a task waiting for `object` can continue now and has a
 * higher priority than the interrupted code (xTaskRemoveFromEventList, tasks.c) */
void wakeCheck(const void *object, BaseType_t *higherPriorityTaskWoken) {
	if (higherPriorityTaskWoken == nullptr) {
		return;
	}
	const UBaseType_t running = runningPriority();
	for (HostTask *task : tasks()) {
		if (task->extWaitObject == object && task->priority > running && task->extWake && task->extWake()) {
			*higherPriorityTaskWoken = pdTRUE;
			return;
		}
	}
}

// ---------------- Queues ----------------
bool queueLive(const HostQueue *queue, uint64_t generation) {
	auto it = queues().find(queue);
	return it != queues().end() && it->second == generation;
}
bool checkQueue(const HostQueue *queue, const char *api) {
	if (queue == nullptr) {
		assertFault(std::string(api) + ": NULL queue handle (configASSERT( pxQueue ), queue.c)");
		return false;
	}
	if (queues().count(queue) == 0) {
		assertFault(std::string(api) + ": handle of a deleted queue (vQueueDelete freed it)");
		return false;
	}
	return true;
}
/** prvCopyDataToQueue (queue.c): back, front, or overwrite of the single item */
void copyToQueue(HostQueue *queue, const void *item, BaseType_t position) {
	std::vector<uint8_t> data(queue->itemSize);
	if (queue->itemSize > 0) {
		std::memcpy(data.data(), item, queue->itemSize);
	}
	if (position == queueOVERWRITE && !queue->items.empty()) {
		queue->items.front() = std::move(data);
	} else if (position == queueSEND_TO_FRONT) {
		queue->items.push_front(std::move(data));
	} else {
		queue->items.push_back(std::move(data));
	}
}
void copyFromQueue(HostQueue *queue, void *buffer, bool remove) {
	if (queue->itemSize > 0) {
		std::memcpy(buffer, queue->items.front().data(), queue->itemSize);
	}
	if (remove) {
		queue->items.pop_front();
	}
}
const char *sendName(BaseType_t position) {
	return position == queueSEND_TO_FRONT ? "xQueueSendToFront" : position == queueOVERWRITE ? "xQueueOverwrite" : "xQueueSendToBack";
}
/** queue.c:747-748 */
bool checkSendArguments(const HostQueue *queue, const void *item, BaseType_t position, const char *api) {
	if (item == nullptr && queue->itemSize != 0) {
		assertFault(std::string(api) + ": NULL item for a queue with item size > 0 (queue.c:747)");
		return false;
	}
	if (position == queueOVERWRITE && queue->length != 1) {
		assertFault(std::string(api) + ": overwrite of a queue whose length is not 1 (queue.c:748)");
		return false;
	}
	return true;
}
HostQueue *createQueue(UBaseType_t length, UBaseType_t itemSize, bool internal) {
	HostQueue *queue = new HostQueue();
	queue->length = length;
	queue->itemSize = itemSize;
	queue->generation = ++objectGenerations;
	queue->internal = internal;
	queues()[queue] = queue->generation;
	return queue;
}

// ---------------- Timer service ----------------
bool timerServiceReady() {
	return timerQueue != nullptr &&
		   (!timerQueue->items.empty() || (!activeTimers().empty() && activeTimers().front()->expiry <= xTaskGetTickCount()));
}

void timerServiceTaskFunction(void *);

/** prvCheckForValidListAndQueue (timers.c:941) and xTimerCreateTimerTask (timers.c:227): created at the first use */
void ensureTimerService() {
	if (timerQueue == nullptr) {
		timerQueue = createQueue(configTIMER_QUEUE_LENGTH, sizeof(TimerMessage), true);
	}
	if (timerService == nullptr) {
		HostTask *task = new HostTask();
		task->fn = timerServiceTaskFunction;
		task->name = "Tmr Svc"; // FreeRTOS.h:64 configTIMER_SERVICE_TASK_NAME
		task->priority = configTIMER_TASK_PRIORITY;
		tasks().push_back(task);
		timerService = task;
	}
}

/** prvSampleTimeNow (timers.c:670). The tick count overflow (timer list switch) is not modelled */
TickType_t sampleTimeNow() {
	const TickType_t now = xTaskGetTickCount();
	if (now < lastSampleTime) {
		unmodelled("tick count went backwards while the timer service runs (overflow / list switch, timers.c:676)");
	}
	lastSampleTime = now;
	return now;
}

/** prvInsertTimerInActiveList (timers.c:693). @return true if the timer expired before it could be inserted */
bool insertActive(HostTimer *timer, TickType_t nextExpiry, TickType_t now, TickType_t commandTime) {
	timer->expiry = nextExpiry;
	if (nextExpiry <= now) {
		if ((TickType_t)(now - commandTime) >= timer->period) {
			return true;
		}
		unmodelled("timer inserted into the overflow timer list (tick count overflow, timers.c:710)");
		return false;
	}
	if (now < commandTime && nextExpiry >= commandTime) {
		unmodelled("timer command issued before a tick count overflow (timers.c:715)");
		return true;
	}
	auto &list = activeTimers();
	auto it = std::find_if(list.begin(), list.end(), [nextExpiry](const HostTimer *t) { return t->expiry > nextExpiry; });
	list.insert(it, timer); // vListInsert: after the items with the same value
	timer->inList = true;
	return false;
}

void removeFromActive(HostTimer *timer) {
	if (timer->inList) {
		activeTimers().remove(timer);
		timer->inList = false;
	}
}

/** Runs a timer callback / pended function in the timer service task. They must not block (timers.h) */
template <class Function>
void runServiceCallback(Function &&function) {
	try {
		function();
	} catch (const HostRtos::TaskBlocked &) {
		// Restart model of the service task: the blocking call left the callback. Recorded; the service continues
		kernelFault("unmodelled: a timer callback / pended function blocked in the timer service task (not allowed, "
					"timers.h); the rest of it was not executed");
		if (timerService != nullptr) {
			timerService->extWake = nullptr;
			timerService->extWaitObject = nullptr;
		}
	}
}

/** Sends a command from the timer service to itself without waiting (tmrNO_DELAY). configASSERT( xResult ) */
void sendServiceCommand(HostTimer *timer, BaseType_t id, TickType_t value, const char *where) {
	if (timerQueue->items.size() >= timerQueue->length) {
		assertFault(std::string("timer service queue full when the service re-arms an auto-reload timer (") + where + ")");
		return;
	}
	const TimerMessage message{id, value, timer, nullptr, nullptr, 0};
	copyToQueue(timerQueue, &message, queueSEND_TO_BACK);
}

void callTimer(HostTimer *timer) {
	runServiceCallback([timer] { timer->callback(timer); });
	serviceStats.timerCallbacks++;
}

/** prvProcessExpiredTimer (timers.c:507) */
void processExpiredTimer(TickType_t nextExpireTime, TickType_t now) {
	HostTimer *timer = activeTimers().front();
	removeFromActive(timer);
	if (timer->autoReload) {
		if (insertActive(timer, nextExpireTime + timer->period, now, nextExpireTime)) {
			sendServiceCommand(timer, tmrCOMMAND_START_DONT_TRACE, nextExpireTime, "timers.c:528");
		}
	} else {
		timer->active = false;
	}
	callTimer(timer);
}

/** prvProcessReceivedCommands (timers.c:734), one message */
void processCommand(const TimerMessage &message) {
	if (message.id < 0) {
		if (message.function == nullptr) {
			unmodelled("pended function call with a NULL function (the timer service would call address 0)");
			return;
		}
		runServiceCallback([&message] { message.function(message.parameter1, message.parameter2); });
		serviceStats.pendedCalls++;
		return;
	}
	HostTimer *timer = message.timer;
	if (timers().count(timer) == 0) {
		assertFault("the timer service received a command for a timer it already deleted (use after vPortFree, timers.c:860)");
		return;
	}
	serviceStats.commands++;
	removeFromActive(timer);
	const TickType_t now = sampleTimeNow();
	switch (message.id) {
	case tmrCOMMAND_START:
	case tmrCOMMAND_START_FROM_ISR:
	case tmrCOMMAND_RESET:
	case tmrCOMMAND_RESET_FROM_ISR:
	case tmrCOMMAND_START_DONT_TRACE:
		timer->active = true;
		if (insertActive(timer, message.value + timer->period, now, message.value)) {
			callTimer(timer); // Expired before the command was processed (timers.c:806)
			if (timer->autoReload) {
				sendServiceCommand(timer, tmrCOMMAND_START_DONT_TRACE, message.value + timer->period, "timers.c:811");
			}
		}
		break;
	case tmrCOMMAND_STOP:
	case tmrCOMMAND_STOP_FROM_ISR:
		timer->active = false;
		break;
	case tmrCOMMAND_CHANGE_PERIOD:
	case tmrCOMMAND_CHANGE_PERIOD_FROM_ISR:
		timer->active = true;
		timer->period = message.value;
		if (timer->period == 0) {
			assertFault("xTimerChangePeriod with a period of 0 (timers.c:838)");
			timer->active = false;
			break;
		}
		(void)insertActive(timer, now + timer->period, now, now);
		break;
	case tmrCOMMAND_DELETE:
		// vPortFree (timers.c:847-858). Kept as a zombie until reset to detect a later use of the handle
		timer->active = false;
		timers().erase(timer);
		zombieTimers().push_back(timer);
		break;
	default:
		unmodelled("unknown timer command " + std::to_string(message.id));
		break;
	}
}

/** prvTimerTask (timers.c:548): one expired timer, then all queued commands, until nothing is left; then it blocks */
void timerServiceTaskFunction(void *) {
	for (;;) {
		bool worked = false;
		const TickType_t now = sampleTimeNow();
		if (!activeTimers().empty() && activeTimers().front()->expiry <= now) {
			processExpiredTimer(activeTimers().front()->expiry, now);
			worked = true;
		}
		while (timerQueue != nullptr && !timerQueue->items.empty()) {
			TimerMessage message;
			copyFromQueue(timerQueue, &message, true);
			processCommand(message);
			worked = true;
		}
		if (!worked) {
			(void)waitUntil("timer service task", timerQueue, [] { return timerServiceReady(); }, portMAX_DELAY);
		}
	}
}

bool timerKnown(const HostTimer *timer, const char *api) {
	if (timer == nullptr) {
		assertFault(std::string(api) + ": NULL timer handle (configASSERT( xTimer ), timers.c)");
		return false;
	}
	if (timers().count(timer) == 0) {
		const bool zombie = std::find(zombieTimers().begin(), zombieTimers().end(), timer) != zombieTimers().end();
		assertFault(std::string(api) + (zombie ? ": handle of a timer the timer service already deleted (use after vPortFree)"
											   : ": invalid timer handle"));
		return false;
	}
	return true;
}

// ---------------- Event groups ----------------
bool groupLive(const HostEventGroup *group, uint64_t generation) {
	auto it = eventGroups().find(group);
	return it != eventGroups().end() && it->second == generation;
}
bool checkGroup(const HostEventGroup *group, const char *api) {
	if (group == nullptr) {
		assertFault(std::string(api) + ": NULL event group handle (configASSERT( xEventGroup ), event_groups.c)");
		return false;
	}
	if (eventGroups().count(group) == 0) {
		assertFault(std::string(api) + ": handle of a deleted event group (vEventGroupDelete freed it)");
		return false;
	}
	return true;
}
/** prvTestWaitCondition (event_groups.c:671) */
bool testWaitCondition(EventBits_t current, EventBits_t waitFor, bool waitAll) {
	return waitAll ? (current & waitFor) == waitFor : (current & waitFor) != 0;
}
HostEventGroup *createGroup(bool isStatic) {
	HostEventGroup *group = new HostEventGroup();
	group->isStatic = isStatic;
	group->generation = ++objectGenerations;
	eventGroups()[group] = group->generation;
	return group;
}
/** xEventGroupSetBits (event_groups.c:519) without the checks */
EventBits_t setGroupBits(HostEventGroup *group, EventBits_t bitsToSet) {
	group->bits |= bitsToSet;
	EventBits_t bitsToClear = 0;
	for (auto it = group->waiters.begin(); it != group->waiters.end();) {
		EventGroupWaiter *waiter = *it;
		if (testWaitCondition(group->bits, waiter->waitFor, waiter->waitAll)) {
			if (waiter->clearOnExit) {
				bitsToClear |= waiter->waitFor;
			}
			waiter->result = group->bits | eventUNBLOCKED_DUE_TO_BIT_SET;
			waiter->unblocked = true;
			it = group->waiters.erase(it);
		} else {
			++it;
		}
	}
	group->bits &= ~bitsToClear;
	return group->bits;
}

/**
 * The blocking part of xEventGroupWaitBits / xEventGroupSync (vTaskPlaceOnUnorderedEventList, then the result of
 * uxTaskResetEventItemValue). @return true if the task was unblocked by bits (result in `result`), false on timeout
 */
bool blockOnGroup(HostEventGroup *group, EventBits_t waitFor, bool clearOnExit, bool waitAll, TickType_t ticksToWait,
				  const char *api, EventBits_t &result) {
	checkMayBlock(api);
	const uint64_t generation = group->generation;
	if (HostTask *self = resumableCurrent()) {
		EventGroupWaiter waiter;
		waiter.task = self;
		waiter.waitFor = waitFor;
		waiter.clearOnExit = clearOnExit;
		waiter.waitAll = waitAll;
		group->waiters.push_back(&waiter);
		struct Unlink { // Timeout or unwinding: the task leaves the event list
			HostEventGroup *group;
			uint64_t generation;
			EventGroupWaiter &waiter;
			~Unlink() {
				if (!waiter.unblocked && groupLive(group, generation)) {
					group->waiters.remove(&waiter);
				}
			}
		} unlink{group, generation, waiter};
		const bool forever = ticksToWait == portMAX_DELAY;
		const TickType_t start = xTaskGetTickCount();
		auto expired = [forever, start, ticksToWait] { return !forever && (TickType_t)(xTaskGetTickCount() - start) >= ticksToWait; };
		auto wake = [&waiter, &expired] { return waiter.unblocked || expired(); };
		WaitMark mark(self, group, wake);
		while (true) {
			if (!parkCurrentTask(wake)) {
				return false;
			}
			if (waiter.unblocked) {
				result = waiter.result;
				return true;
			}
			if (expired()) {
				return false;
			}
		}
	}
	if (currentTask != nullptr) {
		// Restart model: started again when the condition holds; the call then finds the bits set (they are cleared
		// on exit by that call, not by xEventGroupSetBits on behalf of the waiter as on the target)
		currentTask->extWake = [group, generation, waitFor, waitAll] {
			return groupLive(group, generation) && testWaitCondition(group->bits, waitFor, waitAll);
		};
		currentTask->extWaitObject = group;
		throw HostRtos::TaskBlocked();
	}
	if (ticksToWait == portMAX_DELAY) {
		kernelFault(std::string("would block forever: ") + api + "(portMAX_DELAY) outside a task");
		return false;
	}
	for (TickType_t i = 0; i < ticksToWait; i++) {
		HostPlatform::advanceMs(1);
		if (!groupLive(group, generation) || testWaitCondition(group->bits, waitFor, waitAll)) {
			break;
		}
	}
	return false; // Like a timeout: the caller reads (and clears) the bits itself
}

} // namespace rtosext
} // namespace

// ---------------- Test control ----------------
namespace {
/** Called by HostRtos::reset() */
void rtosExtReset() {
	using namespace rtosext;
	if (timerService != nullptr) {
		HostTask *service = timerService;
		timerService = nullptr;
		vTaskDelete(service);
	}
	if (timerQueue != nullptr) {
		// Pending delete commands: their Timer objects are gone; free the timers like the service would
		for (const auto &item : timerQueue->items) {
			TimerMessage message;
			std::memcpy(&message, item.data(), sizeof(message));
			if (message.id == tmrCOMMAND_DELETE && timers().count(message.timer) != 0) {
				timers().erase(message.timer);
				zombieTimers().push_back(message.timer);
			}
		}
		queues().erase(timerQueue);
		delete timerQueue;
		timerQueue = nullptr;
	}
	for (HostTimer *timer : activeTimers()) {
		timer->inList = false;
		timer->active = false;
	}
	activeTimers().clear();
	for (HostTimer *timer : zombieTimers()) {
		delete timer;
	}
	zombieTimers().clear();
	for (HostTask *task : tasks()) {
		if (!task->resumable) {
			task->extWake = nullptr;
			task->extWaitObject = nullptr;
		}
	}
	injections = {};
	critical = HostRtos::CriticalStats();
	schedulerSuspended = 0;
	ticks = 0;
	heapFailed = 0;
	lastSampleTime = 0;
	serviceStats = HostRtos::TimerServiceStats();
}
} // namespace

/** One SysTick (FreeRTOS/tasks.c:2707 xTaskIncrementTick): the tick hook in interrupt context. host_platform.cpp */
void hosttest_rtosTick() {
	using namespace rtosext;
	if (critical.nesting > 0 || critical.basepri != 0) {
		unmodelled("SysTick while interrupts are masked (critical section / taskDISABLE_INTERRUPTS): on the target the tick "
				   "is pended until they are unmasked, and further ticks are lost");
	}
	ticks++;
#if (configUSE_TICK_HOOK == 1)
	// xPortSysTickHandler (port.c:494-503) masks interrupts (portDISABLE_INTERRUPTS: BASEPRI = configMAX_SYSCALL_
	// INTERRUPT_PRIORITY) around xTaskIncrementTick, which calls the tick hook
	const uint32_t savedIpsr = hosttest_ipsr;
	const uint32_t savedBasepri = critical.basepri;
	hosttest_ipsr = sysTickIpsr;
	critical.basepri = maxSyscallBasepri;
	vApplicationTickHook(); // tasks.c:2840
	critical.basepri = savedBasepri;
	hosttest_ipsr = savedIpsr;
#endif
}

namespace HostRtos {

void failCreate(RtosObject kind, uint32_t after, uint32_t count) {
	rtosext::Injection &inj = rtosext::injections[(size_t)kind];
	inj.failFrom = inj.calls + after;
	inj.failCount = count;
}
uint32_t createCalls(RtosObject kind) { return rtosext::injections[(size_t)kind].calls; }

std::vector<std::string> takeKernelFaults() {
	std::vector<std::string> faults;
	faults.swap(kernelFaultList);
	return faults;
}
bool takeKernelFault(const std::string &part) {
	auto it = std::find_if(kernelFaultList.begin(), kernelFaultList.end(), [&part](const std::string &f) { return f.find(part) != std::string::npos; });
	if (it == kernelFaultList.end()) {
		return false;
	}
	kernelFaultList.erase(it);
	return true;
}

const CriticalStats &criticalStats() { return rtosext::critical; }
uint32_t schedulerSuspendNesting() { return rtosext::schedulerSuspended; }
uint32_t tickCount() { return rtosext::ticks; }

TaskHandle_t timerServiceTask() { return rtosext::timerService; }
size_t timerCommandsPending() { return rtosext::timerQueue != nullptr ? rtosext::timerQueue->items.size() : 0; }
size_t activeTimerCount() { return rtosext::activeTimers().size(); }
const TimerServiceStats &timerServiceStats() { return rtosext::serviceStats; }

size_t queueCount() {
	size_t n = 0;
	for (const auto &entry : rtosext::queues()) {
		n += entry.first->internal ? 0 : 1;
	}
	return n;
}
size_t timerCount() { return rtosext::timers().size(); }

std::vector<void *> timerIdsInService() {
	using namespace rtosext;
	std::vector<void *> ids;
	auto add = [&ids](const HostTimer *timer) {
		if (std::find(ids.begin(), ids.end(), timer->id) == ids.end()) {
			ids.push_back(timer->id);
		}
	};
	for (const auto &entry : timers()) {
		if (entry.first->active || entry.first->inList) {
			add(entry.first);
		}
	}
	if (timerQueue != nullptr) {
		for (const auto &item : timerQueue->items) {
			TimerMessage message;
			std::memcpy(&message, item.data(), sizeof(message));
			switch (message.id) {
			case tmrCOMMAND_START_DONT_TRACE:
			case tmrCOMMAND_START:
			case tmrCOMMAND_RESET:
			case tmrCOMMAND_CHANGE_PERIOD:
			case tmrCOMMAND_START_FROM_ISR:
			case tmrCOMMAND_RESET_FROM_ISR:
			case tmrCOMMAND_CHANGE_PERIOD_FROM_ISR:
				add(message.timer); // The HostTimer is still allocated (live, or a zombie freed only by reset)
				break;
			default:
				break;
			}
		}
	}
	return ids;
}

ObjectCounts objectCounts() {
	using namespace rtosext;
	ObjectCounts counts;
	counts.tasks = tasks().size() - (timerService != nullptr && isRegistered(timerService) ? 1 : 0);
	counts.queues = queueCount();
	counts.semaphores = liveSemaphores;
	size_t pendingDeletes = 0;
	if (timerQueue != nullptr) {
		for (const auto &item : timerQueue->items) {
			TimerMessage message;
			std::memcpy(&message, item.data(), sizeof(message));
			pendingDeletes += message.id == tmrCOMMAND_DELETE && timers().count(message.timer) != 0 ? 1 : 0;
		}
	}
	counts.timers = timers().size() - pendingDeletes;
	counts.eventGroups = eventGroups().size();
	return counts;
}
size_t eventGroupCount() { return rtosext::eventGroups().size(); }
size_t heapLiveBlocks() { return rtosext::heapBlocks().size(); }
uint32_t heapFailedCalls() { return rtosext::heapFailed; }

bool isWaiting(TaskHandle_t task) {
	return isRegistered(task) && task->extWake != nullptr; // Restart model: set when it blocked; resumable: WaitMark
}

} // namespace HostRtos

// ---------------- Kernel API ----------------
using namespace rtosext;

extern "C" {

// ---- Heap (FreeRTOS/portable/MemMang/heap_4.c) ----
void *pvPortMalloc(size_t size) {
	heapCalls.mallocCalls++;
	heapCalls.lastMallocSize = size;
	void *p = nullptr;
	// heap_4.c:141: a request of 0 bytes returns NULL; an injected failure models an exhausted heap
	if (!createFails(RtosObject::Heap) && size > 0) {
		p = malloc(size);
	}
	if (p == nullptr) {
		heapFailed++; // heap_4.c:251 calls vApplicationMallocFailedHook (F407 freertos.c:104), not modelled
	} else {
		heapBlocks()[p] = size;
	}
	heapCalls.lastMallocResult = p;
	return p;
}
void vPortFree(void *ptr) {
	heapCalls.freeCalls++;
	heapCalls.lastFreed = ptr;
	if (ptr == nullptr) {
		return; // heap_4.c:271
	}
	auto it = heapBlocks().find(ptr);
	if (it == heapBlocks().end()) {
		assertFault("vPortFree of a block that pvPortMalloc did not return or that was freed already (heap_4.c:281)");
		return;
	}
	heapBlocks().erase(it);
	free(ptr);
}

// ---- Tasks ----
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t, void *param, UBaseType_t prio, TaskHandle_t *handle) {
	if (createFails(RtosObject::Task)) {
		return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY; // tasks.c:815; *handle is not written
	}
	HostTask *task = new HostTask();
	task->fn = fn;
	task->param = param;
	task->name = name ? name : "";
	task->priority = prio;
	tasks().push_back(task);
	if (handle) {
		*handle = task;
	}
	return pdPASS;
}

void vTaskSuspendAll(void) { schedulerSuspended++; } // tasks.c:2110
BaseType_t xTaskResumeAll(void) {						 // tasks.c:2194
	if (schedulerSuspended == 0) {
		assertFault("xTaskResumeAll without vTaskSuspendAll (tasks.c:2201)");
		return pdFALSE;
	}
	checkTaskApi("xTaskResumeAll");
	schedulerSuspended--;
	return pdFALSE; // No context switch in the fake
}
BaseType_t xTaskGetSchedulerState(void) { return schedulerSuspended > 0 ? taskSCHEDULER_SUSPENDED : taskSCHEDULER_RUNNING; }

// ---- Task notifications (tasks.c:4635 ulTaskNotifyTake, 4783 xTaskGenericNotify, 5026 vTaskNotifyGiveFromISR) ----
BaseType_t xTaskNotifyGive(TaskHandle_t task) {
	checkTaskApi("xTaskNotifyGive");
	if (task == nullptr) {
		assertFault("xTaskNotifyGive(NULL) (tasks.c:4789)");
		return pdFAIL;
	}
	if (isRegistered(task)) {
		task->notifications++;
	}
	return pdPASS;
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *higherPriorityTaskWoken) {
	if (task == nullptr) {
		assertFault("vTaskNotifyGiveFromISR(NULL) (tasks.c:5032)");
		return;
	}
	if (!isRegistered(task)) {
		return;
	}
	task->notifications++;
	wakeCheck(task, higherPriorityTaskWoken);
}
uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t ticksToWait) {
	checkTaskApi("ulTaskNotifyTake");
	HostTask *self = currentTask;
	if (self == nullptr) {
		// The test thread has no notification value: it can only wait
		(void)waitUntil("ulTaskNotifyTake", nullptr, [] { return false; }, ticksToWait);
		return 0;
	}
	if (!waitUntil("ulTaskNotifyTake", self, [self] { return self->notifications > 0; }, ticksToWait)) {
		return 0;
	}
	const uint32_t count = self->notifications;
	self->notifications = clearOnExit ? 0 : count - 1;
	return count;
}

// ---- Critical sections (port.c:403-430, portmacro.h:99-104) ----
void vPortEnterCritical(void) {
	if (critical.nesting == 0 && __get_IPSR() != 0U) {
		assertFault("taskENTER_CRITICAL in interrupt context; use taskENTER_CRITICAL_FROM_ISR (port.c:415)");
	}
	critical.basepri = maxSyscallBasepri;
	critical.nesting++;
	critical.enter++;
	critical.maxNesting = std::max(critical.maxNesting, critical.nesting);
}
void vPortExitCritical(void) {
	if (critical.nesting == 0) {
		assertFault("taskEXIT_CRITICAL without taskENTER_CRITICAL (port.c:422)");
		return;
	}
	critical.nesting--;
	critical.exit++;
	if (critical.nesting == 0) {
		critical.basepri = 0; // portENABLE_INTERRUPTS (port.c:426)
	}
}
uint32_t ulPortRaiseBASEPRI(void) {
	const uint32_t previous = critical.basepri;
	critical.basepri = maxSyscallBasepri;
	critical.raiseBasepri++;
	return previous;
}
void vPortRaiseBASEPRI(void) {
	critical.basepri = maxSyscallBasepri;
	critical.disableInterrupts++;
}
void vPortSetBASEPRI(uint32_t ulNewMaskValue) {
	critical.basepri = ulNewMaskValue;
	critical.setBasepri++;
}

// ---- Semaphores and mutexes (semphr.h macros -> queue.c) ----
static SemaphoreHandle_t createSemaphore(UBaseType_t count, UBaseType_t maxCount, uint8_t type, RtosObject kind) {
	if (createFails(kind)) {
		return nullptr;
	}
	liveSemaphores++;
	HostSemaphore *sem = new HostSemaphore{count, maxCount, ++semaphoreGenerations};
	sem->type = type;
	semaphores()[sem] = sem->generation;
	return sem;
}
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return createSemaphore(0, 1, queueQUEUE_TYPE_BINARY_SEMAPHORE, RtosObject::BinarySemaphore); }
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t maxCount, UBaseType_t initialCount) {
	if (maxCount == 0 || initialCount > maxCount) {
		(void)createFails(RtosObject::CountingSemaphore);
		assertFault("xSemaphoreCreateCounting: maxCount 0 or initialCount > maxCount (queue.c:718-719)");
		return nullptr;
	}
	return createSemaphore(initialCount, maxCount, queueQUEUE_TYPE_COUNTING_SEMAPHORE, RtosObject::CountingSemaphore);
}
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return createSemaphore(1, 1, queueQUEUE_TYPE_MUTEX, RtosObject::Mutex); }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) { return createSemaphore(1, 1, queueQUEUE_TYPE_RECURSIVE_MUTEX, RtosObject::RecursiveMutex); }

static bool checkSemaphore(SemaphoreHandle_t sem, const char *api) {
	if (sem == nullptr) {
		assertFault(std::string(api) + ": NULL semaphore handle (configASSERT( pxQueue ), queue.c)");
		return false;
	}
	if (semaphores().count(sem) == 0) {
		assertFault(std::string(api) + ": handle of a deleted semaphore (vSemaphoreDelete freed it)");
		return false;
	}
	return true;
}
static bool isMutex(const HostSemaphore *sem) { return sem->type == queueQUEUE_TYPE_MUTEX || sem->type == queueQUEUE_TYPE_RECURSIVE_MUTEX; }

void vSemaphoreDelete(SemaphoreHandle_t sem) { // vQueueDelete, queue.c:1975
	if (sem == nullptr) {
		assertFault("vSemaphoreDelete(NULL) (configASSERT( pxQueue ), queue.c:1979)");
		return;
	}
	if (!checkSemaphore(sem, "vSemaphoreDelete")) {
		return;
	}
	liveSemaphores--;
	semaphores().erase(sem);
	delete sem;
}

/** The wait of xQueueSemaphoreTake (queue.c:1418); takes the semaphore and records the mutex holder */
static BaseType_t takeSemaphore(SemaphoreHandle_t sem, TickType_t ticksToWait, const char *api) {
	checkSuspendedBlockTime(api, ticksToWait, "queue.c:1438");
	const uint64_t generation = sem->generation;
	if (!waitUntil(api, sem, [sem, generation] { return semaphoreLive(sem, generation) && sem->count > 0; }, ticksToWait)) {
		return pdFALSE;
	}
	sem->count--;
	if (isMutex(sem)) {
		sem->held = true;
		sem->holder = currentTask;
	}
	return pdTRUE;
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticksToWait) {
	if (!checkSemaphore(sem, "xSemaphoreTake")) {
		return pdFALSE;
	}
	checkTaskApi("xSemaphoreTake");
	return takeSemaphore(sem, ticksToWait, "xSemaphoreTake");
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) { // xQueueGenericSend with a NULL item (queue.c:740)
	if (!checkSemaphore(sem, "xSemaphoreGive")) {
		return pdFALSE;
	}
	checkTaskApi("xSemaphoreGive");
	if (isMutex(sem) && sem->held && sem->holder != currentTask) {
		assertFault("xSemaphoreGive of a mutex the caller does not hold (xTaskPriorityDisinherit, tasks.c:4115)");
	}
	if (sem->count < sem->maxCount) {
		sem->count++;
		if (isMutex(sem)) {
			sem->held = false;
			sem->holder = nullptr;
			sem->recursion = 0;
		}
		return pdTRUE;
	}
	return errQUEUE_FULL;
}
BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t sem, BaseType_t *higherPriorityTaskWoken) { // xQueueReceiveFromISR, queue.c:1785
	if (!checkSemaphore(sem, "xSemaphoreTakeFromISR")) {
		return pdFALSE;
	}
	(void)higherPriorityTaskWoken; // Taking wakes no task in the fake (nobody waits to give)
	if (sem->count > 0) {
		sem->count--;
		return pdTRUE;
	}
	return pdFALSE;
}
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t sem, BaseType_t *higherPriorityTaskWoken) { // xQueueGiveFromISR, queue.c:1112
	if (!checkSemaphore(sem, "xSemaphoreGiveFromISR")) {
		return pdFALSE;
	}
	if (isMutex(sem) && sem->held) {
		assertFault("xSemaphoreGiveFromISR of a held mutex (queue.c:1133)");
	}
	if (sem->count < sem->maxCount) {
		sem->count++;
		if (isMutex(sem)) {
			sem->held = false;
			sem->holder = nullptr;
		}
		wakeCheck(sem, higherPriorityTaskWoken);
		return pdTRUE;
	}
	return errQUEUE_FULL;
}
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t ticksToWait) { // xQueueTakeMutexRecursive, queue.c:644
	if (!checkSemaphore(sem, "xSemaphoreTakeRecursive")) {
		return pdFAIL;
	}
	checkTaskApi("xSemaphoreTakeRecursive");
	if (sem->type != queueQUEUE_TYPE_RECURSIVE_MUTEX) {
		unmodelled("xSemaphoreTakeRecursive on a semaphore that is not a recursive mutex");
		return pdFAIL;
	}
	if (sem->held && sem->holder == currentTask) {
		sem->recursion++;
		return pdPASS;
	}
	if (takeSemaphore(sem, ticksToWait, "xSemaphoreTakeRecursive") != pdTRUE) {
		return pdFAIL;
	}
	sem->recursion = 1;
	return pdPASS;
}
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t sem) { // xQueueGiveMutexRecursive, queue.c:589
	if (!checkSemaphore(sem, "xSemaphoreGiveRecursive")) {
		return pdFAIL;
	}
	checkTaskApi("xSemaphoreGiveRecursive");
	if (sem->type != queueQUEUE_TYPE_RECURSIVE_MUTEX) {
		unmodelled("xSemaphoreGiveRecursive on a semaphore that is not a recursive mutex");
		return pdFAIL;
	}
	if (!sem->held || sem->holder != currentTask) {
		return pdFAIL; // Not the holder: no assert, just fails (queue.c:625)
	}
	sem->recursion--;
	if (sem->recursion == 0) {
		sem->count++;
		sem->held = false;
		sem->holder = nullptr;
	}
	return pdPASS;
}
UBaseType_t uxSemaphoreGetCount(SemaphoreHandle_t sem) { // uxQueueMessagesWaiting, queue.c:1930
	if (!checkSemaphore(sem, "uxSemaphoreGetCount")) {
		return 0;
	}
	checkTaskApi("uxSemaphoreGetCount");
	return sem->count;
}

// ---- Queues (queue.c) ----
QueueHandle_t xQueueGenericCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, const uint8_t) { // queue.c:370
	if (createFails(RtosObject::Queue)) {
		return nullptr;
	}
	if (uxQueueLength == 0) {
		assertFault("xQueueCreate with a length of 0 (queue.c:374)");
		return nullptr;
	}
	return createQueue(uxQueueLength, uxItemSize, false);
}

BaseType_t xQueueGenericSend(QueueHandle_t queue, const void *const item, TickType_t ticksToWait, const BaseType_t position) { // queue.c:740
	const char *api = sendName(position);
	if (!checkQueue(queue, api)) {
		return errQUEUE_FULL;
	}
	checkTaskApi(api);
	checkSuspendedBlockTime(api, ticksToWait, "queue.c:751");
	if (!checkSendArguments(queue, item, position, api)) {
		return errQUEUE_FULL;
	}
	const uint64_t generation = queue->generation;
	if (!waitUntil(api, queue,
				   [queue, generation, position] { return queueLive(queue, generation) && (queue->items.size() < queue->length || position == queueOVERWRITE); },
				   ticksToWait)) {
		return errQUEUE_FULL;
	}
	copyToQueue(queue, item, position);
	return pdPASS;
}

BaseType_t xQueueGenericSendFromISR(QueueHandle_t queue, const void *const item, BaseType_t *const higherPriorityTaskWoken, const BaseType_t position) { // queue.c:950
	const char *api = position == queueSEND_TO_FRONT ? "xQueueSendToFrontFromISR" : position == queueOVERWRITE ? "xQueueOverwriteFromISR" : "xQueueSendToBackFromISR";
	if (!checkQueue(queue, api) || !checkSendArguments(queue, item, position, api)) {
		return errQUEUE_FULL;
	}
	if (queue->items.size() < queue->length || position == queueOVERWRITE) {
		copyToQueue(queue, item, position);
		wakeCheck(queue, higherPriorityTaskWoken);
		return pdPASS;
	}
	return errQUEUE_FULL;
}

static BaseType_t receive(QueueHandle_t queue, void *const buffer, TickType_t ticksToWait, bool remove, const char *api) {
	if (!checkQueue(queue, api)) {
		return pdFALSE;
	}
	checkTaskApi(api);
	checkSuspendedBlockTime(api, ticksToWait, remove ? "queue.c:1293" : "queue.c:1653");
	if (buffer == nullptr && queue->itemSize != 0) {
		assertFault(std::string(api) + ": NULL buffer for a queue with item size > 0 (queue.c:1288)");
		return pdFALSE;
	}
	const uint64_t generation = queue->generation;
	if (!waitUntil(api, queue, [queue, generation] { return queueLive(queue, generation) && !queue->items.empty(); }, ticksToWait)) {
		return errQUEUE_EMPTY;
	}
	copyFromQueue(queue, buffer, remove);
	return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *const buffer, TickType_t ticksToWait) { return receive(queue, buffer, ticksToWait, true, "xQueueReceive"); } // queue.c:1277
BaseType_t xQueuePeek(QueueHandle_t queue, void *const buffer, TickType_t ticksToWait) { return receive(queue, buffer, ticksToWait, false, "xQueuePeek"); }	 // queue.c:1636

BaseType_t xQueueReceiveFromISR(QueueHandle_t queue, void *const buffer, BaseType_t *const higherPriorityTaskWoken) { // queue.c:1785
	if (!checkQueue(queue, "xQueueReceiveFromISR")) {
		return pdFAIL;
	}
	if (buffer == nullptr && queue->itemSize != 0) {
		assertFault("xQueueReceiveFromISR: NULL buffer for a queue with item size > 0 (queue.c:1792)");
		return pdFAIL;
	}
	if (queue->items.empty()) {
		return pdFAIL;
	}
	copyFromQueue(queue, buffer, true);
	wakeCheck(queue, higherPriorityTaskWoken); // A task waiting to send
	return pdPASS;
}
BaseType_t xQueuePeekFromISR(QueueHandle_t queue, void *const buffer) { // queue.c:1876
	if (!checkQueue(queue, "xQueuePeekFromISR")) {
		return pdFAIL;
	}
	if ((buffer == nullptr && queue->itemSize != 0) || queue->itemSize == 0) {
		assertFault("xQueuePeekFromISR: NULL buffer, or a queue with item size 0 (queue.c:1884-1885)");
		return pdFAIL;
	}
	if (queue->items.empty()) {
		return pdFAIL;
	}
	copyFromQueue(queue, buffer, false);
	return pdPASS;
}

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t queue) { // queue.c:1930
	if (!checkQueue(queue, "uxQueueMessagesWaiting")) {
		return 0;
	}
	checkTaskApi("uxQueueMessagesWaiting");
	return queue->items.size();
}
UBaseType_t uxQueueSpacesAvailable(const QueueHandle_t queue) { // queue.c:1946
	if (!checkQueue(queue, "uxQueueSpacesAvailable")) {
		return 0;
	}
	checkTaskApi("uxQueueSpacesAvailable");
	return queue->length - queue->items.size();
}
UBaseType_t uxQueueMessagesWaitingFromISR(const QueueHandle_t queue) { // queue.c:1963
	return checkQueue(queue, "uxQueueMessagesWaitingFromISR") ? queue->items.size() : 0;
}
BaseType_t xQueueIsQueueEmptyFromISR(const QueueHandle_t queue) { // queue.c:2312
	return checkQueue(queue, "xQueueIsQueueEmptyFromISR") && queue->items.empty() ? pdTRUE : pdFALSE;
}
BaseType_t xQueueIsQueueFullFromISR(const QueueHandle_t queue) { // queue.c:2352
	return checkQueue(queue, "xQueueIsQueueFullFromISR") && queue->items.size() >= queue->length ? pdTRUE : pdFALSE;
}
BaseType_t xQueueGenericReset(QueueHandle_t queue, BaseType_t) { // queue.c:255
	if (!checkQueue(queue, "xQueueReset")) {
		return pdPASS;
	}
	checkTaskApi("xQueueReset");
	queue->items.clear(); // A task waiting to send can continue (its wait condition holds now)
	return pdPASS;
}
void vQueueDelete(QueueHandle_t queue) { // queue.c:1975
	if (!checkQueue(queue, "vQueueDelete")) {
		return;
	}
	queues().erase(queue);
	delete queue;
}

// ---- Software timers (timers.c) ----
TimerHandle_t xTimerCreate(const char *const name, const TickType_t period, const UBaseType_t autoReload, void *const id, TimerCallbackFunction_t callback) { // timers.c:280
	if (createFails(RtosObject::Timer)) {
		return nullptr;
	}
	if (period == 0) {
		assertFault("xTimerCreate with a period of 0 (timers.c:357)");
		return nullptr;
	}
	ensureTimerService();
	HostTimer *timer = new HostTimer();
	timer->name = name;
	timer->period = period;
	timer->autoReload = autoReload != pdFALSE;
	timer->id = id;
	timer->callback = callback;
	timer->generation = ++objectGenerations;
	timers()[timer] = timer->generation;
	return timer;
}

BaseType_t xTimerGenericCommand(TimerHandle_t timer, const BaseType_t id, const TickType_t value, BaseType_t *const higherPriorityTaskWoken, const TickType_t ticksToWait) { // timers.c:381
	if (!timerKnown(timer, "xTimerGenericCommand")) {
		return pdFAIL;
	}
	ensureTimerService();
	const TimerMessage message{id, value, timer, nullptr, nullptr, 0};
	if (id < tmrFIRST_FROM_ISR_COMMAND) {
		return xQueueGenericSend(timerQueue, &message, xTaskGetSchedulerState() == taskSCHEDULER_RUNNING ? ticksToWait : 0, queueSEND_TO_BACK);
	}
	return xQueueGenericSendFromISR(timerQueue, &message, higherPriorityTaskWoken, queueSEND_TO_BACK);
}

BaseType_t xTimerPendFunctionCall(PendedFunction_t function, void *parameter1, uint32_t parameter2, TickType_t ticksToWait) { // timers.c:1072
	ensureTimerService();
	const TimerMessage message{tmrCOMMAND_EXECUTE_CALLBACK, 0, nullptr, function, parameter1, parameter2};
	return xQueueGenericSend(timerQueue, &message, ticksToWait, queueSEND_TO_BACK);
}
BaseType_t xTimerPendFunctionCallFromISR(PendedFunction_t function, void *parameter1, uint32_t parameter2, BaseType_t *higherPriorityTaskWoken) { // timers.c:1047
	ensureTimerService();
	const TimerMessage message{tmrCOMMAND_EXECUTE_CALLBACK_FROM_ISR, 0, nullptr, function, parameter1, parameter2};
	return xQueueGenericSendFromISR(timerQueue, &message, higherPriorityTaskWoken, queueSEND_TO_BACK);
}

void *pvTimerGetTimerID(const TimerHandle_t timer) { // timers.c:1017
	if (!timerKnown(timer, "pvTimerGetTimerID")) {
		return nullptr;
	}
	checkTaskApi("pvTimerGetTimerID");
	return timer->id;
}
void vTimerSetTimerID(TimerHandle_t timer, void *id) { // timers.c:1034
	if (timerKnown(timer, "vTimerSetTimerID")) {
		checkTaskApi("vTimerSetTimerID");
		timer->id = id;
	}
}
BaseType_t xTimerIsTimerActive(TimerHandle_t timer) { // timers.c:992
	if (!timerKnown(timer, "xTimerIsTimerActive")) {
		return pdFALSE;
	}
	checkTaskApi("xTimerIsTimerActive");
	return timer->active ? pdTRUE : pdFALSE;
}
TaskHandle_t xTimerGetTimerDaemonTaskHandle(void) { // timers.c:424
	ensureTimerService();
	return timerService;
}
const char *pcTimerGetName(TimerHandle_t timer) { return timerKnown(timer, "pcTimerGetName") ? timer->name : nullptr; } // timers.c:498
void vTimerSetReloadMode(TimerHandle_t timer, const UBaseType_t autoReload) {											  // timers.c:442
	if (timerKnown(timer, "vTimerSetReloadMode")) {
		checkTaskApi("vTimerSetReloadMode");
		timer->autoReload = autoReload != pdFALSE;
	}
}
UBaseType_t uxTimerGetReloadMode(TimerHandle_t timer) { // timers.c:462
	if (!timerKnown(timer, "uxTimerGetReloadMode")) {
		return pdFALSE;
	}
	checkTaskApi("uxTimerGetReloadMode");
	return timer->autoReload ? pdTRUE : pdFALSE;
}
TickType_t xTimerGetPeriod(TimerHandle_t timer) { return timerKnown(timer, "xTimerGetPeriod") ? timer->period : 0; }		 // timers.c:433
TickType_t xTimerGetExpiryTime(TimerHandle_t timer) { return timerKnown(timer, "xTimerGetExpiryTime") ? timer->expiry : 0; } // timers.c:487

// ---- Event groups (event_groups.c) ----
EventGroupHandle_t xEventGroupCreate(void) { // event_groups.c:145
	if (createFails(RtosObject::EventGroup)) {
		return nullptr;
	}
	return createGroup(false);
}
EventGroupHandle_t xEventGroupCreateStatic(StaticEventGroup_t *buffer) { // event_groups.c:93
	if (createFails(RtosObject::StaticEventGroup)) {
		return nullptr;
	}
	if (buffer == nullptr) {
		assertFault("xEventGroupCreateStatic(NULL) (event_groups.c:98)");
		return nullptr;
	}
	return createGroup(true); // The fake keeps its state outside the buffer; the handle is not the buffer address
}

EventBits_t xEventGroupWaitBits(EventGroupHandle_t group, const EventBits_t waitFor, const BaseType_t clearOnExit, const BaseType_t waitAll, TickType_t ticksToWait) { // event_groups.c:311
	if (!checkGroup(group, "xEventGroupWaitBits")) {
		return 0;
	}
	checkTaskApi("xEventGroupWaitBits");
	checkSuspendedBlockTime("xEventGroupWaitBits", ticksToWait, "event_groups.c:325");
	if ((waitFor & eventEVENT_BITS_CONTROL_BYTES) != 0 || waitFor == 0) {
		assertFault("xEventGroupWaitBits: no bits or control bits (bits 24..31) to wait for (event_groups.c:321-322)");
		return 0;
	}
	const EventBits_t current = group->bits;
	if (testWaitCondition(current, waitFor, waitAll != pdFALSE)) {
		if (clearOnExit != pdFALSE) {
			group->bits &= ~waitFor;
		}
		return current;
	}
	if (ticksToWait == 0) {
		return current;
	}
	const uint64_t generation = group->generation;
	EventBits_t result = 0;
	if (blockOnGroup(group, waitFor, clearOnExit != pdFALSE, waitAll != pdFALSE, ticksToWait, "xEventGroupWaitBits", result)) {
		return result & ~eventEVENT_BITS_CONTROL_BYTES;
	}
	if (!groupLive(group, generation)) {
		return 0;
	}
	// Timeout (event_groups.c:415-435): return the current bits, clear them if the condition holds by now
	result = group->bits;
	if (testWaitCondition(result, waitFor, waitAll != pdFALSE) && clearOnExit != pdFALSE) {
		group->bits &= ~waitFor;
	}
	return result & ~eventEVENT_BITS_CONTROL_BYTES;
}

EventBits_t xEventGroupSync(EventGroupHandle_t group, const EventBits_t bitsToSet, const EventBits_t waitFor, TickType_t ticksToWait) { // event_groups.c:191
	if (!checkGroup(group, "xEventGroupSync")) {
		return 0;
	}
	checkTaskApi("xEventGroupSync");
	checkSuspendedBlockTime("xEventGroupSync", ticksToWait, "event_groups.c:202");
	if ((waitFor & eventEVENT_BITS_CONTROL_BYTES) != 0 || waitFor == 0) {
		assertFault("xEventGroupSync: no bits or control bits (bits 24..31) to wait for (event_groups.c:198-199)");
		return 0;
	}
	// Sync calls SetBits in the supplied kernel (event_groups.c:210), including its argument assertion (:531).
	if ((bitsToSet & eventEVENT_BITS_CONTROL_BYTES) != 0) {
		assertFault("xEventGroupSetBits of control bits (event_groups.c:531)");
		return 0;
	}
	const EventBits_t original = group->bits;
	(void)setGroupBits(group, bitsToSet);
	if (((original | bitsToSet) & waitFor) == waitFor) {
		group->bits &= ~waitFor; // Rendezvous always clears the bits
		return original | bitsToSet;
	}
	if (ticksToWait == 0) {
		return group->bits;
	}
	const uint64_t generation = group->generation;
	EventBits_t result = 0;
	if (blockOnGroup(group, waitFor, true, true, ticksToWait, "xEventGroupSync", result)) {
		return result & ~eventEVENT_BITS_CONTROL_BYTES;
	}
	if (!groupLive(group, generation)) {
		return 0;
	}
	result = group->bits; // Timeout (event_groups.c:270-285)
	if ((result & waitFor) == waitFor) {
		group->bits &= ~waitFor;
	}
	return result & ~eventEVENT_BITS_CONTROL_BYTES;
}

EventBits_t xEventGroupClearBits(EventGroupHandle_t group, const EventBits_t bitsToClear) { // event_groups.c:461
	if (!checkGroup(group, "xEventGroupClearBits")) {
		return 0;
	}
	checkTaskApi("xEventGroupClearBits");
	if ((bitsToClear & eventEVENT_BITS_CONTROL_BYTES) != 0) {
		assertFault("xEventGroupClearBits of control bits (bits 24..31) (event_groups.c:469)");
		return group->bits;
	}
	const EventBits_t previous = group->bits;
	group->bits &= ~bitsToClear;
	return previous;
}
EventBits_t xEventGroupSetBits(EventGroupHandle_t group, const EventBits_t bitsToSet) { // event_groups.c:519
	if (!checkGroup(group, "xEventGroupSetBits")) {
		return 0;
	}
	checkTaskApi("xEventGroupSetBits");
	if ((bitsToSet & eventEVENT_BITS_CONTROL_BYTES) != 0) {
		assertFault("xEventGroupSetBits of control bits (bits 24..31) (event_groups.c:531)");
		return group->bits;
	}
	return setGroupBits(group, bitsToSet);
}
EventBits_t xEventGroupGetBitsFromISR(EventGroupHandle_t group) { // event_groups.c:503
	return checkGroup(group, "xEventGroupGetBitsFromISR") ? group->bits : 0;
}
void vEventGroupDelete(EventGroupHandle_t group) { // event_groups.c:613
	if (!checkGroup(group, "vEventGroupDelete")) {
		return;
	}
	checkTaskApi("vEventGroupDelete");
	for (EventGroupWaiter *waiter : group->waiters) {
		waiter->unblocked = true; // Unblocked with no bits set (event_groups.c:627)
		waiter->result = eventUNBLOCKED_DUE_TO_BIT_SET;
	}
	group->waiters.clear();
	eventGroups().erase(group);
	delete group;
}
void vEventGroupSetBitsCallback(void *group, const uint32_t bitsToSet) { // event_groups.c:657
	(void)xEventGroupSetBits(static_cast<EventGroupHandle_t>(group), (EventBits_t)bitsToSet);
}
void vEventGroupClearBitsCallback(void *group, const uint32_t bitsToClear) { // event_groups.c:665
	(void)xEventGroupClearBits(static_cast<EventGroupHandle_t>(group), (EventBits_t)bitsToClear);
}

} // extern "C"
