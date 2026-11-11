/*
 * Tests of TimerHandler (Src/TimerHandler.cpp) and ExtiHandler (Src/ExtiHandler.cpp): the callback lists of the timer
 * update and external interrupts (ffb_target_tests). The interrupts are raised through the fake HAL
 * (HostTarget::timerUpdate -> HAL_TIM_PeriodElapsedCallback, HostTarget::triggerExti -> HAL_GPIO_EXTI_Callback) and
 * reach the handlers through global_callbacks.cpp like on the target. The lists are not reset by the fixture; every
 * handler is created and deleted inside its test, sizes are compared with the size before the test.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "HostPlatform.h"

#include "ExtiHandler.h"
#include "TimerHandler.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace {

/** Order in which the recording handlers were called (shared by all recorders of a test) */
std::vector<int> &callOrder() {
	static std::vector<int> order;
	return order;
}

struct TimerRecorder : TimerHandler {
	explicit TimerRecorder(int id) : id(id) {}
	void timerElapsed(TIM_HandleTypeDef *htim) override {
		handles.push_back(htim);
		inInterrupt.push_back(inIsr());
		callOrder().push_back(id);
	}
	int id;
	std::vector<TIM_HandleTypeDef *> handles;
	std::vector<bool> inInterrupt;
};

struct ExtiRecorder : ExtiHandler {
	explicit ExtiRecorder(int id) : id(id) {}
	void exti(uint16_t pin) override {
		pins.push_back(pin);
		inInterrupt.push_back(inIsr());
		callOrder().push_back(id);
	}
	int id;
	std::vector<uint16_t> pins;
	std::vector<bool> inInterrupt;
};

struct HandlerFixture : TargetFixture {
	HandlerFixture() {
		callOrder().clear();
		REQUIRE(HAL_TIM_Base_Start_IT(&htim13) == HAL_OK); // FFB rate timer with update interrupt, no handler of its own here
	}
	~HandlerFixture() override { callOrder().clear(); }
	const size_t timersBefore = TimerHandler::timerHandlers.size();
	const size_t extisBefore = ExtiHandler::extiHandlers.size();
};

template <class T, class H>
bool listed(const std::vector<T *> &list, H *handler) {
	return std::find(list.begin(), list.end(), static_cast<T *>(handler)) != list.end();
}

} // namespace

TEST_SUITE("TimerHandler") {

TEST_CASE_FIXTURE(HandlerFixture, "a timer handler is in the timer handler list while it exists") {
	auto handler = std::make_unique<TimerRecorder>(1);
	CHECK(TimerHandler::timerHandlers.size() == timersBefore + 1);
	CHECK(listed(TimerHandler::timerHandlers, handler.get()));
	handler.reset();
	CHECK(TimerHandler::timerHandlers.size() == timersBefore);
}

TEST_CASE_FIXTURE(HandlerFixture, "an update interrupt reaches every timer handler with the timer handle in interrupt context") {
	TimerRecorder first(1), second(2);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(first.handles == std::vector<TIM_HandleTypeDef *>{&htim13});
	CHECK(second.handles == std::vector<TIM_HandleTypeDef *>{&htim13});
	CHECK(first.inInterrupt == std::vector<bool>{true});
	CHECK(second.inInterrupt == std::vector<bool>{true});
}

TEST_CASE_FIXTURE(HandlerFixture, "timer handlers are called in the order they were created") {
	TimerRecorder first(1), second(2), third(3);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(callOrder() == std::vector<int>{1, 2, 3});
}

TEST_CASE_FIXTURE(HandlerFixture, "a deleted timer handler is no longer called and the others keep their order") {
	TimerRecorder first(1);
	auto middle = std::make_unique<TimerRecorder>(2);
	TimerRecorder last(3);
	middle.reset();
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(callOrder() == std::vector<int>{1, 3});
	CHECK(TimerHandler::timerHandlers.size() == timersBefore + 2);
}

TEST_CASE_FIXTURE(HandlerFixture, "a timer handler without its own callback does not stop the dispatch to later handlers") {
	TimerHandler plain; // TimerHandler::timerElapsed: empty default
	TimerRecorder after(1);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(listed(TimerHandler::timerHandlers, &plain));
	CHECK(after.handles.size() == 1); // the dispatch continues after the default callback
}

TEST_CASE_FIXTURE(HandlerFixture, "each update interrupt calls the timer handlers once") {
	TimerRecorder handler(1);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(handler.handles.size() == 2);
}

} // TEST_SUITE

TEST_SUITE("ExtiHandler") {

TEST_CASE_FIXTURE(HandlerFixture, "an EXTI handler is in the EXTI handler list while it exists") {
	auto handler = std::make_unique<ExtiRecorder>(1);
	CHECK(ExtiHandler::extiHandlers.size() == extisBefore + 1);
	CHECK(listed(ExtiHandler::extiHandlers, handler.get()));
	handler.reset();
	CHECK(ExtiHandler::extiHandlers.size() == extisBefore);
}

TEST_CASE_FIXTURE(HandlerFixture, "an external interrupt reaches every EXTI handler with its pin in interrupt context") {
	ExtiRecorder first(1), second(2);
	HostTarget::triggerExti(BUTTON_A_Pin);
	HostTarget::triggerExti(ENCODER_Z_Pin);
	CHECK(first.pins == std::vector<uint16_t>{BUTTON_A_Pin, ENCODER_Z_Pin});
	CHECK(second.pins == std::vector<uint16_t>{BUTTON_A_Pin, ENCODER_Z_Pin});
	CHECK(first.inInterrupt == std::vector<bool>{true, true});
}

TEST_CASE_FIXTURE(HandlerFixture, "EXTI handlers are called in the order they were created") {
	ExtiRecorder first(1), second(2), third(3);
	HostTarget::triggerExti(FLAG_Pin);
	CHECK(callOrder() == std::vector<int>{1, 2, 3});
}

TEST_CASE_FIXTURE(HandlerFixture, "a deleted EXTI handler is no longer called and the others keep their order") {
	auto first = std::make_unique<ExtiRecorder>(1);
	ExtiRecorder second(2), third(3);
	first.reset();
	HostTarget::triggerExti(E_STOP_Pin);
	CHECK(callOrder() == std::vector<int>{2, 3});
	CHECK(ExtiHandler::extiHandlers.size() == extisBefore + 2);
}

TEST_CASE_FIXTURE(HandlerFixture, "an EXTI handler without its own callback does not stop the dispatch to later handlers") {
	ExtiHandler plain; // ExtiHandler::exti: empty default
	ExtiRecorder after(1);
	HostTarget::triggerExti(BUTTON_A_Pin);
	CHECK(listed(ExtiHandler::extiHandlers, &plain));
	CHECK(after.pins.size() == 1);
}

TEST_CASE_FIXTURE(HandlerFixture, "timer and EXTI handlers are separate lists") {
	TimerRecorder timer(1);
	ExtiRecorder exti(2);
	HostTarget::triggerExti(BUTTON_A_Pin);
	CHECK(timer.handles.empty());
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(exti.pins.size() == 1);
	CHECK(callOrder() == std::vector<int>{2, 1});
}

} // TEST_SUITE
