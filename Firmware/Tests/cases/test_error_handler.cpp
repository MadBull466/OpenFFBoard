/*
 * Error handling (ErrorHandler.cpp): global error list, de-duplication, clearing,
 * sorting, string formatting, callbacks to all registered ErrorHandler instances
 * and the ErrorPrinter thread.
 *
 * The error list is a static array of ERRORHANDLER_MAXERRORS entries. The fixture
 * resets it with ErrorHandler::clearAll() (FirmwareFixture) before and after every test.
 *
 * Not covered here: the branch of ErrorPrinter::Run() that sends the error list
 * (needs SystemCommands::systemCommandsInstance, SystemCommands.cpp is not compiled
 * in ffb_unit_tests, see host_stubs.cpp).
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FirmwareFixture.h"
#include "MockCommandInterface.h"

#include "ErrorHandler.h"

#include <algorithm>
#include <string>
#include <vector>

namespace {

/** Records every callback it receives */
class RecordingErrorHandler : public ErrorHandler {
public:
	struct Call {
		Error error;
		bool cleared;
	};
	void errorCallback(const Error &error, bool cleared) override { calls.push_back({error, cleared}); }

	size_t countAdded() const {
		return std::count_if(calls.begin(), calls.end(), [](const Call &c) { return !c.cleared; });
	}
	size_t countCleared() const {
		return std::count_if(calls.begin(), calls.end(), [](const Call &c) { return c.cleared; });
	}

	std::vector<Call> calls;
};

/**
 * Read-only view of the protected error storage through a derived class.
 * Workaround for the getErrors() KNOWN_ISSUE (empty slots sorted to the front hide stored errors
 * after any clear). Only used to check WHICH errors are stored and in which relative order,
 * never their slot positions, and never to write.
 */
class ErrorListProbe : public ErrorHandler {
public:
	static const std::array<Error, ERRORHANDLER_MAXERRORS> &raw() { return errors; }
};

struct ErrorFixture : FirmwareFixture {
	static std::vector<Error> active() {
		std::span<Error> s = ErrorHandler::getErrors();
		return std::vector<Error>(s.begin(), s.end());
	}
	static bool isRegistered(ErrorHandler *h) {
		auto &v = ErrorHandler::errorHandlers;
		return std::find(v.begin(), v.end(), h) != v.end();
	}
	static size_t storedCount() {
		auto &raw = ErrorListProbe::raw();
		return std::count_if(raw.begin(), raw.end(), [](const Error &e) { return e.isError(); });
	}
	/** Codes of the stored errors in storage order, empty slots skipped */
	static std::vector<ErrorCode> storedCodes() {
		std::vector<ErrorCode> codes;
		for (const Error &e : ErrorListProbe::raw()) {
			if (e.isError()) codes.push_back(e.code);
		}
		return codes;
	}
	/** True if an equal error (code and type) is stored anywhere in the list, regardless of getErrors() */
	static bool stored(const Error &e) {
		auto &raw = ErrorListProbe::raw();
		return std::find(raw.begin(), raw.end(), e) != raw.end();
	}
};

const Error underVoltage(ErrorCode::undervoltage, ErrorType::warning, "low");
const Error overTemp(ErrorCode::overtemp, ErrorType::critical, "hot");
const Error encoderTemp(ErrorCode::encoderIndexMissed, ErrorType::temporary, "idx");

} // namespace

TEST_SUITE("ErrorHandler") {

// ---------------- Error ----------------

TEST_CASE("a default constructed error is no error") {
	Error e;
	CHECK(e.code == ErrorCode::none);
	CHECK(e.type == ErrorType::none);
	CHECK_FALSE(e.isError());
}

TEST_CASE("an error needs both a code and a type to count as error") {
	CHECK(Error(ErrorCode::overtemp, ErrorType::warning, "").isError());
	CHECK_FALSE(Error(ErrorCode::none, ErrorType::warning, "").isError());
	CHECK_FALSE(Error(ErrorCode::overtemp, ErrorType::none, "").isError());
}

TEST_CASE("errors are equal if code and type match, the info text is ignored") {
	CHECK(Error(ErrorCode::overtemp, ErrorType::warning, "a") == Error(ErrorCode::overtemp, ErrorType::warning, "b"));
	CHECK_FALSE(Error(ErrorCode::overtemp, ErrorType::warning, "a") == Error(ErrorCode::overtemp, ErrorType::critical, "a"));
	CHECK_FALSE(Error(ErrorCode::overtemp, ErrorType::warning, "a") == Error(ErrorCode::undervoltage, ErrorType::warning, "a"));
}

TEST_CASE("toString formats code, type and info for every error type") {
	CHECK(Error(ErrorCode::undervoltage, ErrorType::warning, "low").toString() == "10:warn:low");
	CHECK(Error(ErrorCode::overtemp, ErrorType::critical, "hot").toString() == "15:crit:hot");
	CHECK(Error(ErrorCode::encoderIndexMissed, ErrorType::temporary, "idx").toString() == "23:info:idx");
	CHECK(Error(ErrorCode::axisOutOfRange, ErrorType::none, "x").toString() == "31:err:x");
}

TEST_CASE("toString prints no info for an empty info text") {
	CHECK(Error(ErrorCode::externalConfigurationError, ErrorType::critical, "").toString() == "32:crit:no info");
	CHECK(Error().toString() == "0:err:no info");
}

TEST_CASE("characterization: systemError and cmdNotFound share the error code 5") {
	// NEEDS REVIEW 10: ErrorHandler.h defines systemError = 5 and cmdNotFound = 5. Clearing one by code
	// also clears the other and both print as "5:...". Intended alias or numbering mistake?
	CHECK(static_cast<uint32_t>(ErrorCode::systemError) == 5);
	CHECK(static_cast<uint32_t>(ErrorCode::cmdNotFound) == 5);
	CHECK(Error(ErrorCode::systemError, ErrorType::critical, "") == Error(ErrorCode::cmdNotFound, ErrorType::critical, ""));
}

// ---------------- addError ----------------

TEST_CASE_FIXTURE(ErrorFixture, "the error list is empty after clearAll") {
	CHECK(ErrorHandler::getErrors().empty());
	CHECK(storedCount() == 0);
}

TEST_CASE_FIXTURE(ErrorFixture, "added errors are listed in the order they were added") {
	ErrorHandler::addError(overTemp);
	ErrorHandler::addError(underVoltage);
	auto list = active();
	REQUIRE(list.size() == 2);
	CHECK(list[0] == overTemp);
	CHECK(list[0].info == "hot");
	CHECK(list[1] == underVoltage);
	CHECK(list[1].info == "low");
}

TEST_CASE_FIXTURE(ErrorFixture, "an error that is already active is not added again") {
	ErrorHandler::addError(underVoltage);
	ErrorHandler::addError(Error(ErrorCode::undervoltage, ErrorType::warning, "other text"));
	auto list = active();
	REQUIRE(list.size() == 1);
	CHECK(list[0].info == "low"); // The first info text is kept
}

TEST_CASE_FIXTURE(ErrorFixture, "the same code with a different type is a separate error") {
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::warning, ""));
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::critical, ""));
	CHECK(active().size() == 2);
}

TEST_CASE_FIXTURE(ErrorFixture, "adding an error notifies every registered handler once") {
	RecordingErrorHandler a, b;
	ErrorHandler::addError(overTemp);
	REQUIRE(a.calls.size() == 1);
	REQUIRE(b.calls.size() == 1);
	CHECK(a.calls[0].error == overTemp);
	CHECK(a.calls[0].error.info == "hot");
	CHECK_FALSE(a.calls[0].cleared);
	CHECK(b.calls[0].error == overTemp);
	CHECK_FALSE(b.calls[0].cleared);
}

TEST_CASE_FIXTURE(ErrorFixture, "adding a duplicate error does not notify the handlers") {
	ErrorHandler::addError(overTemp);
	RecordingErrorHandler a;
	ErrorHandler::addError(overTemp);
	CHECK(a.calls.empty());
}

TEST_CASE_FIXTURE(ErrorFixture, "a full error list keeps its errors and still notifies the handlers of a new one") {
	// Fill all ERRORHANDLER_MAXERRORS slots with distinct errors (code x type)
	const ErrorCode codes[] = {ErrorCode::shutdown, ErrorCode::emergencyStop, ErrorCode::brakeResistorFailure,
							   ErrorCode::cmdExecutionError, ErrorCode::undervoltage, ErrorCode::overvoltage};
	const ErrorType types[] = {ErrorType::warning, ErrorType::critical, ErrorType::temporary};
	std::vector<Error> added;
	for (ErrorCode c : codes) {
		for (ErrorType t : types) {
			if (added.size() < ERRORHANDLER_MAXERRORS) {
				added.emplace_back(c, t, "");
			}
		}
	}
	REQUIRE(added.size() == ERRORHANDLER_MAXERRORS);
	for (const Error &e : added) {
		ErrorHandler::addError(e);
	}
	REQUIRE(active().size() == ERRORHANDLER_MAXERRORS);

	RecordingErrorHandler rec;
	const Error extra(ErrorCode::axisOutOfRange, ErrorType::critical, "extra");
	ErrorHandler::addError(extra);

	auto list = active();
	CHECK(list.size() == ERRORHANDLER_MAXERRORS);
	CHECK(std::find(list.begin(), list.end(), extra) == list.end()); // Not stored
	CHECK(std::equal(list.begin(), list.end(), added.begin()));		 // Existing errors unchanged
	REQUIRE(rec.calls.size() == 1);									 // But reported
	CHECK(rec.calls[0].error == extra);
	CHECK_FALSE(rec.calls[0].cleared);
}

TEST_CASE_FIXTURE(ErrorFixture, "a slot freed by clearing an error in a full list is used by the next new error") {
	// Fill all ERRORHANDLER_MAXERRORS slots with distinct errors (code x type)
	const ErrorCode codes[] = {ErrorCode::shutdown, ErrorCode::emergencyStop, ErrorCode::brakeResistorFailure,
							   ErrorCode::cmdExecutionError, ErrorCode::undervoltage, ErrorCode::overvoltage};
	const ErrorType types[] = {ErrorType::warning, ErrorType::critical, ErrorType::temporary};
	std::vector<Error> added;
	for (ErrorCode c : codes) {
		for (ErrorType t : types) {
			if (added.size() < ERRORHANDLER_MAXERRORS) {
				added.emplace_back(c, t, "");
			}
		}
	}
	for (const Error &e : added) {
		ErrorHandler::addError(e);
	}
	REQUIRE(storedCount() == ERRORHANDLER_MAXERRORS);

	ErrorHandler::clearError(added[5]);
	REQUIRE(storedCount() == ERRORHANDLER_MAXERRORS - 1);

	const Error first(ErrorCode::axisOutOfRange, ErrorType::critical, "first");
	const Error second(ErrorCode::externalConfigurationError, ErrorType::critical, "second");
	ErrorHandler::addError(first);
	CHECK(stored(first));
	CHECK(storedCount() == ERRORHANDLER_MAXERRORS);
	ErrorHandler::addError(second); // Full again: not stored
	CHECK_FALSE(stored(second));
	CHECK(stored(first));
	CHECK(storedCount() == ERRORHANDLER_MAXERRORS);
}

// ---------------- clearError(const Error&) ----------------

TEST_CASE_FIXTURE(ErrorFixture, "clearing an error removes it from the list") {
	ErrorHandler::addError(overTemp);
	ErrorHandler::clearError(overTemp);
	CHECK(ErrorHandler::getErrors().empty());
	CHECK_FALSE(stored(overTemp));
}

TEST_CASE_FIXTURE(ErrorFixture, "clearing an error matches code and type, not the info text") {
	ErrorHandler::addError(overTemp);
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::warning, "warm"));
	ErrorHandler::clearError(Error(ErrorCode::overtemp, ErrorType::critical, "different text"));
	CHECK_FALSE(stored(overTemp));
	CHECK(stored(Error(ErrorCode::overtemp, ErrorType::warning, "")));
}

TEST_CASE_FIXTURE(ErrorFixture, "clearing an error notifies every handler with the cleared flag") {
	ErrorHandler::addError(overTemp);
	RecordingErrorHandler a, b;
	ErrorHandler::clearError(overTemp);
	REQUIRE(a.calls.size() == 1);
	REQUIRE(b.calls.size() == 1);
	CHECK(a.calls[0].error == overTemp);
	CHECK(a.calls[0].cleared);
	CHECK(b.calls[0].error == overTemp);
	CHECK(b.calls[0].cleared);
}

TEST_CASE_FIXTURE(ErrorFixture, "characterization: clearing an error that is not active still notifies the handlers") {
	// NEEDS REVIEW 11: clearError(const Error&) calls the callbacks unconditionally, also if the error was never set
	RecordingErrorHandler rec;
	ErrorHandler::clearError(overTemp);
	REQUIRE(rec.calls.size() == 1);
	CHECK(rec.calls[0].error == overTemp);
	CHECK(rec.calls[0].cleared);
}

TEST_CASE_FIXTURE(ErrorFixture, "clearing an error that is not active keeps the other errors stored") {
	ErrorHandler::addError(overTemp);
	ErrorHandler::clearError(underVoltage);
	CHECK(stored(overTemp));
	CHECK(storedCount() == 1);
}

TEST_CASE_FIXTURE(ErrorFixture, "the other errors stay listed after one error is cleared" *
						KNOWN_ISSUE("(bug 7) sortErrors() sorts the empty slots (code 0) before the active errors, "
									"getErrors() then stops at the first slot and returns an empty list")) {
	ErrorHandler::addError(underVoltage);
	ErrorHandler::addError(overTemp);
	ErrorHandler::clearError(underVoltage);
	auto list = active();
	REQUIRE(list.size() == 1); // Fails: list is empty although overTemp is still stored
	CHECK(list[0] == overTemp);
}

// ---------------- clearError(ErrorCode) ----------------

TEST_CASE_FIXTURE(ErrorFixture, "clearing by code removes the errors of every type with that code") {
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::warning, ""));
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::critical, ""));
	ErrorHandler::addError(underVoltage);
	ErrorHandler::clearError(ErrorCode::overtemp);
	CHECK_FALSE(stored(Error(ErrorCode::overtemp, ErrorType::warning, "")));
	CHECK_FALSE(stored(Error(ErrorCode::overtemp, ErrorType::critical, "")));
	CHECK(stored(underVoltage));
	CHECK(storedCount() == 1);
}

TEST_CASE_FIXTURE(ErrorFixture, "clearing by code notifies the handlers once per removed error") {
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::warning, ""));
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::critical, ""));
	ErrorHandler::addError(underVoltage);
	RecordingErrorHandler a, b;
	ErrorHandler::clearError(ErrorCode::overtemp);
	CHECK(a.countCleared() == 2);
	CHECK(b.countCleared() == 2);
	CHECK(a.countAdded() == 0);
}

TEST_CASE_FIXTURE(ErrorFixture, "clearing a code that is not active does not notify the handlers") {
	ErrorHandler::addError(underVoltage);
	RecordingErrorHandler rec;
	ErrorHandler::clearError(ErrorCode::overtemp);
	CHECK(rec.calls.empty());
	CHECK(stored(underVoltage));
}

TEST_CASE_FIXTURE(ErrorFixture, "clearing by code tells the handlers which error was cleared" *
						KNOWN_ISSUE("(bug 8) clearError(ErrorCode) empties the slot first and then passes the emptied "
									"Error() to the callbacks (ErrorHandler.cpp:121-123)")) {
	ErrorHandler::addError(overTemp);
	RecordingErrorHandler rec;
	ErrorHandler::clearError(ErrorCode::overtemp);
	REQUIRE(rec.calls.size() == 1);
	CHECK(rec.calls[0].error.code == ErrorCode::overtemp); // Fails: code is ErrorCode::none
	CHECK(rec.calls[0].error.type == ErrorType::critical);
	CHECK(rec.calls[0].cleared);
}

// ---------------- clearTemp ----------------

TEST_CASE_FIXTURE(ErrorFixture, "clearTemp removes a temporary error and keeps the others") {
	ErrorHandler::addError(underVoltage);
	ErrorHandler::addError(encoderTemp);
	ErrorHandler::addError(overTemp);
	ErrorHandler::clearTemp();
	CHECK_FALSE(stored(encoderTemp));
	CHECK(stored(underVoltage));
	CHECK(stored(overTemp));
	CHECK(storedCount() == 2);
}

TEST_CASE_FIXTURE(ErrorFixture, "clearTemp without temporary errors keeps all errors") {
	ErrorHandler::addError(underVoltage);
	ErrorHandler::addError(overTemp);
	ErrorHandler::clearTemp();
	CHECK(storedCount() == 2);
}

TEST_CASE_FIXTURE(ErrorFixture, "characterization: clearTemp does not notify the handlers") {
	// NEEDS REVIEW 12: clearError() and clearAll() report cleared errors to the handlers, clearTemp() does not
	ErrorHandler::addError(encoderTemp);
	RecordingErrorHandler rec;
	ErrorHandler::clearTemp();
	CHECK_FALSE(stored(encoderTemp));
	CHECK(rec.calls.empty());
}

TEST_CASE_FIXTURE(ErrorFixture, "clearTemp removes all temporary errors" *
						KNOWN_ISSUE("(bug 9) clearTemp() leaves the loop after the first temporary error (break in "
									"ErrorHandler.cpp:71) although it is documented to clear the temporary errors")) {
	const Error otherTemp(ErrorCode::encoderReversed, ErrorType::temporary, "rev");
	ErrorHandler::addError(encoderTemp);
	ErrorHandler::addError(otherTemp);
	ErrorHandler::clearTemp();
	CHECK(storedCount() == 0); // Fails: one temporary error is left
	CHECK_FALSE(stored(encoderTemp));
	CHECK_FALSE(stored(otherTemp));
}

// ---------------- clearAll ----------------

TEST_CASE_FIXTURE(ErrorFixture, "clearAll removes every error") {
	ErrorHandler::addError(underVoltage);
	ErrorHandler::addError(overTemp);
	ErrorHandler::clearAll();
	CHECK(ErrorHandler::getErrors().empty());
	CHECK(storedCount() == 0);
}

TEST_CASE_FIXTURE(ErrorFixture, "clearAll reports every active error as cleared to every handler") {
	ErrorHandler::addError(underVoltage);
	ErrorHandler::addError(overTemp);
	RecordingErrorHandler a, b;
	ErrorHandler::clearAll();
	for (RecordingErrorHandler *h : {&a, &b}) {
		CHECK(h->countAdded() == 0);
		std::vector<Error> clearedErrors;
		for (const auto &c : h->calls) {
			if (c.error.isError()) clearedErrors.push_back(c.error);
		}
		REQUIRE(clearedErrors.size() == 2);
		CHECK(clearedErrors[0] == underVoltage);
		CHECK(clearedErrors[1] == overTemp);
	}
}

TEST_CASE_FIXTURE(ErrorFixture, "characterization: clearAll also calls the handlers for every empty slot") {
	// NEEDS REVIEW 13: the callback is called ERRORHANDLER_MAXERRORS times per handler, with Error() for unused slots
	ErrorHandler::addError(overTemp);
	RecordingErrorHandler rec;
	ErrorHandler::clearAll();
	CHECK(rec.calls.size() == ERRORHANDLER_MAXERRORS);
	CHECK(rec.countCleared() == ERRORHANDLER_MAXERRORS);
	CHECK(std::count_if(rec.calls.begin(), rec.calls.end(), [](const auto &c) { return !c.error.isError(); }) ==
		  ERRORHANDLER_MAXERRORS - 1);
}

// ---------------- sortErrors ----------------

TEST_CASE_FIXTURE(ErrorFixture, "sortErrors orders the stored errors by ascending code") {
	ErrorHandler::addError(Error(ErrorCode::axisOutOfRange, ErrorType::warning, ""));
	ErrorHandler::addError(Error(ErrorCode::emergencyStop, ErrorType::critical, ""));
	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::temporary, ""));
	ErrorHandler::addError(Error(ErrorCode::undervoltage, ErrorType::warning, ""));
	REQUIRE(storedCodes() == std::vector<ErrorCode>{ErrorCode::axisOutOfRange, ErrorCode::emergencyStop,
													 ErrorCode::overtemp, ErrorCode::undervoltage}); // Unsorted before
	ErrorHandler::clearError(ErrorCode::shutdown); // Not active: removes nothing, only sorts
	CHECK(storedCodes() == std::vector<ErrorCode>{ErrorCode::emergencyStop, ErrorCode::undervoltage,
												   ErrorCode::overtemp, ErrorCode::axisOutOfRange});
}

TEST_CASE_FIXTURE(ErrorFixture, "clearing an error sorts the remaining errors by code") {
	ErrorHandler::addError(Error(ErrorCode::axisOutOfRange, ErrorType::warning, ""));
	ErrorHandler::addError(Error(ErrorCode::emergencyStop, ErrorType::critical, ""));
	ErrorHandler::addError(underVoltage);
	ErrorHandler::clearError(underVoltage);
	CHECK(storedCodes() == std::vector<ErrorCode>{ErrorCode::emergencyStop, ErrorCode::axisOutOfRange});
}

TEST_CASE_FIXTURE(ErrorFixture, "clearTemp sorts the remaining errors by code") {
	ErrorHandler::addError(Error(ErrorCode::axisOutOfRange, ErrorType::warning, ""));
	ErrorHandler::addError(Error(ErrorCode::emergencyStop, ErrorType::critical, ""));
	ErrorHandler::addError(encoderTemp);
	ErrorHandler::clearTemp();
	CHECK(storedCodes() == std::vector<ErrorCode>{ErrorCode::emergencyStop, ErrorCode::axisOutOfRange});
}

TEST_CASE_FIXTURE(ErrorFixture, "a new error after a sort is sorted in by the next clear") {
	ErrorHandler::addError(Error(ErrorCode::axisOutOfRange, ErrorType::warning, ""));
	ErrorHandler::addError(Error(ErrorCode::emergencyStop, ErrorType::critical, ""));
	ErrorHandler::clearError(ErrorCode::shutdown); // Sorts
	ErrorHandler::addError(underVoltage);
	ErrorHandler::clearError(ErrorCode::shutdown); // Sorts again
	CHECK(storedCodes() == std::vector<ErrorCode>{ErrorCode::emergencyStop, ErrorCode::undervoltage,
												   ErrorCode::axisOutOfRange});
}

// ---------------- Handler registration ----------------

TEST_CASE_FIXTURE(ErrorFixture, "an error handler registers itself on construction and unregisters on destruction") {
	const size_t before = ErrorHandler::errorHandlers.size();
	auto *h = new RecordingErrorHandler();
	CHECK(isRegistered(h));
	CHECK(ErrorHandler::errorHandlers.size() == before + 1);
	delete h;
	CHECK_FALSE(isRegistered(h));
	CHECK(ErrorHandler::errorHandlers.size() == before);
}

TEST_CASE_FIXTURE(ErrorFixture, "a destroyed handler is no longer notified") {
	RecordingErrorHandler kept;
	{
		RecordingErrorHandler temporary;
	}
	ErrorHandler::addError(overTemp);
	CHECK(kept.calls.size() == 1); // The remaining handler still works after the other one was removed
}

TEST_CASE_FIXTURE(ErrorFixture, "the default error callback of the base class ignores errors") {
	auto *plain = new ErrorHandler(); // Registered with the empty default errorCallback
	CHECK(isRegistered(plain));
	ErrorHandler::addError(overTemp);
	CHECK(stored(overTemp));
	ErrorHandler::clearError(overTemp);
	CHECK_FALSE(stored(overTemp));
	delete plain;
	CHECK_FALSE(isRegistered(plain));
}

} // TEST_SUITE

// ---------------- ErrorPrinter ----------------

namespace {

struct ErrorPrinterFixture : ErrorFixture {
	ErrorPrinter printer;
	TaskHandle_t task = HostRtos::findTask("errprint");

	// ErrorPrinter inherits privately from ErrorHandler. It is the last handler registered when the fixture is built
	ErrorHandler *registeredPrinter = ErrorHandler::errorHandlers.empty() ? nullptr : ErrorHandler::errorHandlers.back();
};

} // namespace

TEST_SUITE("ErrorPrinter") {

TEST_CASE_FIXTURE(ErrorPrinterFixture, "the error printer starts its thread and registers as error handler") {
	REQUIRE(task != nullptr);
	CHECK_FALSE(HostRtos::isSuspended(task));
	CHECK(printer.getEnabled());
	REQUIRE(registeredPrinter != nullptr);
	CHECK(isRegistered(registeredPrinter));
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "the error printer thread suspends itself after a pass") {
	REQUIRE(task != nullptr);
	CHECK(HostRtos::runUntilBlocked(task));
	CHECK(HostRtos::isSuspended(task));
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "a new error resumes the error printer thread") {
	REQUIRE(task != nullptr);
	HostRtos::runUntilBlocked(task);
	REQUIRE(HostRtos::isSuspended(task));
	ErrorHandler::addError(overTemp);
	CHECK_FALSE(HostRtos::isSuspended(task));
	// Thread context uses the normal resume function
	CHECK(HostRtos::resumeStats().resume == 1);
	CHECK(HostRtos::resumeStats().resumeFromIsr == 0);
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "a new error reported from an interrupt resumes the error printer thread") {
	REQUIRE(task != nullptr);
	HostRtos::runUntilBlocked(task);
	REQUIRE(HostRtos::isSuspended(task));
	{
		HostPlatform::IsrContext isr;
		ErrorHandler::addError(overTemp);
	}
	CHECK_FALSE(HostRtos::isSuspended(task));
	// Interrupt context must use the ISR safe resume function only
	CHECK(HostRtos::resumeStats().resumeFromIsr == 1);
	CHECK(HostRtos::resumeStats().resume == 0);
	CHECK(HostRtos::resumeStats().resumeInIsr == 0);
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "a cleared error does not resume the error printer thread") {
	REQUIRE(task != nullptr);
	ErrorHandler::addError(overTemp);
	HostRtos::runUntilBlocked(task);
	REQUIRE(HostRtos::isSuspended(task));
	ErrorHandler::clearError(overTemp);
	CHECK(HostRtos::isSuspended(task));
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "a disabled error printer is not resumed by new errors") {
	REQUIRE(task != nullptr);
	printer.setEnabled(false);
	CHECK_FALSE(printer.getEnabled());
	CHECK_FALSE(isRegistered(registeredPrinter));
	HostRtos::runUntilBlocked(task);
	REQUIRE(HostRtos::isSuspended(task));
	ErrorHandler::addError(overTemp);
	CHECK(HostRtos::isSuspended(task));
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "a re-enabled error printer is registered once and resumed by new errors") {
	REQUIRE(task != nullptr);
	const size_t before = ErrorHandler::errorHandlers.size();
	printer.setEnabled(false);
	printer.setEnabled(true);
	printer.setEnabled(true); // No change: not registered twice
	CHECK(printer.getEnabled());
	CHECK(ErrorHandler::errorHandlers.size() == before);
	CHECK(isRegistered(registeredPrinter));
	HostRtos::runUntilBlocked(task);
	ErrorHandler::addError(overTemp);
	CHECK_FALSE(HostRtos::isSuspended(task));
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "disabling twice unregisters the error printer once") {
	RecordingErrorHandler other;
	const size_t before = ErrorHandler::errorHandlers.size();
	printer.setEnabled(false);
	printer.setEnabled(false);
	CHECK(ErrorHandler::errorHandlers.size() == before - 1);
	CHECK(isRegistered(&other));
}

TEST_CASE_FIXTURE(ErrorPrinterFixture, "without system commands the error printer sends nothing and keeps temporary errors") {
	// SystemCommands::systemCommandsInstance is nullptr in ffb_unit_tests (SystemCommands.cpp not compiled)
	REQUIRE(task != nullptr);
	REQUIRE(SystemCommands::systemCommandsInstance == nullptr);
	MockCommandInterface itf;
	ErrorHandler::addError(encoderTemp);
	HostRtos::runUntilBlocked(task);
	CHECK(itf.replies.empty());
	CHECK(stored(encoderTemp));
	CHECK(HostRtos::isSuspended(task));
}

TEST_CASE_FIXTURE(ErrorFixture, "destroying the error printer deletes its thread and unregisters it") {
	const size_t handlers = ErrorHandler::errorHandlers.size();
	const size_t tasks = HostRtos::taskCount();
	{
		ErrorPrinter printer;
		CHECK(ErrorHandler::errorHandlers.size() == handlers + 1);
		CHECK(HostRtos::taskCount() == tasks + 1);
	}
	CHECK(ErrorHandler::errorHandlers.size() == handlers);
	CHECK(HostRtos::taskCount() == tasks);
	CHECK(HostRtos::findTask("errprint") == nullptr);
}

} // TEST_SUITE
