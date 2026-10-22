/*
 * UART port and uart callback handlers (UART.cpp, UartHandler.cpp)
 *
 * Every test uses its own UART handle and UARTPort, so the global external_uart used by the
 * UART command interface is not touched. The fake HAL (platform/src/host_platform.cpp) records
 * transfers and can be told to return errors (HostPlatform::uart()).
 *
 * The HAL completion callbacks (HAL_UART_RxCpltCallback/TxCpltCallback in global_callbacks.cpp,
 * not compiled for the host) call every registered UartHandler with the handle of the uart.
 * rxComplete()/txComplete() below do the same.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "HostPlatform.h"
#include "HostRtos.h"

#include "UART.h"
#include "UartHandler.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

/** Calls the receive complete callback of all uart handlers like HAL_UART_RxCpltCallback */
void rxComplete(UART_HandleTypeDef *huart) {
	HostPlatform::IsrContext isr;
	for (UartHandler *handler : UartHandler::getUARTHandlers()) {
		handler->uartRxComplete(huart);
	}
}

/** Calls the transmit complete callback of all uart handlers like HAL_UART_TxCpltCallback */
void txComplete(UART_HandleTypeDef *huart) {
	HostPlatform::IsrContext isr;
	for (UartHandler *handler : UartHandler::getUARTHandlers()) {
		handler->uartTxComplete(huart);
	}
}

bool isRegistered(UartHandler *handler) {
	auto &handlers = UartHandler::getUARTHandlers();
	return std::count(handlers.begin(), handlers.end(), handler) == 1;
}

/** Uart device recording received bytes and the transfer start/end notifications of the port */
class RecordingDevice : public UARTDevice {
public:
	explicit RecordingDevice(UARTPort &port) : UARTDevice(port) {}

	void uartRcv(char &buf) override {
		received.push_back(buf);
		lastRcvAddress = &buf;
	}
	void startUartTransfer(UARTPort *port, bool transmit) override {
		events.push_back(transmit ? "start tx" : "start rx");
		UARTDevice::startUartTransfer(port, transmit);
	}
	void endUartTransfer(UARTPort *port, bool transmit) override {
		events.push_back(transmit ? "end tx" : "end rx");
		UARTDevice::endUartTransfer(port, transmit);
	}

	std::string received;
	const char *lastRcvAddress = nullptr;
	std::vector<std::string> events;
};

/** Device using the default constructor. Attaches the port itself like a derived class may do */
class LateAttachDevice : public UARTDevice {
public:
	explicit LateAttachDevice(UARTPort &port) : UARTDevice() {
		uartport = &port;
		port.reservePort(this);
	}
};

/** Uart handler recording the callbacks it receives */
class RecordingUartHandler : public UartHandler {
public:
	explicit RecordingUartHandler(std::vector<std::string> *log = nullptr, std::string name = "") : log(log), name(std::move(name)) {}
	void uartRxComplete(UART_HandleTypeDef *huart) override {
		rx.push_back(huart);
		if (log) log->push_back(name + " rx");
	}
	void uartTxComplete(UART_HandleTypeDef *huart) override {
		tx.push_back(huart);
		if (log) log->push_back(name + " tx");
	}
	std::vector<UART_HandleTypeDef *> rx;
	std::vector<UART_HandleTypeDef *> tx;
	std::vector<std::string> *log;
	std::string name;
};

UART_InitTypeDef defaultConfig() {
	UART_InitTypeDef cfg{};
	cfg.BaudRate = 115200;
	cfg.WordLength = 0;
	cfg.StopBits = 0;
	cfg.Parity = 0;
	cfg.Mode = 0x0C;
	cfg.HwFlowCtl = 0;
	cfg.OverSampling = 0;
	return cfg;
}

struct UartPortFixture {
	UART_HandleTypeDef handle{};
	UART_HandleTypeDef otherHandle{};
	UARTPort port{handle};

	UartPortFixture() {
		HostPlatform::reset();
		HostRtos::reset();
		handle.Init = defaultConfig();
	}
	~UartPortFixture() { HostPlatform::reset(); }

	/** True if the transmit side of the port can be taken right now (and gives it back) */
	bool txFree() {
		bool ok = port.takeSemaphore(true, 0);
		if (ok) port.giveSemaphore(true);
		return ok;
	}
	/** True if the receive side of the port can be taken right now (and gives it back) */
	bool rxFree() {
		bool ok = port.takeSemaphore(false, 0);
		if (ok) port.giveSemaphore(false);
		return ok;
	}
};

using Events = std::vector<std::string>;

} // namespace

TEST_SUITE("UART") {

// ---------------------------------------------------------------------------
// Port registration and configuration
// ---------------------------------------------------------------------------

TEST_CASE("a port registers as uart handler and unregisters when destroyed") {
	UART_HandleTypeDef handle{};
	UartHandler *registered = nullptr;
	{
		auto port = std::make_unique<UARTPort>(handle);
		registered = port.get();
		CHECK(isRegistered(registered));
	}
	auto &handlers = UartHandler::getUARTHandlers();
	CHECK(std::find(handlers.begin(), handlers.end(), registered) == handlers.end());
}

TEST_CASE_FIXTURE(UartPortFixture, "getHuart returns the handle of the port") {
	CHECK(port.getHuart() == &handle);
}

TEST_CASE_FIXTURE(UartPortFixture, "getConfig returns the init structure of the handle") {
	CHECK(&port.getConfig() == &handle.Init);
	CHECK(port.getConfig().BaudRate == 115200);
}

TEST_CASE_FIXTURE(UartPortFixture, "getErrors returns the HAL error code of the handle") {
	CHECK(port.getErrors() == 0);
	handle.ErrorCode = 0x0A; // e.g. HAL_UART_ERROR_NE | HAL_UART_ERROR_OE
	CHECK(port.getErrors() == 0x0A);
}

TEST_CASE_FIXTURE(UartPortFixture, "reconfigurePort initialises the uart with a changed configuration") {
	UART_InitTypeDef cfg = defaultConfig();
	cfg.BaudRate = 9600;
	CHECK(port.reconfigurePort(cfg));
	CHECK(HostPlatform::uart().initCount == 1);
	CHECK(handle.Init.BaudRate == 9600);
}

TEST_CASE_FIXTURE(UartPortFixture, "reconfigurePort does not initialise the uart if the configuration is unchanged") {
	UART_InitTypeDef cfg = defaultConfig();
	CHECK_FALSE(port.reconfigurePort(cfg));
	CHECK(HostPlatform::uart().initCount == 0);
}

TEST_CASE_FIXTURE(UartPortFixture, "every field of the configuration is compared by reconfigurePort") {
	UART_InitTypeDef cfg = defaultConfig();
	SUBCASE("BaudRate") { cfg.BaudRate = 57600; }
	SUBCASE("WordLength") { cfg.WordLength = 0x1000; }
	SUBCASE("StopBits") { cfg.StopBits = 0x2000; }
	SUBCASE("Parity") { cfg.Parity = 0x400; }
	SUBCASE("Mode") { cfg.Mode = 0x04; }
	SUBCASE("HwFlowCtl") { cfg.HwFlowCtl = 0x300; }
	SUBCASE("OverSampling") { cfg.OverSampling = 0x8000; }
	CHECK(port.reconfigurePort(cfg));
	CHECK(HostPlatform::uart().initCount == 1);
	CHECK(std::memcmp(&handle.Init, &cfg, sizeof(cfg)) == 0);
}

TEST_CASE_FIXTURE(UartPortFixture, "reconfigurePort reports a failed initialisation") {
	HostPlatform::uart().initResult = HAL_ERROR;
	UART_InitTypeDef cfg = defaultConfig();
	cfg.BaudRate = 9600;
	CHECK_FALSE(port.reconfigurePort(cfg));
	CHECK(HostPlatform::uart().initCount == 1);
}

// NEEDS REVIEW: the configuration is stored in the handle before HAL_UART_Init is called. After a failed
// initialisation the same configuration counts as "unchanged" and is not initialised again. Intended?
TEST_CASE_FIXTURE(UartPortFixture, "characterization: a configuration that failed to initialise is not retried") {
	HostPlatform::uart().initResult = HAL_ERROR;
	UART_InitTypeDef cfg = defaultConfig();
	cfg.BaudRate = 9600;
	CHECK_FALSE(port.reconfigurePort(cfg));
	CHECK(handle.Init.BaudRate == 9600); // Stored although the init failed

	HostPlatform::uart().initResult = HAL_OK;
	CHECK_FALSE(port.reconfigurePort(cfg));
	CHECK(HostPlatform::uart().initCount == 1);
}

// ---------------------------------------------------------------------------
// Reservation by uart devices
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(UartPortFixture, "a port is reserved by a device and released when the device is destroyed") {
	CHECK_FALSE(port.isReserved());
	{
		RecordingDevice device(port);
		CHECK(port.isReserved());
	}
	CHECK_FALSE(port.isReserved());
}

TEST_CASE_FIXTURE(UartPortFixture, "a reserved port cannot be reserved by another device") {
	RecordingDevice owner(port);
	RecordingDevice other(port); // Its reservation fails silently
	CHECK_FALSE(port.reservePort(&other));
	CHECK(port.reservePort(&owner)); // The owner may reserve again
}

TEST_CASE_FIXTURE(UartPortFixture, "a port can only be freed by the device that reserved it") {
	RecordingDevice owner(port);
	RecordingDevice other(port);
	CHECK_FALSE(port.freePort(&other));
	CHECK(port.isReserved());
	CHECK(port.freePort(&owner));
	CHECK_FALSE(port.isReserved());
	CHECK(port.reservePort(&other)); // Free again
	port.freePort(&other);
}

TEST_CASE_FIXTURE(UartPortFixture, "destroying a device that did not get the port keeps the reservation of the owner") {
	RecordingDevice owner(port);
	{
		RecordingDevice other(port);
	}
	CHECK(port.isReserved());
	CHECK_FALSE(port.reservePort(nullptr)); // Still owned by `owner`
}

TEST_CASE_FIXTURE(UartPortFixture, "transfers are reported to the device holding the reservation") {
	RecordingDevice owner(port);
	RecordingDevice other(port);
	CHECK(port.transmit("x", 1));
	CHECK(owner.events == Events{"start tx", "end tx"});
	CHECK(other.events.empty());
}

TEST_CASE_FIXTURE(UartPortFixture, "a device created with the default constructor can attach a port later") {
	{
		LateAttachDevice device(port);
		CHECK(port.isReserved());
	}
	CHECK_FALSE(port.isReserved());
}

TEST_CASE("a device created without a port can be destroyed" * doctest::skip() *
		doctest::description("KNOWN ISSUE (crash): UARTDevice::~UARTDevice calls uartport->freePort(this) without checking uartport, which is nullptr after the default constructor (UART.cpp:212-222). Skipped because it terminates the test run")) {
	{
		UARTDevice device;
	}
	CHECK(true);
}

// ---------------------------------------------------------------------------
// Port locking (semaphores)
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(UartPortFixture, "the transmit and receive side of a port are locked independently") {
	CHECK(port.takeSemaphore(true, 0));
	CHECK(port.takeSemaphore(false, 0));
	CHECK_FALSE(port.takeSemaphore(true, 0));
	CHECK_FALSE(port.takeSemaphore(false, 0));
	CHECK(port.giveSemaphore(true));
	CHECK(port.takeSemaphore(true, 0));
	port.giveSemaphore(true);
	port.giveSemaphore(false);
}

TEST_CASE_FIXTURE(UartPortFixture, "isTaken follows take and give of the port") {
	CHECK_FALSE(port.isTaken());
	port.takeSemaphore(true, 0);
	CHECK(port.isTaken());
	port.giveSemaphore(true);
	CHECK_FALSE(port.isTaken());
}

TEST_CASE_FIXTURE(UartPortFixture, "the port can be taken and given in interrupt context without blocking") {
	HostPlatform::IsrContext isr;
	CHECK(port.takeSemaphore(true));
	CHECK(port.isTaken());
	CHECK_FALSE(port.takeSemaphore(true)); // Fails instead of waiting
	CHECK(port.giveSemaphore(true));
	CHECK_FALSE(port.isTaken());
	CHECK_FALSE(port.giveSemaphore(true)); // Binary semaphore: already given
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(UartPortFixture, "blocking transmit sends the data with the given timeout") {
	CHECK(port.transmit("hello", 5, 123));
	CHECK(HostPlatform::uart().tx == "hello");
	CHECK(HostPlatform::uart().txBlockingCount == 1);
	CHECK(HostPlatform::uart().lastTimeout == 123);
}

TEST_CASE_FIXTURE(UartPortFixture, "blocking transmit uses a timeout of 10 s by default") {
	CHECK(port.transmit("a", 1));
	CHECK(HostPlatform::uart().lastTimeout == 10000);
}

TEST_CASE_FIXTURE(UartPortFixture, "blocking transmit holds the port only during the transfer") {
	RecordingDevice device(port);
	CHECK(port.transmit("abc", 3));
	CHECK(device.events == Events{"start tx", "end tx"});
	CHECK_FALSE(port.isTaken());
	CHECK(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "blocking transmit reports a HAL error and still releases the port") {
	RecordingDevice device(port);
	HostPlatform::uart().txResult = HAL_BUSY;
	CHECK_FALSE(port.transmit("abc", 3));
	CHECK(device.events == Events{"start tx", "end tx"});
	CHECK(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "DMA transmit keeps the port taken until the transfer is complete") {
	RecordingDevice device(port);
	CHECK(port.transmit_DMA("dma", 3));
	CHECK(HostPlatform::uart().tx == "dma");
	CHECK(HostPlatform::uart().txDmaCount == 1);
	CHECK(device.events == Events{"start tx"});
	CHECK(port.isTaken());
	CHECK_FALSE(txFree());

	txComplete(&handle);
	CHECK(device.events == Events{"start tx", "end tx"});
	CHECK_FALSE(port.isTaken());
	CHECK(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "interrupt transmit keeps the port taken until the transfer is complete") {
	RecordingDevice device(port);
	CHECK(port.transmit_IT("it", 2));
	CHECK(HostPlatform::uart().txItCount == 1);
	CHECK(device.events == Events{"start tx"});
	CHECK_FALSE(txFree());

	txComplete(&handle);
	CHECK(device.events == Events{"start tx", "end tx"});
	CHECK(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "DMA and interrupt transmit report HAL errors") {
	HostPlatform::uart().txResult = HAL_ERROR;
	CHECK_FALSE(port.transmit_DMA("a", 1));
	CHECK_FALSE(port.transmit_IT("a", 1));
	CHECK(HostPlatform::uart().tx.empty());
}

TEST_CASE_FIXTURE(UartPortFixture, "transfers without a reserved device do not lock the port") {
	CHECK(port.transmit("a", 1));
	CHECK(port.transmit_DMA("b", 1));
	CHECK(port.transmit_IT("c", 1));
	CHECK(HostPlatform::uart().tx == "abc");
	CHECK_FALSE(port.isTaken());
	CHECK(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "a transmit complete of another uart is ignored") {
	RecordingDevice device(port);
	port.transmit_DMA("a", 1);
	txComplete(&otherHandle);
	CHECK(device.events == Events{"start tx"});
	CHECK(port.isTaken());
}

TEST_CASE_FIXTURE(UartPortFixture, "completions do not release a port that has no device") {
	port.takeSemaphore(true, 0);
	txComplete(&handle);
	rxComplete(&handle);
	CHECK(port.isTaken());
	CHECK_FALSE(txFree());
	port.giveSemaphore(true);
}

TEST_CASE_FIXTURE(UartPortFixture, "a failed interrupt transmit releases the port" * KNOWN_ISSUE("UARTPort::transmit_IT takes the port before HAL_UART_Transmit_IT and only the tx complete interrupt releases it. If the HAL call fails no interrupt follows and the port stays taken forever (UART.cpp:78-83). UART_CommandInterface::Run then waits forever in `while(uartport->isTaken())`")) {
	RecordingDevice device(port);
	HostPlatform::uart().txResult = HAL_BUSY;
	CHECK_FALSE(port.transmit_IT("a", 1));
	CHECK_FALSE(port.isTaken());
	CHECK(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "a failed DMA transmit releases the port" * KNOWN_ISSUE("UARTPort::transmit_DMA takes the port before HAL_UART_Transmit_DMA and only the tx complete interrupt releases it. If the HAL call fails the port stays taken forever (UART.cpp:62-67)")) {
	RecordingDevice device(port);
	HostPlatform::uart().txResult = HAL_ERROR;
	CHECK_FALSE(port.transmit_DMA("a", 1));
	CHECK_FALSE(port.isTaken());
	CHECK(txFree());
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(UartPortFixture, "blocking receive reads the requested bytes with the given timeout") {
	RecordingDevice device(port);
	HostPlatform::uart().rxData = "xyz!";
	char buf[4] = {};
	CHECK(port.receive(buf, 3, 50));
	CHECK(std::string(buf, 3) == "xyz");
	CHECK(HostPlatform::uart().lastTimeout == 50);
	CHECK(HostPlatform::uart().lastRxSize == 3);
	CHECK(device.events == Events{"start rx", "end rx"});
	CHECK(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "blocking receive reports a timeout and releases the port") {
	RecordingDevice device(port);
	char buf[4] = {};
	CHECK_FALSE(port.receive(buf, 4));
	CHECK(HostPlatform::uart().lastTimeout == 10000); // Default timeout
	CHECK(device.events == Events{"start rx", "end rx"});
	CHECK(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "blocking receive works without a device") {
	HostPlatform::uart().rxData = "q";
	char c = 0;
	CHECK(port.receive(&c, 1));
	CHECK(c == 'q');
	CHECK(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "DMA receive passes the user buffer to the device when complete") {
	RecordingDevice device(port);
	char buf[8] = {};
	CHECK(port.receive_DMA(buf, 8));
	CHECK(HostPlatform::uart().rxDmaArmedCount == 1);
	CHECK(HostPlatform::uart().lastRxSize == 8);
	CHECK(device.events == Events{"start rx"});
	CHECK_FALSE(rxFree());

	HostPlatform::uartReceive("D"); // Fake stores the byte in buf[0] and calls the rx complete callbacks
	CHECK(device.received == "D");
	CHECK(device.lastRcvAddress == buf); // Reference to the start of the user buffer
	CHECK(device.events == Events{"start rx", "end rx"});
	CHECK(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "interrupt receive passes the user buffer to the device when complete") {
	RecordingDevice device(port);
	char buf[2] = {};
	CHECK(port.receive_IT(buf, 2));
	CHECK(HostPlatform::uart().rxArmedCount == 1);
	CHECK(HostPlatform::uart().lastRxSize == 2);
	CHECK_FALSE(rxFree());

	HostPlatform::uartReceive("I");
	CHECK(device.received == "I");
	CHECK(device.lastRcvAddress == buf);
	CHECK(device.events == Events{"start rx", "end rx"});
	CHECK(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "DMA and interrupt receive report HAL errors") {
	HostPlatform::uart().rxArmResult = HAL_BUSY;
	char buf[2] = {};
	CHECK_FALSE(port.receive_DMA(buf, 2));
	CHECK_FALSE(port.receive_IT(buf, 2));
}

TEST_CASE_FIXTURE(UartPortFixture, "registerInterrupt receives single bytes into the port buffer") {
	RecordingDevice device(port);
	CHECK(port.registerInterrupt());
	CHECK(HostPlatform::uart().rxArmedCount == 1);
	CHECK(HostPlatform::uart().lastRxSize == UART_BUF_SIZE);

	HostPlatform::uartReceive("A");
	CHECK(device.received == "A");
	CHECK(device.events.empty()); // The port was not taken for single byte reception

	// Single shot: the device has to rearm
	HostPlatform::uartReceive("B");
	CHECK(device.received == "A");
	CHECK(HostPlatform::uart().rxLost == 1);
	CHECK(port.registerInterrupt());
	HostPlatform::uartReceive("C");
	CHECK(device.received == "AC");
}

TEST_CASE_FIXTURE(UartPortFixture, "after a user buffer receive the next completion uses the port buffer again") {
	RecordingDevice device(port);
	char buf[2] = {};
	port.receive_IT(buf, 1);
	HostPlatform::uartReceive("1");
	REQUIRE(device.lastRcvAddress == buf);

	port.registerInterrupt();
	HostPlatform::uartReceive("2");
	CHECK(device.received == "12");
	CHECK(device.lastRcvAddress != buf);
	CHECK(buf[0] == '1'); // User buffer not overwritten
}

TEST_CASE_FIXTURE(UartPortFixture, "registerInterrupt reports a HAL error") {
	HostPlatform::uart().rxArmResult = HAL_ERROR;
	CHECK_FALSE(port.registerInterrupt());
}

TEST_CASE_FIXTURE(UartPortFixture, "a receive complete of another uart is ignored") {
	RecordingDevice device(port);
	char buf[1] = {'z'};
	port.receive_IT(buf, 1);
	rxComplete(&otherHandle);
	CHECK(device.received.empty());
	CHECK(device.events == Events{"start rx"});
	CHECK_FALSE(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "a receive complete without a device is ignored") {
	CHECK(port.registerInterrupt());
	HostPlatform::uartReceive("x"); // Must not call a device
	CHECK(HostPlatform::uart().rxLost == 0);
	CHECK_FALSE(port.isTaken());
}

TEST_CASE_FIXTURE(UartPortFixture, "a failed interrupt receive releases the port" * KNOWN_ISSUE("UARTPort::receive_IT takes the receive side of the port before HAL_UART_Receive_IT. If the HAL call fails no rx complete interrupt follows and the port stays taken (UART.cpp:101-106)")) {
	RecordingDevice device(port);
	HostPlatform::uart().rxArmResult = HAL_BUSY;
	char buf[1] = {};
	CHECK_FALSE(port.receive_IT(buf, 1));
	CHECK_FALSE(port.isTaken());
	CHECK(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "a failed DMA receive releases the port" * KNOWN_ISSUE("UARTPort::receive_DMA takes the receive side of the port before HAL_UART_Receive_DMA. If the HAL call fails no rx complete interrupt follows and the port stays taken (UART.cpp:94-99)")) {
	RecordingDevice device(port);
	HostPlatform::uart().rxArmResult = HAL_ERROR;
	char buf[1] = {};
	CHECK_FALSE(port.receive_DMA(buf, 1));
	CHECK_FALSE(port.isTaken());
	CHECK(rxFree());
}

// NEEDS REVIEW: one isTaken flag is shared by the transmit and the receive semaphore. A receive completion
// (even of a single byte armed by registerInterrupt) clears it while a transmit is still running, so isTaken()
// and UART_CommandInterface::readyToSend() report a free port although the tx semaphore is still taken.
TEST_CASE_FIXTURE(UartPortFixture, "characterization: a receive completion clears the taken flag of a running transmit") {
	RecordingDevice device(port);
	CHECK(port.transmit_IT("long reply", 10));
	CHECK(port.isTaken());
	port.registerInterrupt();
	HostPlatform::uartReceive("r");

	CHECK_FALSE(port.isTaken());   // Flag cleared by the receive side
	CHECK_FALSE(txFree());		   // but the transmit is still running
	CHECK(device.events == Events{"start tx", "end rx"});
	txComplete(&handle);
	CHECK(txFree());
}

// ---------------------------------------------------------------------------
// Abort
// ---------------------------------------------------------------------------

TEST_CASE_FIXTURE(UartPortFixture, "abortReceive stops the receive and releases the receive side") {
	RecordingDevice device(port);
	char buf[4] = {};
	port.receive_IT(buf, 4);
	CHECK(port.abortReceive());
	CHECK(HostPlatform::uart().abortRxCount == 1);
	CHECK(device.events == Events{"start rx", "end rx"});
	CHECK(rxFree());

	HostPlatform::uartReceive("x"); // Nothing armed anymore
	CHECK(HostPlatform::uart().rxLost == 1);
	CHECK(device.received.empty());
}

TEST_CASE_FIXTURE(UartPortFixture, "a failed abortReceive keeps the receive side taken") {
	RecordingDevice device(port);
	char buf[4] = {};
	port.receive_DMA(buf, 4);
	HostPlatform::uart().abortResult = HAL_ERROR;
	CHECK_FALSE(port.abortReceive());
	CHECK(device.events == Events{"start rx"});
	CHECK_FALSE(rxFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "abortTransmit stops the transmit and releases the transmit side") {
	RecordingDevice device(port);
	port.transmit_DMA("abc", 3);
	CHECK(port.abortTransmit());
	CHECK(HostPlatform::uart().abortTxCount == 1);
	CHECK(device.events == Events{"start tx", "end tx"});
	CHECK_FALSE(port.isTaken());
	CHECK(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "a failed abortTransmit keeps the transmit side taken") {
	RecordingDevice device(port);
	port.transmit_IT("abc", 3);
	HostPlatform::uart().abortResult = HAL_BUSY;
	CHECK_FALSE(port.abortTransmit());
	CHECK(device.events == Events{"start tx"});
	CHECK_FALSE(txFree());
}

TEST_CASE_FIXTURE(UartPortFixture, "abort without a device only aborts the HAL transfer") {
	CHECK(port.abortReceive());
	CHECK(port.abortTransmit());
	CHECK(HostPlatform::uart().abortRxCount == 1);
	CHECK(HostPlatform::uart().abortTxCount == 1);
	HostPlatform::uart().abortResult = HAL_ERROR;
	CHECK_FALSE(port.abortReceive());
	CHECK_FALSE(port.abortTransmit());
}

} // TEST_SUITE

// ---------------------------------------------------------------------------
TEST_SUITE("UartHandler") {

TEST_CASE("a uart handler is registered while it exists") {
	UartHandler *address = nullptr;
	{
		RecordingUartHandler handler;
		address = &handler;
		CHECK(isRegistered(&handler));
	}
	auto &handlers = UartHandler::getUARTHandlers();
	CHECK(std::find(handlers.begin(), handlers.end(), address) == handlers.end());
}

TEST_CASE("every registered handler receives rx and tx completions with the uart handle") {
	HostPlatform::reset();
	UART_HandleTypeDef handle{};
	RecordingUartHandler a;
	RecordingUartHandler b;
	rxComplete(&handle);
	txComplete(&handle);
	CHECK(a.rx == std::vector<UART_HandleTypeDef *>{&handle});
	CHECK(a.tx == std::vector<UART_HandleTypeDef *>{&handle});
	CHECK(b.rx == std::vector<UART_HandleTypeDef *>{&handle});
	CHECK(b.tx == std::vector<UART_HandleTypeDef *>{&handle});
}

TEST_CASE("uart handlers are called in the order they were created") {
	UART_HandleTypeDef handle{};
	std::vector<std::string> log;
	RecordingUartHandler first(&log, "first");
	RecordingUartHandler second(&log, "second");
	rxComplete(&handle);
	CHECK(log == std::vector<std::string>{"first rx", "second rx"});
}

TEST_CASE("a destroyed uart handler receives no more completions") {
	UART_HandleTypeDef handle{};
	std::vector<std::string> log;
	RecordingUartHandler stays(&log, "stays");
	{
		RecordingUartHandler gone(&log, "gone");
	}
	txComplete(&handle);
	CHECK(log == std::vector<std::string>{"stays tx"});
}

TEST_CASE("the default callbacks of the uart handler base class ignore completions") {
	UART_HandleTypeDef handle{};
	std::vector<std::string> log;
	UartHandler plain; // Registered, does not override the callbacks
	RecordingUartHandler after(&log, "after");
	CHECK(isRegistered(&plain));
	rxComplete(&handle);
	txComplete(&handle);
	CHECK(log == std::vector<std::string>{"after rx", "after tx"}); // Dispatch continues behind the base handler
}

} // TEST_SUITE
