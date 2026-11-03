/*
 * FakeTmc4671.h
 *
 * Fake TMC4671-LA servo controller chip on the fake SPI1 bus (ffb_driver_tests only). Peer of the TMC4671 driver
 * (Firmware/FFBoard/UserExtensions/Src/TMC4671.cpp). Requirements: Firmware/Tests/driver/TMC4671_REQUIREMENTS.md,
 * section "Fake peer".
 *
 * Source: TMC4671-LA datasheet, IC version V1.3, document revision 2.07 (2022-01-12), cited "DS §x pNN".
 *
 * MODELS
 *  - Datagram (DS §4.2.1 p17): 40 bit, MSB first, bit 39 = WRnRD (1 = write), bits 38..32 = address, bits 31..0 = data.
 *    The chip answers inside the same datagram (bytes 1..4 of the MISO frame) for reads and writes. Reads answer with
 *    the content of the addressed register at the time of the frame. For a write frame the answer is the content
 *    BEFORE the write (assumption: DS only says the chip "immediately responds" on read and write; the driver does
 *    not use the answer of a write). A transfer is decoded from the MOSI bytes whatever the HAL call (Transmit,
 *    TransmitReceive, polled / IT / DMA).
 *  - 128 x 32 bit register file with the power-on defaults of DS §7.3 p111-127 (see defaultValue() in the .cpp for
 *    every non-zero default and its page). Fields whose default is 0 and registers without a default row are 0.
 *    Where the DS gives a default per field, the register default is the fields composed at their bit positions of
 *    DS §7.2 p77-110.
 *  - Read-only registers (the "R" entries of DS §7.1 p73-75): writes are ignored and recorded as a violation.
 *  - Stacked registers (DS §7.1 p73): CHIPINFO_DATA 0x00 selected by CHIPINFO_ADDR 0x01 (SI_TYPE 0x34363731 = "4671",
 *    SI_VERSION default 0x00010003 = "V1.3" of the DS title, other words 0), ADC_RAW_DATA 0x02 selected by 0x03
 *    (4 words, settable), CONFIG_DATA 0x4D selected by CONFIG_ADDR 0x4E (sparse store addressed by the full 32 bit
 *    CONFIG_ADDR; reads and writes of 0x4D hit the selected word), PID_ERROR_DATA 0x6C / PID_ERROR_ADDR 0x6D and
 *    INTERIM_DATA 0x6E / INTERIM_ADDR 0x6F (settable words per selector).
 *  - STATUS_FLAGS 0x7C: the test raises bits (raiseFlags); a write clears the flags whose bit is 0 in the written
 *    value (DS 0x7C p107: "The register is also used to reset status flags"; a write of 0 clears all. The semantic
 *    for non-zero values is an ASSUMPTION).
 *  - PID_POSITION_ACTUAL 0x6B: a write also writes PID_POSITION_TARGET 0x68 (DS 0x6B p98).
 *  - TMC4671_INPUTS_RAW 0x76: ENABLE_IN_RAW (bit 15) default high (assumption: the DS default 0 describes the register
 *    reset, the pin is driven high on the board), N of ABN (bit 2) low. Settable with setEnableIn / setNPulse.
 *  - Addresses that Rev 2.07 does not document (not in DS §7.1; 0x5F is documented in 7.3 only and counts as
 *    documented): accesses are stored (write) / answered from the stored value (read) and recorded in
 *    undocumentedAccesses(). They are not violations.
 *  - Strictness (violations()): frame size != 5 bytes (answer zeros, frame ignored), write to a read-only register,
 *    transfer with none or several CS lines low at its start (recorded on the fake with the lowest CS index), CS of
 *    this chip not low (not reachable: the bus routes by the low CS). Tests that provoke a violation on purpose
 *    declare it with expectViolation(); TmcFixture fails the test at teardown on any other violation.
 *  - Two chips share SPI1: the first constructed fake installs one responder that routes every frame by
 *    SpiTransfer::selectedCs() (index 0 = PA4 = TMC_1, index 1 = PB0 = TMC_2). A frame for a CS without a fake is
 *    answered with zeros and not recorded.
 *  - Optional simulated time per frame (setMicrosPerTransfer, default 0). The fake SPI takes no time, so busy loops
 *    on HAL_GetTick in the driver (calibrateAdcOffset, pidAutoTune) only end if frames advance time. A real
 *    frame is 40 bit at 5.25 MHz = 7.6 us. Modelling choice.
 *  - Test hooks: onRead(addr, fn) runs before the answer is computed, onWrite(addr, fn) after the write was applied.
 *    They can script a plant (e.g. let an encoder follow the commanded angle).
 *
 * DOES NOT MODEL
 *  FOC control loop, PWM outputs, ADC conversion, encoder decoding (ABN, hall, AENC), PLL, watchdog, UART / RTMI,
 *  step / direction, ENI / ENO pin effects, SPI timing (clock, the 500 ns pause after the address byte, DS Table 2
 *  p18-19: only checked by the tests through the recorded SPI configuration), CS release inside a frame (the CS state
 *  is checked at the start of the frame only), register value ranges (min / max of DS §7.3 are not enforced),
 *  side effects of writes other than those listed above, bits of 0x7C other than "raised by the test".
 *
 * CONFIDENCE
 *  frame decode and register file: High. Stacked registers: High. Status flags: Medium. Defaults: High where DS §7.3
 *  gives them, otherwise 0 (documented per register in the .cpp). Anything dynamic (hooks, plant): Low.
 *
 * ASSUMPTIONS (summary): answer of a write frame, status flag write semantic for non-zero values, ENABLE_IN default,
 * ADC_RAW words default 0 (DS §7.3), out-of-range selectors (CHIPINFO_ADDR > 5 answers 0; ADC_RAW_ADDR uses bits 1..0,
 * the DS maximum is 3).
 */
#ifndef HOSTTEST_FAKETMC4671_H_
#define HOSTTEST_FAKETMC4671_H_

#include "HostTarget.h"

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

class FakeTmc4671 {
public:
	/** One decoded 40 bit frame */
	struct Frame {
		bool write = false;
		uint8_t addr = 0;
		uint32_t data = 0;	 //!< Data field of the MOSI frame (0 for reads)
		uint32_t answer = 0; //!< Data field of the MISO frame
		uint64_t timeUs = 0; //!< HostPlatform::nowUs() at the start of the frame
		int cs = 0;			 //!< CS index (0 = TMC_1, 1 = TMC_2)
	};

	enum class ViolationKind : uint8_t {
		WrongFrameSize,	  //!< Transfer is not 5 bytes (DS §4.2.1 p17)
		WriteToReadOnly,  //!< Write to a register that is "R" in DS §7.1
		NoSingleCsLow,	  //!< None or several CS lines low when the frame started
	};
	struct Violation {
		ViolationKind kind = ViolationKind::WrongFrameSize;
		int addr = -1; //!< Register address, -1 if not applicable (frame size, CS)
		uint64_t timeUs = 0;
		std::string text() const;
	};

	struct UndocumentedAccess {
		uint8_t addr = 0;
		bool write = false;
		uint32_t data = 0;
		uint64_t timeUs = 0;
	};

	using ReadHook = std::function<void(FakeTmc4671 &)>;
	using WriteHook = std::function<void(FakeTmc4671 &, uint32_t data)>;

	/** Creates the chip on CS index `csIndex` (0 = PA4 = TMC_1, 1 = PB0 = TMC_2) and (re)installs the bus responder. */
	explicit FakeTmc4671(int csIndex = 0);
	~FakeTmc4671();
	FakeTmc4671(const FakeTmc4671 &) = delete;
	FakeTmc4671 &operator=(const FakeTmc4671 &) = delete;

	/** Installs the responder on hspi1 again (the target reset removes it). Done by the constructor. */
	static void installResponder();

	int cs() const { return csIndex; }

	/** Power-on state: defaults, empty logs, no hooks, no expected violations, 0 us per transfer. Chip stays present */
	void reset();
	/** false: the chip does not answer (all answers 0, frames still logged), like an unpopulated board */
	void setPresent(bool present) { present_ = present; }
	bool present() const { return present_; }

	// --- register access from the test (no strictness, no log) ---
	/** What a read of `addr` would answer now (stacked data registers: the selected word). No hooks, no log */
	uint32_t reg(uint8_t addr) const;
	/** Sets the backing store of `addr`, also read-only registers. Stacked data registers: sets the selected word */
	void setReg(uint8_t addr, uint32_t value);
	/** CONFIG_DATA word of CONFIG_ADDR n (0 if never written) */
	uint32_t configWord(uint32_t n) const;
	void setConfigWord(uint32_t n, uint32_t value);
	/** All writes to CONFIG_DATA in order: {CONFIG_ADDR at that time, data} */
	const std::vector<std::pair<uint32_t, uint32_t>> &configWrites() const { return configWrites_; }

	/** ADC_RAW_DATA word for ADC_RAW_ADDR sel (0..3), DS §7.1 p73 */
	void setAdcRaw(uint8_t sel, uint32_t word);
	/** CHIPINFO_DATA word for CHIPINFO_ADDR sel (0..5) */
	void setChipInfo(uint8_t sel, uint32_t word);
	void setPidError(uint8_t sel, uint32_t word);
	void setInterim(uint8_t sel, uint32_t word);
	/** STATUS_FLAGS |= mask */
	void raiseFlags(uint32_t mask);
	/** ENABLE_IN_RAW, bit 15 of 0x76 */
	void setEnableIn(bool high);
	/** N of ABN_RAW, bit 2 of 0x76 */
	void setNPulse(bool high);

	// --- hooks ---
	void onRead(uint8_t addr, ReadHook fn);
	void onWrite(uint8_t addr, WriteHook fn);
	void clearHooks();

	// --- time ---
	/** Simulated time added per frame (HostPlatform::advanceUs) */
	void setMicrosPerTransfer(uint32_t us) { microsPerTransfer_ = us; }
	uint32_t microsPerTransfer() const { return microsPerTransfer_; }

	// --- records ---
	const std::vector<Frame> &log() const { return log_; }
	void clearLog() { log_.clear(); }
	/** Data of all write frames to `addr`, in order */
	std::vector<uint32_t> writesTo(uint8_t addr) const;
	/** Number of read frames of `addr` */
	size_t readsOf(uint8_t addr) const;
	const std::vector<Violation> &violations() const { return violations_; }
	const std::vector<UndocumentedAccess> &undocumentedAccesses() const { return undocumented_; }
	/** Declares a violation as expected (addr -1: any address). Expected violations are not reported by unexpectedViolations() */
	void expectViolation(ViolationKind kind, int addr = -1);
	std::vector<Violation> unexpectedViolations() const;

	/** DS §7.1: true for the addresses with access "R" */
	static bool isReadOnly(uint8_t addr);
	/** true for the addresses of DS §7.1 plus 0x5F (DS §7.3) */
	static bool isDocumented(uint8_t addr);
	/** Power-on default of DS §7.3 */
	static uint32_t defaultValue(uint8_t addr);

	static constexpr uint32_t SI_TYPE_4671 = 0x34363731; //!< ASCII "4671", DS §4.2.1 p17 example
	static constexpr uint32_t SI_VERSION_V13 = 0x00010003;

private:
	struct Expected {
		ViolationKind kind;
		int addr;
	};

	static std::vector<uint8_t> dispatch(const HostTarget::SpiTransfer &t);
	std::vector<uint8_t> handle(const HostTarget::SpiTransfer &t, bool singleCs);
	uint32_t readValue(uint8_t addr) const;
	void writeValue(uint8_t addr, uint32_t data);
	void violate(ViolationKind kind, int addr, uint64_t timeUs);

	int csIndex;
	bool present_ = true;
	uint32_t microsPerTransfer_ = 0;
	std::array<uint32_t, 128> regs_{};
	std::array<uint32_t, 6> chipInfo_{};
	std::array<uint32_t, 4> adcRaw_{};
	std::array<uint32_t, 256> pidError_{};
	std::array<uint32_t, 256> interim_{};
	std::map<uint32_t, uint32_t> config_;
	std::vector<std::pair<uint32_t, uint32_t>> configWrites_;
	std::map<uint8_t, ReadHook> readHooks_;
	std::map<uint8_t, WriteHook> writeHooks_;
	std::vector<Frame> log_;
	std::vector<Violation> violations_;
	std::vector<UndocumentedAccess> undocumented_;
	std::vector<Expected> expected_;
};

#endif
