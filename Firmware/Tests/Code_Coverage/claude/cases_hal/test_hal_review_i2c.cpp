// P12 review witnesses, I2C (Codex review of P12, P12C-T4C-001 and P12C-T4C-002).
#include "I2cTestSupport.h"
#include "HalDma.h"

extern "C" uint32_t SystemCoreClock;

TEST_CASE("RV9 a second sequential DMA frame programs the memory address of its own buffer (P12C-T4C-001)") {
	// UM1725 HAL_I2C_Master_Seq_Transmit_DMA: each frame is its own DMA transfer from the caller's buffer. The tests moved the bytes directly and
	// never looked at the stream registers, so a continuation that kept the first buffer's address went unnoticed.
	I2cFixture f;
	SystemCoreClock = 168000000u;
	RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10);
	uwTickPrio = 15;
	I2C_HandleTypeDef h{};
	h.Instance = I2C1;
	h.Init.ClockSpeed = 100000;
	h.Init.DutyCycle = I2C_DUTYCYCLE_2;
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	h.Init.OwnAddress1 = 0x30;
	REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	DMA_HandleTypeDef d{};
	d.Instance = DMA1_Stream6;
	d.Init.Channel = DMA_CHANNEL_1;
	d.Init.Direction = DMA_MEMORY_TO_PERIPH;
	d.Init.MemInc = DMA_MINC_ENABLE;
	d.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	d.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	d.Init.Mode = DMA_NORMAL;
	d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
	h.hdmatx = &d;
	d.Parent = &h;
	hostI2cDeviceAdd(0, 0x50, 0);
	uint8_t a[] = {0x12, 0x34}, b[] = {0x56, 0x78, 0x9A};
	REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&h, 0xA0, a, 2, I2C_FIRST_FRAME) == HAL_OK);
	i2cRunIrqs(&h, 0);
	I2C1->DR = a[0];
	I2C1->DR = a[1];
	hostDmaComplete(0, 6);
	HAL_DMA_IRQHandler(&d);
	i2cRunIrqs(&h, 0);
	REQUIRE(i2cCounts.masterTx == 1);
	REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&h, 0xA0, b, 2, I2C_LAST_FRAME) == HAL_OK);
	CHECK(d.Instance->M0AR == static_cast<uint32_t>(reinterpret_cast<uintptr_t>(b)));
	CHECK(d.Instance->NDTR == 2u);
}

namespace {
struct TenBitFx : I2cFixture {
	I2C_HandleTypeDef h{};
	TenBitFx() {
		SystemCoreClock = 168000000;
		RCC->CFGR = (5u << 10);
		uwTickPrio = 15;
		h.Instance = I2C1;
		h.Init.ClockSpeed = 100000;
		h.Init.DutyCycle = I2C_DUTYCYCLE_2;
		h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
		REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	}
};
}  // namespace

TEST_CASE_FIXTURE(TenBitFx, "RV10 a second ten-bit device sharing the header with the first one is selected by its full address (P12C-T4C-002)") {
	// I2C ten-bit addressing (RM0090 27.3.3): every device whose two high address bits match the 11110xx header acknowledges it; the second byte
	// (the low eight bits) selects the one device. 0x2A4 and 0x2A5 share the header 11110 10.
	REQUIRE(hostI2cDeviceAdd(0, 0x2A4, 1) == 0);
	REQUIRE(hostI2cDeviceAdd(0, 0x2A5, 1) == 1);
	uint8_t b = 0x35;
	CHECK(HAL_I2C_Master_Transmit(&h, 0x2A5, &b, 1, 100) == HAL_OK);
	CHECK(hostI2cLogAt(0, 2).value == 0xA5);
	CHECK(hostI2cLogAt(0, 2).ack == 1);
}

TEST_CASE_FIXTURE(TenBitFx, "RV10 the first ten-bit device of a shared header is still addressed (control, P12C-T4C-002)") {
	REQUIRE(hostI2cDeviceAdd(0, 0x2A4, 1) == 0);
	REQUIRE(hostI2cDeviceAdd(0, 0x2A5, 1) == 1);
	uint8_t b = 0x35;
	CHECK(HAL_I2C_Master_Transmit(&h, 0x2A4, &b, 1, 100) == HAL_OK);
	CHECK(hostI2cLogAt(0, 2).ack == 1);
}
