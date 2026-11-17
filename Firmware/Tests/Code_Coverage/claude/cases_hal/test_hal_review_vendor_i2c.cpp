// V129 (vendor, ST HAL I2C): I2C_ITError dereferences the Tx DMA handle although only an Rx DMA handle is configured.
// Found by source reading in the Codex review of P12 (P12C-T4C-003). The reproduction below crashes the process (null pointer read), so the case
// is skipped like the other crashing known issues; run it on its own to see the crash.
#include "I2cTestSupport.h"
#include "HalDma.h"
#include "TestHelpers.h"

extern "C" uint32_t SystemCoreClock;

TEST_CASE("RV21 aborting a receive only DMA master transfer does not read through a missing Tx DMA handle" *
		  doctest::description("KNOWN ISSUE (crash, skipped because it terminates the run): V129: I2C_ITError (stm32f4xx_hal_i2c.c:6443-6447) tests DMAEN and then reads hi2c->hdmatx->State unconditionally; HAL_I2C_Master_Receive_DMA itself only uses hdmarx, so with hdmatx == NULL the abort of a running receive reads through a null pointer; source-level finding, the crash is the reproduction (the oracle is the HAL's own Receive_DMA, which needs no Tx handle)") *
		  doctest::skip()) {
	I2cFixture f;
	SystemCoreClock = 168000000u;
	RCC->CFGR = (5u << 10);
	uwTickPrio = 15;
	I2C_HandleTypeDef h{};
	h.Instance = I2C1;
	h.Init.ClockSpeed = 100000;
	h.Init.DutyCycle = I2C_DUTYCYCLE_2;
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	DMA_HandleTypeDef d{};
	d.Instance = DMA1_Stream0;
	d.Init.Channel = DMA_CHANNEL_1;
	d.Init.Direction = DMA_PERIPH_TO_MEMORY;
	d.Init.MemInc = DMA_MINC_ENABLE;
	d.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	d.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	d.Init.Mode = DMA_NORMAL;
	d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	REQUIRE(HAL_DMA_Init(&d) == HAL_OK);
	h.hdmarx = &d;
	d.Parent = &h;
	hostI2cDeviceAdd(0, 0x50, 0);
	uint8_t b[4] = {};
	REQUIRE(h.hdmatx == nullptr);
	REQUIRE(HAL_I2C_Master_Receive_DMA(&h, 0xA0, b, 4) == HAL_OK);
	i2cRunIrqs(&h, 0);
	REQUIRE(HAL_I2C_Master_Abort_IT(&h, 0xA0) == HAL_OK);  // reaches I2C_ITError with DMAEN set
	HAL_DMA_IRQHandler(&d);
	CHECK(i2cCounts.abortCplt == 1u);
}
