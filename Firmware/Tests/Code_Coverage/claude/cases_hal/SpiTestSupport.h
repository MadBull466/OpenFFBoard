/*
 * SpiTestSupport.h (P12 T4a): the strong HAL_SPI_* callbacks of the SPI test files, defined once in test_hal_spi_support.cpp.
 * They count their calls and remember the handle of the last call. The test files include this header and call spiSupportReset()
 * at the start of each case (or use SpiFixture below).
 */
#ifndef FFB_TESTS_SPI_TEST_SUPPORT_H
#define FFB_TESTS_SPI_TEST_SUPPORT_H

#include "HalFixture.h"
#include "HalSpi.h"

struct SpiCallbackCounts {
	unsigned mspInit, mspDeInit;
	unsigned txCplt, rxCplt, txRxCplt;
	unsigned txHalf, rxHalf, txRxHalf;
	unsigned error, abortCplt;
	SPI_HandleTypeDef *last;      /* handle of the most recent callback */
	uint32_t errorCodeAtError;    /* hspi->ErrorCode seen by the most recent ErrorCallback */
	HAL_SPI_StateTypeDef stateAtLast; /* hspi->State seen by the most recent callback */
};

extern SpiCallbackCounts spiCounts;
void spiSupportReset(void);

class SpiFixture : public HalFixture {
public:
	SpiFixture() { spiSupportReset(); }
};

#endif /* FFB_TESTS_SPI_TEST_SUPPORT_H */
