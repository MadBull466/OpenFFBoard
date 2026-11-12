// P12 T4a: the strong SPI callbacks shared by the SPI test files (see SpiTestSupport.h).
#include "SpiTestSupport.h"

SpiCallbackCounts spiCounts;

void spiSupportReset(void) { spiCounts = SpiCallbackCounts{}; }

extern "C" {
static void note(SPI_HandleTypeDef *h) {
	spiCounts.last = h;
	spiCounts.stateAtLast = h->State;
}
void HAL_SPI_MspInit(SPI_HandleTypeDef *h) { spiCounts.mspInit++; note(h); }
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *h) { spiCounts.mspDeInit++; note(h); }
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h) { spiCounts.txCplt++; note(h); }
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *h) { spiCounts.rxCplt++; note(h); }
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h) { spiCounts.txRxCplt++; note(h); }
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *h) { spiCounts.txHalf++; note(h); }
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *h) { spiCounts.rxHalf++; note(h); }
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *h) { spiCounts.txRxHalf++; note(h); }
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h) {
	spiCounts.error++;
	spiCounts.errorCodeAtError = h->ErrorCode;
	note(h);
}
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *h) { spiCounts.abortCplt++; note(h); }
}
