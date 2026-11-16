/*
 * OtgTestSupport.h (P12 T8): the strong HAL_PCD_* callbacks of the USB device test files, defined once in test_hal_pcd_support.cpp, and the OtgFixture.
 * The callbacks count their calls and keep the order. OtgFixture owns a PCD handle set up as the firmware sets it up (main.c MX_USB_OTG_FS_PCD_Init: USB_OTG_FS,
 * 4 endpoints, full speed, embedded PHY, no DMA, no SOF, no low power, no LPM, VBUS sensing on, no dedicated EP1) and helpers that run the interrupt handler.
 */
#ifndef FFB_TESTS_OTG_TEST_SUPPORT_H
#define FFB_TESTS_OTG_TEST_SUPPORT_H

#include "HalFixture.h"
#include "HalOtg.h"

enum {
	OTG_CB_MSP_INIT = 1, OTG_CB_MSP_DEINIT, OTG_CB_SOF, OTG_CB_SETUP, OTG_CB_RESET, OTG_CB_SUSPEND, OTG_CB_RESUME, OTG_CB_CONNECT, OTG_CB_DISCONNECT,
	OTG_CB_DATA_OUT, OTG_CB_DATA_IN, OTG_CB_ISO_OUT, OTG_CB_ISO_IN, OTG_CB_LPM, OTG_CB_BCD
};

struct OtgCallbackCounts {
	unsigned mspInit, mspDeInit, sof, setup, reset, suspend, resume, connect, disconnect, dataOut, dataIn, isoOut, isoIn, lpm, bcd;
	unsigned lastEp;
	unsigned lastMsg;
	uint32_t setupWords[2];        /* hpcd->Setup[0..1] when the last setup callback ran */
	uint32_t statusAtCallback[2];  /* GINTSTS and DAINT when the last callback ran */
	unsigned order[64];            /* callback kinds in call order (OTG_CB_*) */
	unsigned epOrder[64];          /* endpoint argument of each callback, 0xFF for those without */
	unsigned orderCount;
};

extern OtgCallbackCounts otgCounts;
extern PCD_HandleTypeDef *otgLastHandle;
void otgSupportReset(void);

class OtgFixture : public HalFixture {
public:
	PCD_HandleTypeDef h;
	OtgFixture();
	/* The init structure of the firmware (main.c). */
	void setFirmwareInit();
	/* HAL_PCD_Init with the firmware init. Returns the HAL status. */
	HAL_StatusTypeDef init();
	/* Runs HAL_PCD_IRQHandler until the model reports no pending interrupt (at most maxCalls). Returns the number of calls. */
	unsigned drain(unsigned maxCalls = 64);
	/* Standard bring-up of the firmware's USB glue: init, FIFO sizes (RX 0x40, TX0 0x20, TX1-3 0x40 each: 288 words of the 320 word RAM), start. */
	void bringUp();
};

#endif /* FFB_TESTS_OTG_TEST_SUPPORT_H */
