// P12 T8: the strong PCD callbacks shared by the USB device test files (see OtgTestSupport.h).
#include "OtgTestSupport.h"

#include <cstring>

OtgCallbackCounts otgCounts;
PCD_HandleTypeDef *otgLastHandle = nullptr;

void otgSupportReset(void) {
	std::memset(&otgCounts, 0, sizeof otgCounts);
	otgLastHandle = nullptr;
}

static void note(PCD_HandleTypeDef *hpcd, unsigned kind, unsigned ep) {
	otgLastHandle = hpcd;
	otgCounts.lastEp = ep;
	otgCounts.statusAtCallback[0] = hostRegsPeek(HOST_OTG_BASE + 0x14u);
	otgCounts.statusAtCallback[1] = hostRegsPeek(HOST_OTG_BASE + 0x818u);
	if (otgCounts.orderCount < 64u) {
		otgCounts.order[otgCounts.orderCount] = kind;
		otgCounts.epOrder[otgCounts.orderCount++] = ep;
	}
}

extern "C" {
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
	otgCounts.mspInit++;
	note(hpcd, OTG_CB_MSP_INIT, 0xFF);
}
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
	otgCounts.mspDeInit++;
	note(hpcd, OTG_CB_MSP_DEINIT, 0xFF);
}
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
	otgCounts.sof++;
	note(hpcd, OTG_CB_SOF, 0xFF);
}
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
	otgCounts.setup++;
	otgCounts.setupWords[0] = hpcd->Setup[0];
	otgCounts.setupWords[1] = hpcd->Setup[1];
	note(hpcd, OTG_CB_SETUP, 0xFF);
}
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
	otgCounts.reset++;
	note(hpcd, OTG_CB_RESET, 0xFF);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
	otgCounts.suspend++;
	note(hpcd, OTG_CB_SUSPEND, 0xFF);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
	otgCounts.resume++;
	note(hpcd, OTG_CB_RESUME, 0xFF);
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
	otgCounts.connect++;
	note(hpcd, OTG_CB_CONNECT, 0xFF);
}
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
	otgCounts.disconnect++;
	note(hpcd, OTG_CB_DISCONNECT, 0xFF);
}
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	otgCounts.dataOut++;
	note(hpcd, OTG_CB_DATA_OUT, epnum);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	otgCounts.dataIn++;
	note(hpcd, OTG_CB_DATA_IN, epnum);
}
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	otgCounts.isoOut++;
	note(hpcd, OTG_CB_ISO_OUT, epnum);
}
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	otgCounts.isoIn++;
	note(hpcd, OTG_CB_ISO_IN, epnum);
}
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg) {
	otgCounts.lpm++;
	otgCounts.lastMsg = (unsigned)msg;
	note(hpcd, OTG_CB_LPM, 0xFF);
}
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg) {
	otgCounts.bcd++;
	otgCounts.lastMsg = (unsigned)msg;
	note(hpcd, OTG_CB_BCD, 0xFF);
}
}

OtgFixture::OtgFixture() {
	otgSupportReset();
	std::memset(&h, 0, sizeof h);
	setFirmwareInit();
}

void OtgFixture::setFirmwareInit() {
	h.Instance = USB_OTG_FS;
	h.Init.dev_endpoints = 4;
	h.Init.speed = PCD_SPEED_FULL;
	h.Init.dma_enable = DISABLE;
	h.Init.phy_itface = PCD_PHY_EMBEDDED;
	h.Init.Sof_enable = DISABLE;
	h.Init.low_power_enable = DISABLE;
	h.Init.lpm_enable = DISABLE;
	h.Init.vbus_sensing_enable = ENABLE;
	h.Init.use_dedicated_ep1 = DISABLE;
}

HAL_StatusTypeDef OtgFixture::init() { return HAL_PCD_Init(&h); }

unsigned OtgFixture::drain(unsigned maxCalls) {
	unsigned n = 0;
	while (n < maxCalls && hostOtgIrqPending()) {
		HAL_PCD_IRQHandler(&h);
		++n;
	}
	return n;
}

void OtgFixture::bringUp() {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x40) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 0, 0x20) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 1, 0x40) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 2, 0x40) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 3, 0x40) == HAL_OK);
	REQUIRE(HAL_PCD_Start(&h) == HAL_OK);
}
