// P12 T5 lead probe of the CAN model through the real HAL (kept as the model's own sanity cases; the writers' files cover the HAL).
#include "CanTestSupport.h"

#include <cstdint>

namespace {
struct Probe : CanFixture {
	CAN_HandleTypeDef h{};
	Probe() {
		h.Instance = CAN1;
		h.Init.Prescaler = 6;
		h.Init.Mode = CAN_MODE_LOOPBACK;
		h.Init.SyncJumpWidth = CAN_SJW_1TQ;
		h.Init.TimeSeg1 = CAN_BS1_11TQ;
		h.Init.TimeSeg2 = CAN_BS2_2TQ;
	}
	void openFilter() {
		CAN_FilterTypeDef f{};
		f.FilterBank = 0;
		f.FilterMode = CAN_FILTERMODE_IDMASK;
		f.FilterScale = CAN_FILTERSCALE_32BIT;
		f.FilterFIFOAssignment = CAN_RX_FIFO0;
		f.FilterActivation = CAN_FILTER_ENABLE;
		f.SlaveStartFilterBank = 14;
		REQUIRE(HAL_CAN_ConfigFilter(&h, &f) == HAL_OK);
	}
};
}

TEST_CASE_FIXTURE(Probe, "probe: init, filter, start, loopback transmit and receive") {
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_READY);
	openFilter();
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_LISTENING);
	CAN_TxHeaderTypeDef th{};
	th.StdId = 0x123;
	th.IDE = CAN_ID_STD;
	th.RTR = CAN_RTR_DATA;
	th.DLC = 3;
	uint8_t d[8] = {1, 2, 3};
	uint32_t mb = 99;
	REQUIRE(HAL_CAN_AddTxMessage(&h, &th, d, &mb) == HAL_OK);
	CHECK(mb == CAN_TX_MAILBOX0);
	REQUIRE(hostCanTxCount(0) == 1u);
	CHECK(hostCanTxAt(0, 0).id == 0x123u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t r[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, r) == HAL_OK);
	CHECK(rh.StdId == 0x123u);
	CHECK(rh.DLC == 3u);
	CHECK(r[2] == 3);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Probe, "probe: external frame through a list filter and fifo overrun") {
	h.Init.Mode = CAN_MODE_NORMAL;
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	openFilter();
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	HostCanFrame f{};
	f.id = 0x55;
	f.dlc = 1;
	f.data[0] = 9;
	for (int k = 0; k < 4; ++k) CHECK(hostCanRx(0, &f) != 0);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 3u);
	CHECK((CAN1->RF0R & (1u << 4)) != 0u);
	CHECK(HalFixture::faults() == 0u);
}
