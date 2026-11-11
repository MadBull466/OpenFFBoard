/*
 * HostUsb.h
 *
 * Test control interface of the fake TinyUSB device API (platform/src/host_usb.cpp).
 * Everything the firmware sends via CDC or HID is recorded here.
 */
#ifndef HOSTTEST_HOSTUSB_H_
#define HOSTTEST_HOSTUSB_H_

#include <cstdint>
#include <string>
#include <vector>

namespace HostUsb {

struct HidReport {
	uint8_t itf = 0;
	uint8_t reportId = 0; //!< Report id passed to tinyusb. 0 = id is contained in the data
	std::vector<uint8_t> data;
};

struct State {
	bool mounted = true;				//!< tud_ready() result
	bool cdcConnected = true;			//!< tud_cdc_connected() result (DTR)
	uint32_t cdcWriteAvailable = 1024;	//!< Free space in the cdc tx fifo. Limits how much a single write accepts
	bool hidReady = true;				//!< tud_hid_ready() result
	bool hidReportResult = true;		//!< Return value of tud_hid_report()

	std::string cdcTx;					//!< All data written to cdc interface 0
	uint32_t cdcWrites = 0;				//!< Amount of tud_cdc_n_write calls
	uint32_t cdcFlushes = 0;			//!< Amount of tud_cdc_n_write_flush calls
	std::vector<HidReport> hidReports;	//!< All HID IN reports sent

	std::string cdcRx;					//!< Received CDC bytes not read yet. Served by tud_cdc_n_available / tud_cdc_n_read
};

State &state();

/** Restores defaults and clears all recorded data */
void reset();

/**
 * Queues bytes received on CDC interface 0 (host -> device). The firmware reads them with tud_cdc_n_read.
 * Does not call tud_cdc_rx_cb: the test calls the receiver (e.g. FFBoardMain::cdcRcvReady(0)) like
 * global_callbacks.cpp does.
 */
void cdcReceive(const std::string &data);

} // namespace HostUsb

#endif
