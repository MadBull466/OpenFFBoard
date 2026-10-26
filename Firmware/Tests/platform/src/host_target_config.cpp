/*
 * host_target_config.cpp
 *
 * Host replacement of Targets/F407VG/Core/Src/cpp_target_config.cpp (target hardware configuration, not
 * tested) and of the peripheral handles defined by the CubeMX code in main.c, for ffb_target_tests.
 *
 * Same objects and parameters as the F407 file, with one difference: `canport` is a MockCANPort instead of
 * CANPort_2B (CANPort2B.cpp needs the fake HAL CAN F8, planned for P7). Like CANPort_2B its handle is &hcan1, so the
 * CAN interrupt callbacks of global_callbacks.cpp find it with CANPort::handleToPort. huart1, hcan1 and
 * external_uart are defined in host_platform.cpp.
 *
 * resetTargetConfig() also restores the global SPI ports and i2cport to their state after construction. A few of their
 * members are private without a public way to reset them; the reset reaches them through member pointers obtained by
 * explicit template instantiation (standard C++: access checking does not apply to the template arguments of an
 * explicit instantiation, [temp.spec.general]). Only this file uses them, only for the reset.
 */
#include "cpp_target_config.h"
#include "HostTarget.h"
#include "MockCANPort.h"
#include "I2CHandler.h"
#include "SpiHandler.h"
#include "EncoderBissC.h"
#include "EncoderLocal.h"
#include "EncoderSSI.h"
#include "MtEncoderSPI.h"

#include <algorithm>
#include <array>

// Peripheral handles (main.c). Instance values are only compared by the firmware. Initialised like the CubeMX code
// before the static initialisation of the firmware objects would run on the target (main() runs MX_SPIx_Init and
// MX_I2C1_Init before the scheduler starts; SPIDevice copies hspi->Init when it is constructed)
namespace {
/** MX_SPIx_Init (Targets/F407VG/Core/Src/main.c:598-608, 636-646, 674-684) followed by HAL_SPI_Init: State READY */
SPI_HandleTypeDef mxSpi(void *instance, uint32_t polarity, uint32_t phase, uint32_t prescaler) {
	SPI_HandleTypeDef h = {};
	h.Instance = instance;
	h.Init.Mode = SPI_MODE_MASTER;
	h.Init.Direction = SPI_DIRECTION_2LINES;
	h.Init.DataSize = SPI_DATASIZE_8BIT;
	h.Init.CLKPolarity = polarity;
	h.Init.CLKPhase = phase;
	h.Init.NSS = SPI_NSS_SOFT;
	h.Init.BaudRatePrescaler = prescaler;
	h.Init.FirstBit = SPI_FIRSTBIT_MSB;
	h.Init.TIMode = SPI_TIMODE_DISABLE;
	h.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	h.Init.CRCPolynomial = 10;
	h.State = HAL_SPI_STATE_READY;
	h.ErrorCode = HAL_SPI_ERROR_NONE;
	h.Lock = HAL_UNLOCKED;
	return h;
}
SPI_HandleTypeDef mxSpi1() { return mxSpi((void *)0x40013000, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_8); } // SPI1
SPI_HandleTypeDef mxSpi2() { return mxSpi((void *)0x40003800, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_32); } // SPI2
SPI_HandleTypeDef mxSpi3() { return mxSpi((void *)0x40003C00, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_4); }	 // SPI3
/** MX_I2C1_Init (main.c:524-548) followed by HAL_I2C_Init: State READY, Mode NONE */
I2C_HandleTypeDef mxI2c1() {
	I2C_HandleTypeDef h = {};
	h.Instance = (void *)0x40005400; // I2C1
	h.Init.ClockSpeed = 100000;
	h.Init.DutyCycle = I2C_DUTYCYCLE_2;
	h.Init.OwnAddress1 = 0;
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	h.Init.OwnAddress2 = 0;
	h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	h.State = HAL_I2C_STATE_READY;
	h.ErrorCode = HAL_I2C_ERROR_NONE;
	h.Lock = HAL_UNLOCKED;
	h.Mode = HAL_I2C_MODE_NONE;
	return h;
}
} // namespace
SPI_HandleTypeDef hspi1 = mxSpi1();
SPI_HandleTypeDef hspi2 = mxSpi2();
SPI_HandleTypeDef hspi3 = mxSpi3();
I2C_HandleTypeDef hi2c1 = mxI2c1();
UART_HandleTypeDef huart3 = {};
ADC_HandleTypeDef hadc1 = {}; // ADC1 (MX_ADC1_Init, see resetTargetConfig)
ADC_HandleTypeDef hadc2 = {}; // ADC2 (MX_ADC2_Init)
ADC_TypeDef hosttest_adc1 = {}, hosttest_adc2 = {}; // Registers of ADC1 / ADC2

namespace {
#define HOSTTEST_ADC_CR1_SCAN 0x00000100U // stm32f407xx.h ADC_CR1_SCAN
/**
 * MX_ADCx_Init of Targets/F407VG/Core/Src/main.c followed by HAL_ADC_Init / ADC_Init (stm32f4xx_hal_adc.c): the Init
 * fields of main.c, the registers ADC_Init derives from them (CR1 SCAN/RES, CR2 EXTSEL/EXTEN, CONT, DDS, EOCS,
 * SQR1 L) and State READY. The ADC stays disabled (ADON 0) until HAL_ADC_Start(_DMA).
 */
void initAdc(ADC_HandleTypeDef &hadc, ADC_TypeDef &regs, uint32_t conversions, uint32_t triggerEdge) {
	regs = ADC_TypeDef();
	hadc = ADC_HandleTypeDef();
	hadc.Instance = &regs;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.ScanConvMode = ENABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = triggerEdge;
	hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.NbrOfConversion = conversions;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	regs.CR1 = HOSTTEST_ADC_CR1_SCAN | hadc.Init.Resolution;
	regs.CR2 = hadc.Init.DataAlign | hadc.Init.ExternalTrigConv | hadc.Init.ExternalTrigConvEdge | ADC_CR2_DDS | ADC_CR2_EOCS; // CONT 0
	regs.SQR1 = (conversions - 1U) << ADC_SQR1_L_Pos;
	hadc.Lock = HAL_UNLOCKED;
	hadc.ErrorCode = HAL_ADC_ERROR_NONE;
	hadc.State = HAL_ADC_STATE_READY;
}
} // namespace

// ---- From cpp_target_config.cpp (F407VG) ----
static const std::vector<OutputPin> external_spi_cspins{OutputPin(*SPI2_NSS_GPIO_Port, SPI2_NSS_Pin), OutputPin(*SPI2_SS2_GPIO_Port, SPI2_SS2_Pin), OutputPin(*SPI2_SS3_GPIO_Port, SPI2_SS3_Pin)};
SPIPort external_spi{hspi2, external_spi_cspins, 42000000, true};

static const std::vector<OutputPin> motor_spi_cspins{OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin), OutputPin(*SPI1_SS2_GPIO_Port, SPI1_SS2_Pin), OutputPin(*SPI1_SS3_GPIO_Port, SPI1_SS3_Pin)};
SPIPort motor_spi{hspi1, motor_spi_cspins, 84000000, false};

static const std::vector<OutputPin> ext3_spi_cspins{OutputPin(*SPI3_SS1_GPIO_Port, SPI3_SS1_Pin), OutputPin(*SPI3_SS2_GPIO_Port, SPI3_SS2_Pin), OutputPin(*SPI3_SS3_GPIO_Port, SPI3_SS3_Pin)};
SPIPort ext3_spi{hspi3, ext3_spi_cspins, 42000000, true};

UARTPort motor_uart{UART_PORT_MOTOR};

const I2C_InitTypeDef i2cPreset_10000 = {100000, I2C_DUTYCYCLE_2, 0, I2C_ADDRESSINGMODE_7BIT, I2C_DUALADDRESS_DISABLE, 0, I2C_GENERALCALL_DISABLE, I2C_NOSTRETCH_DISABLE};
const I2C_InitTypeDef i2cPreset_40000 = {400000, I2C_DUTYCYCLE_2, 0, I2C_ADDRESSINGMODE_7BIT, I2C_DUALADDRESS_DISABLE, 0, I2C_GENERALCALL_DISABLE, I2C_NOSTRETCH_DISABLE};
const auto i2cpresets = std::to_array<I2CPortHardwareConfig::I2CPortHardwareConfig_preset>({{i2cPreset_10000, "100kb/s"}, {i2cPreset_40000, "400kb/s"}});
const I2CPortHardwareConfig i2cHwConf = I2CPortHardwareConfig(true, i2cpresets);
I2CPort i2cport{I2C_PORT, i2cHwConf};

// CANPort_2B in the F407 configuration (see file comment)
namespace {
/** MockCANPort with the handle of CANPort_2B (CANPort2B.h: getHandle() returns its CAN_HandleTypeDef, here &hcan1) */
class HostCanPort : public MockCANPort {
protected:
	void *getHandle() override { return &hcan1; }
};
HostCanPort hostCanPort;
} // namespace
CANPort &canport = hostCanPort;

const OutputPin debugpin = OutputPin(*GP1_GPIO_Port, GP1_Pin);

// ---- Reset support (see the file comment) ----
namespace {
/** Explicit instantiation of this template with &Class::member makes the member pointer available through privateMember(Tag) */
template <class Tag, typename Tag::type Member>
struct PrivateMember {
	friend typename Tag::type privateMember(Tag) { return Member; }
};
struct SpiPortCurrentDevice {
	using type = SPIDevice *SPIPort::*;
	friend type privateMember(SpiPortCurrentDevice);
};
struct I2CPortCurrentDevice {
	using type = I2CDevice *I2CPort::*;
	friend type privateMember(I2CPortCurrentDevice);
};
struct I2CPortUsers {
	using type = int32_t I2CPort::*;
	friend type privateMember(I2CPortUsers);
};
struct EncoderSSIInUse {
	using type = bool *;
	friend type privateMember(EncoderSSIInUse);
};
struct EncoderBissCInUse {
	using type = bool *;
	friend type privateMember(EncoderBissCInUse);
};
// Explicit instantiations in the namespace of the template (here the unnamed namespace of this file)
template struct PrivateMember<SpiPortCurrentDevice, &SPIPort::current_device>;
template struct PrivateMember<EncoderSSIInUse, &EncoderSSI::inUse>;
template struct PrivateMember<EncoderBissCInUse, &EncoderBissC::inUse>;
template struct PrivateMember<I2CPortCurrentDevice, &I2CPort::currentDevice>;
template struct PrivateMember<I2CPortUsers, &I2CPort::portUsers>;

/** Access to the protected PersistentStorage::restoreDelayedFlag (pointer to member formed through a derived class) */
struct PersistentStorageAccess : PersistentStorage {
	static bool PersistentStorage::*restoreDelayedFlagMember() { return &PersistentStorageAccess::restoreDelayedFlag; }
};

/** SPI port as constructed in cpp_target_config.cpp: all CS pins free, no current device, not exclusive, semaphore free */
void resetSpiPort(SPIPort &port, const std::vector<OutputPin> &csPins) {
	port.*privateMember(SpiPortCurrentDevice()) = nullptr;
	port.getCsPins() = csPins;
	port.getFreeCsPins() = csPins;
	port.takeExclusive(false);
	port.giveSemaphore(); // clears isTakenFlag; the binary semaphore is available afterwards (no effect if it already was)
}

/** CS pins of a port for the fake HAL SPI (recorded levels around the transfers) */
std::vector<std::pair<GPIO_TypeDef *, uint16_t>> pinList(const std::vector<OutputPin> &pins) {
	std::vector<std::pair<GPIO_TypeDef *, uint16_t>> list;
	for (const OutputPin &pin : pins) {
		list.emplace_back(const_cast<GPIO_TypeDef *>(pin.getPort()), pin.getPin());
	}
	return list;
}

// Handlers registered during the static initialisation (the SPI ports and i2cport of this file; no other file defines
// static SPI or I2C handlers). Taken after the port objects above (same translation unit: initialised in order)
const std::vector<SpiHandler *> initialSpiHandlers = SpiHandler::getSPIHandlers();
const std::vector<I2CHandler *> initialI2cHandlers = I2CHandler::getI2CHandlers();
} // namespace

namespace HostTarget {
MockCANPort &canPort() { return hostCanPort; }

void resetTargetConfig() {
	// Handles: back to the state after MX_SPIx_Init / MX_I2C1_Init / MX_ADCx_Init (HAL_xxx_Init sets State READY)
	hspi1 = mxSpi1();
	hspi2 = mxSpi2();
	hspi3 = mxSpi3();
	hi2c1 = mxI2c1();
	resetBus();
	spiSetCsPins(&hspi1, pinList(motor_spi_cspins));
	spiSetCsPins(&hspi2, pinList(external_spi_cspins));
	spiSetCsPins(&hspi3, pinList(ext3_spi_cspins));
	huart3 = {};
	initAdc(hadc1, hosttest_adc1, ADC1_CHANNELS, ADC_EXTERNALTRIGCONVEDGE_RISING);		   // MX_ADC1_Init: 8 conversions
	initAdc(hadc2, hosttest_adc2, ADC2_CHANNELS, ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING); // MX_ADC2_Init: 2 conversions
	hcan1 = {CAN1, HAL_CAN_STATE_READY, HAL_CAN_ERROR_NONE}; // MX_CAN1_Init: HAL_CAN_Init sets State READY
	// Release the motor uart in case a test left a transfer unfinished (like external_uart in HostPlatform::reset)
	motor_uart.giveSemaphore(true);
	motor_uart.giveSemaphore(false);
	hostCanPort.resetForTest();
	// Global bus ports: state after construction
	resetSpiPort(external_spi, external_spi_cspins);
	resetSpiPort(motor_spi, motor_spi_cspins);
	resetSpiPort(ext3_spi, ext3_spi_cspins);
	i2cport.*privateMember(I2CPortCurrentDevice()) = nullptr;
	i2cport.*privateMember(I2CPortUsers()) = 0;
	i2cport.setSpeedPreset(0); // config = preset 0; no HAL call without port users
	i2cport.giveSemaphore();
	static_cast<PersistentStorage &>(i2cport).*PersistentStorageAccess::restoreDelayedFlagMember() = true; // restoreFlashDelayed() in the constructor
	SpiHandler::getSPIHandlers() = initialSpiHandlers;
	I2CHandler::getI2CHandlers() = initialI2cHandlers;
	// Encoders (P6): the "one instance" flags (static inUse, set by the constructor, cleared by the destructor) as after
	// start-up, in case a test left an encoder undeleted
	EncoderLocal::inUse = false;
	MtEncoderSPI::inUse = false;
	*privateMember(EncoderSSIInUse()) = false;
	*privateMember(EncoderBissCInUse()) = false;
	// Not reset: the device pointer of motor_uart (see HostTarget.h)
}

uint32_t canReceive(uint32_t id, const std::vector<uint8_t> &data, bool extId, bool rtr) {
	if (!hostCanPort.started) {
		return 0; // CANPort_2B::stop: HAL_CAN_Stop, nothing is received
	}
	// bxCAN 32 bit identifier mask filters (RM0090 32.7.4, register layout of CAN_FxR1/2 = CAN_RIxR): STID[10:0] or
	// EXID[28:18] in bits 31:21, EXID[17:0] in bits 20:3, IDE bit 2, RTR bit 1. The filter and mask registers are built
	// like CANPort_2B::addCanFilter (CANPort2B.cpp:272-287): a standard filter leaves IDE and RTR unmasked (FilterMaskIdLow
	// = 0), so it also accepts extended frames whose EXID[28:18] match; an extended filter masks IDE = 1, so it rejects
	// standard frames. The first matching active filter decides the FIFO.
	// (MockCANPort::passesFilter keeps its simpler generic semantics: no extended frame passes a standard filter.)
	const uint32_t frameReg = extId ? (((id & 0x1FFFFFFFU) << 3) | 0x4U | (rtr ? 0x2U : 0U)) : (((id & 0x7FFU) << 21) | (rtr ? 0x2U : 0U));
	const CAN_filter *match = nullptr;
	for (const CAN_filter &f : hostCanPort.filters) {
		uint32_t filterReg, maskReg;
		if (!f.extid) {
			filterReg = ((f.filter_id << 5) & 0xFFFFU) << 16;
			maskReg = ((f.filter_mask << 5) & 0xFFFFU) << 16;
		} else {
			filterReg = (((f.filter_id >> 13) & 0xFFFFU) << 16) | ((f.filter_id << 3) & 0xFFF8U) | 0x4U;
			maskReg = (((f.filter_mask >> 13) & 0xFFFFU) << 16) | ((f.filter_mask << 3) & 0xFFF8U) | 0x4U;
		}
		if (f.active && ((frameReg ^ filterReg) & maskReg) == 0U) {
			match = &f;
			break;
		}
	}
	if (match == nullptr) {
		return 0; // No acceptance filter matches: the frame is not stored (RM0090 32.7.4)
	}
	const uint32_t fifo = match->buffer == 0 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
	CAN_RxHeaderTypeDef header = {};
	header.StdId = extId ? 0 : id;
	header.ExtId = extId ? id : 0;
	header.IDE = extId ? CAN_ID_EXT : CAN_ID_STD;
	header.RTR = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	header.DLC = static_cast<uint32_t>(std::min<size_t>(data.size(), 8));
	header.FilterMatchIndex = static_cast<uint32_t>(match - hostCanPort.filters.data());
	uint8_t bytes[8] = {};
	std::copy(data.begin(), data.begin() + header.DLC, bytes);
	canQueueRx(fifo, header, bytes);
	return canRxInterrupt(fifo);
}
} // namespace HostTarget
