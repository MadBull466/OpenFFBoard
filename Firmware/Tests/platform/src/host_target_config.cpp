/*
 * host_target_config.cpp
 *
 * Host replacement of Targets/F407VG/Core/Src/cpp_target_config.cpp (target hardware configuration, not
 * tested) and of the peripheral handles defined by the CubeMX code in main.c, for ffb_target_tests.
 *
 * Same objects and parameters as the F407 file, with one difference: `canport` is a MockCANPort instead of
 * CANPort_2B (CANPort2B.cpp needs the fake HAL CAN F8, planned for P7). huart1, hcan1 and external_uart are
 * defined in host_platform.cpp.
 */
#include "cpp_target_config.h"
#include "HostTarget.h"
#include "MockCANPort.h"

#include <array>

// Peripheral handles (main.c). Instance values are only compared by the firmware
SPI_HandleTypeDef hspi1 = {(void *)0x40013000, {}, HAL_SPI_STATE_READY, 0}; // SPI1
SPI_HandleTypeDef hspi2 = {(void *)0x40003800, {}, HAL_SPI_STATE_READY, 0}; // SPI2
SPI_HandleTypeDef hspi3 = {(void *)0x40003C00, {}, HAL_SPI_STATE_READY, 0}; // SPI3
I2C_HandleTypeDef hi2c1 = {(void *)0x40005400, {}, HAL_I2C_STATE_READY, 0}; // I2C1
UART_HandleTypeDef huart3 = {};
ADC_HandleTypeDef hadc1 = {(void *)0x40012000, {ADC_RESOLUTION_12B}}; // ADC1, MX_ADC1_Init: 12 bit
ADC_HandleTypeDef hadc2 = {(void *)0x40012100, {ADC_RESOLUTION_12B}}; // ADC2, MX_ADC2_Init: 12 bit

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
static MockCANPort hostCanPort;
CANPort &canport = hostCanPort;

const OutputPin debugpin = OutputPin(*GP1_GPIO_Port, GP1_Pin);

namespace HostTarget {
MockCANPort &canPort() { return hostCanPort; }

void resetTargetConfig() {
	// Handles: back to the state after MX_SPIx_Init / MX_I2C1_Init / MX_ADCx_Init (HAL_xxx_Init sets State READY)
	hspi1 = {(void *)0x40013000, {}, HAL_SPI_STATE_READY, 0};
	hspi2 = {(void *)0x40003800, {}, HAL_SPI_STATE_READY, 0};
	hspi3 = {(void *)0x40003C00, {}, HAL_SPI_STATE_READY, 0};
	hi2c1 = {(void *)0x40005400, {}, HAL_I2C_STATE_READY, 0};
	huart3 = {};
	hadc1 = {(void *)0x40012000, {ADC_RESOLUTION_12B}};
	hadc2 = {(void *)0x40012100, {ADC_RESOLUTION_12B}};
	// Release the motor uart in case a test left a transfer unfinished (like external_uart in HostPlatform::reset)
	motor_uart.giveSemaphore(true);
	motor_uart.giveSemaphore(false);
	hostCanPort.resetForTest();
	// Not reset (private state, see HostTarget.h): external_spi / motor_spi / ext3_spi, i2cport, motor_uart device
}
} // namespace HostTarget
