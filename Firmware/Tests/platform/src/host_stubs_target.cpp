/*
 * host_stubs_target.cpp
 *
 * Link stubs for ffb_target_tests: HAL functions that do not have a fake yet. Anything defined here is NOT tested.
 *
 * Remove an entry as soon as the fake is compiled:
 *  - fake HAL SPI (F4, P5/P6) and fake HAL I2C (F5, P5): HAL_SPI_*, HAL_I2C_*
 *
 * The firmware stubs of P2 (cppmain.cpp, global_callbacks.cpp, voltagesense.cpp) were removed in P4: the real
 * sources are compiled.
 */
#include "HostTarget.h"

#include "main.h"

// ---------------- HAL functions without a fake ----------------
// Recorded in HostTarget::state().unmodelledHalCalls and answered with HAL_ERROR. A test that reaches one of
// these needs the fake first; the fixtures check that the list is empty.
namespace {
HAL_StatusTypeDef unmodelled(const char *name) {
	HostTarget::state().unmodelledHalCalls.emplace_back(name);
	return HAL_ERROR;
}
} // namespace

extern "C" {
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *) { return unmodelled("HAL_SPI_Init"); }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_SPI_Transmit"); }
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_SPI_Receive"); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_SPI_TransmitReceive"); }
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Transmit_IT"); }
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Receive_IT"); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_TransmitReceive_IT"); }
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Transmit_DMA"); }
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Receive_DMA"); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_TransmitReceive_DMA"); }

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *) { return unmodelled("HAL_I2C_Init"); }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *) { return unmodelled("HAL_I2C_DeInit"); }
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Master_Transmit"); }
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Master_Receive"); }
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Transmit_IT"); }
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Receive_IT"); }
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Transmit_DMA"); }
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Receive_DMA"); }
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Mem_Write"); }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Mem_Read"); }
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Mem_Write_IT"); }
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Mem_Read_IT"); }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t, uint32_t) { return unmodelled("HAL_I2C_IsDeviceReady"); }
} // extern "C"
