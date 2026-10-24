/*
 * Host test replacement for the CubeMX generated main.h and the STM32 HAL headers of the F407 target.
 * Used by ffb_target_tests only (its include directory platform/target_f407 comes before platform/include).
 *
 * The generic fake HAL of ffb_unit_tests (platform/include/main.h) is not changed and not included:
 * the F407 sources need handles with more fields (timer registers, SPI/I2C/TIM state). The declarations
 * shared with platform/include/main.h (GPIO, UART, CAN, system) are identical, so the platform sources
 * (host_platform.cpp, ...) compile against both.
 *
 * Types follow the real STM32F4 HAL (Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Inc, read only) but only
 * contain the fields used by the compiled firmware sources. Constant values are copied from the HAL and
 * CMSIS device headers (stm32f407xx.h).
 *
 * Implementations: platform/src/host_target.cpp (TIM, EXTI, system, CMSIS),
 * platform/src/host_target_system.cpp (ADC, IWDG, CAN receive FIFO, TIM7 HAL tick, main.c callbacks) and
 * platform/src/host_platform.cpp (GPIO, UART, tick). Test control: support/HostTarget.h.
 * ffb_flash_tests additionally gets the FLASH HAL (host_flash_hal.h, see the end of this file).
 *
 * If a newly tested source file needs more of the HAL add it here. Keep it minimal.
 */
#ifndef HOSTTEST_TARGET_MAIN_H_
#define HOSTTEST_TARGET_MAIN_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __IO
#define __IO volatile
#endif
// stm32f4xx_hal_def.h, GNU compiler branch
#ifndef __ALIGN_END
#define __ALIGN_END __attribute__((aligned(4)))
#endif
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif

typedef enum { HAL_OK = 0x00U, HAL_ERROR = 0x01U, HAL_BUSY = 0x02U, HAL_TIMEOUT = 0x03U } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef enum { RESET = 0U, SET = !RESET } FlagStatus, ITStatus;
typedef enum { DISABLE = 0U, ENABLE = !DISABLE } FunctionalState;

// ---------------- GPIO (same as platform/include/main.h) ----------------
typedef struct { uint32_t ODR; uint32_t IDR; } GPIO_TypeDef;
typedef struct { uint32_t Pin; uint32_t Mode; uint32_t Pull; uint32_t Speed; uint32_t Alternate; } GPIO_InitTypeDef;
#define GPIO_NOPULL 0x00000000U
#define GPIO_PULLUP 0x00000001U
#define GPIO_PULLDOWN 0x00000002U
#define GPIO_SPEED_FREQ_LOW 0x00000000U
#define GPIO_MODE_INPUT 0x00000000U
#define GPIO_MODE_OUTPUT_PP 0x00000001U
#define GPIO_MODE_OUTPUT_OD 0x00000011U
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);

#define GPIO_PIN_0 ((uint16_t)0x0001)
#define GPIO_PIN_1 ((uint16_t)0x0002)
#define GPIO_PIN_2 ((uint16_t)0x0004)
#define GPIO_PIN_3 ((uint16_t)0x0008)
#define GPIO_PIN_4 ((uint16_t)0x0010)
#define GPIO_PIN_5 ((uint16_t)0x0020)
#define GPIO_PIN_6 ((uint16_t)0x0040)
#define GPIO_PIN_7 ((uint16_t)0x0080)
#define GPIO_PIN_8 ((uint16_t)0x0100)
#define GPIO_PIN_9 ((uint16_t)0x0200)
#define GPIO_PIN_10 ((uint16_t)0x0400)
#define GPIO_PIN_11 ((uint16_t)0x0800)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)

// Fake GPIO ports. Tests set input levels in IDR and read outputs from ODR (HAL_GPIO_* in host_platform.cpp)
extern GPIO_TypeDef hosttest_gpioa, hosttest_gpiob, hosttest_gpioc, hosttest_gpiod, hosttest_gpioe;
#define GPIOA (&hosttest_gpioa)
#define GPIOB (&hosttest_gpiob)
#define GPIOC (&hosttest_gpioc)
#define GPIOD (&hosttest_gpiod)
#define GPIOE (&hosttest_gpioe)

/**
 * HAL_GPIO_EXTI_Callback (stm32f4xx_hal_gpio.c, weak). Called by the fake EXTI interrupt (HostTarget::triggerExti).
 * Defined by the firmware (global_callbacks.cpp).
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// ---------------- Interrupt numbers referenced by the pin names (values of stm32f407xx.h) ----------------
typedef enum { EXTI2_IRQn = 8, EXTI9_5_IRQn = 23, EXTI15_10_IRQn = 40 } IRQn_Type;

// ---------------- TIM ----------------
typedef struct {
	__IO uint32_t CR1;	//!< Bit 0 CEN: counter enabled
	__IO uint32_t DIER; //!< Bit 0 UIE: update interrupt enabled
	__IO uint32_t SR;
	__IO uint32_t CCER; //!< Capture/compare outputs. __HAL_TIM_DISABLE only clears CEN while no CCxE / CCxNE bit is set
	__IO uint32_t CNT;
	__IO uint32_t PSC;
	__IO uint32_t ARR;
} TIM_TypeDef;
#define TIM_CR1_CEN 0x00000001U
#define TIM_DIER_UIE 0x00000001U
// stm32f4xx_hal_tim.h: CC1E|CC2E|CC3E|CC4E and CC1NE|CC2NE|CC3NE (stm32f407xx.h bit values)
#define TIM_CCER_CCxE_MASK 0x00001111U
#define TIM_CCER_CCxNE_MASK 0x00000444U

typedef enum {
	HAL_TIM_STATE_RESET = 0x00U,
	HAL_TIM_STATE_READY = 0x01U,
	HAL_TIM_STATE_BUSY = 0x02U,
	HAL_TIM_STATE_TIMEOUT = 0x03U,
	HAL_TIM_STATE_ERROR = 0x04U
} HAL_TIM_StateTypeDef;

typedef struct {
	uint32_t Prescaler;
	uint32_t CounterMode;
	uint32_t Period;
	uint32_t ClockDivision;
	uint32_t RepetitionCounter;
	uint32_t AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct {
	TIM_TypeDef *Instance;
	TIM_Base_InitTypeDef Init;
	__IO HAL_TIM_StateTypeDef State;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U
#define TIM_CHANNEL_4 0x0000000CU

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
/**
 * HAL_TIM_PeriodElapsedCallback (stm32f4xx_hal_tim.c, weak). Called by the fake update interrupt
 * (HostTarget::timerUpdate). Like on the target the strong definition is the one of main.c (modelled in
 * host_target_system.cpp: TIM7 -> HAL_IncTick, every other timer -> HAL_TIM_PeriodElapsedCallback_CPP of
 * global_callbacks.cpp). The __weak definition in global_callbacks.cpp is overridden, as on the target.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// HAL time base (Targets/F407VG/Core/Src/stm32f4xx_hal_timebase_tim.c): TIM7, 1 MHz counter, update every 1 ms.
// TIM_MICROS_HALTICK of target_constants.h; its CNT follows the simulated time (HostTarget::tickTimerCounter).
extern TIM_TypeDef hosttest_tim7;
#define TIM7 (&hosttest_tim7)
extern TIM_HandleTypeDef htim7;
/** Models HAL_IncTick (stm32f4xx_hal.c): uwTick += uwTickFreq. The fake only counts the calls (HostTarget::state().halIncTicks): HAL_GetTick() is derived from the simulated time */
void HAL_IncTick(void);

// ---------------- ADC (stm32f4xx_hal_adc.h, stm32f407xx.h; fake: host_target_system.cpp, F6) ----------------
typedef struct {
	__IO uint32_t SR;	//!< EOC (bit 1), STRT (bit 4), OVR (bit 5)
	__IO uint32_t CR1;	//!< RES (bits 24-25), JAUTO (bit 10)
	__IO uint32_t CR2;	//!< ADON, CONT, DMA, DDS, EOCS, EXTEN, SWSTART
	__IO uint32_t SQR1; //!< L (bits 20-23): number of regular conversions - 1
	__IO uint32_t DR;	//!< Regular data register
} ADC_TypeDef;
extern ADC_TypeDef hosttest_adc1, hosttest_adc2;
#define ADC1 (&hosttest_adc1)
#define ADC2 (&hosttest_adc2)
#define ADC_SR_EOC 0x00000002U
#define ADC_SR_STRT 0x00000010U
#define ADC_SR_OVR 0x00000020U
#define ADC_CR1_JAUTO 0x00000400U
#define ADC_CR1_RES_0 0x01000000U
#define ADC_CR1_RES_1 0x02000000U
#define ADC_CR1_RES 0x03000000U
#define ADC_CR2_ADON 0x00000001U
#define ADC_CR2_CONT 0x00000002U
#define ADC_CR2_DMA 0x00000100U
#define ADC_CR2_DDS 0x00000200U
#define ADC_CR2_EOCS 0x00000400U
#define ADC_CR2_EXTEN_0 0x10000000U
#define ADC_CR2_EXTEN_1 0x20000000U
#define ADC_CR2_EXTEN 0x30000000U
#define ADC_CR2_SWSTART 0x40000000U
#define ADC_SQR1_L 0x00F00000U
#define ADC_SQR1_L_Pos 20U

/** ADC_InitTypeDef of stm32f4xx_hal_adc.h (same fields and order) */
typedef struct {
	uint32_t ClockPrescaler;
	uint32_t Resolution;
	uint32_t DataAlign;
	uint32_t ScanConvMode;
	uint32_t EOCSelection;
	FunctionalState ContinuousConvMode;
	uint32_t NbrOfConversion;
	FunctionalState DiscontinuousConvMode;
	uint32_t NbrOfDiscConversion;
	uint32_t ExternalTrigConv;
	uint32_t ExternalTrigConvEdge;
	FunctionalState DMAContinuousRequests;
} ADC_InitTypeDef;
typedef enum { HAL_UNLOCKED = 0x00U, HAL_LOCKED = 0x01U } HAL_LockTypeDef;
/**
 * ADC_HandleTypeDef of stm32f4xx_hal_adc.h without NbrOfCurrentConversionRank and DMA_Handle: the DMA transfer of
 * HAL_ADC_Start_DMA is modelled by the fake (host_target_system.cpp), which keeps the target buffer per handle.
 */
typedef struct {
	ADC_TypeDef *Instance;
	ADC_InitTypeDef Init;
	HAL_LockTypeDef Lock;
	__IO uint32_t State;
	__IO uint32_t ErrorCode;
} ADC_HandleTypeDef;
#define ADC_RESOLUTION_12B 0x00000000U
#define ADC_RESOLUTION_10B 0x01000000U // ADC_CR1_RES_0
#define ADC_RESOLUTION_8B 0x02000000U  // ADC_CR1_RES_1
#define ADC_RESOLUTION_6B 0x03000000U  // ADC_CR1_RES
#define ADC_CLOCK_SYNC_PCLK_DIV4 0x00010000U
#define ADC_DATAALIGN_RIGHT 0x00000000U
#define ADC_EOC_SEQ_CONV 0x00000000U
#define ADC_EOC_SINGLE_CONV 0x00000001U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0x00000000U
#define ADC_EXTERNALTRIGCONVEDGE_RISING ADC_CR2_EXTEN_0
#define ADC_EXTERNALTRIGCONVEDGE_FALLING ADC_CR2_EXTEN_1
#define ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING ADC_CR2_EXTEN
#define ADC_EXTERNALTRIGCONV_T4_CC4 0x09000000U // ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_3
#define ADC_SOFTWARE_START 0x0F000001U			// ADC_CR2_EXTSEL + 1
#define ADC_FLAG_EOC ADC_SR_EOC
#define ADC_FLAG_STRT ADC_SR_STRT
#define ADC_FLAG_OVR ADC_SR_OVR
#define HAL_ADC_STATE_RESET 0x00000000U
#define HAL_ADC_STATE_READY 0x00000001U
#define HAL_ADC_STATE_BUSY_INTERNAL 0x00000002U
#define HAL_ADC_STATE_TIMEOUT 0x00000004U
#define HAL_ADC_STATE_ERROR_INTERNAL 0x00000010U
#define HAL_ADC_STATE_ERROR_CONFIG 0x00000020U
#define HAL_ADC_STATE_ERROR_DMA 0x00000040U
#define HAL_ADC_STATE_REG_BUSY 0x00000100U
#define HAL_ADC_STATE_REG_EOC 0x00000200U
#define HAL_ADC_STATE_REG_OVR 0x00000400U
#define HAL_ADC_STATE_INJ_BUSY 0x00001000U
#define HAL_ADC_STATE_INJ_EOC 0x00002000U
#define HAL_ADC_ERROR_NONE 0x00U
#define HAL_ADC_ERROR_INTERNAL 0x01U
#define HAL_ADC_ERROR_OVR 0x02U
#define HAL_ADC_ERROR_DMA 0x04U
#define HAL_MAX_DELAY 0xFFFFFFFFU

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);
/** HAL_ADC_ConvCpltCallback (weak in stm32f4xx_hal_adc.c). Defined by the firmware (global_callbacks.cpp), raised by HostTarget::adcConversionComplete */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

// ---------------- LL ADC helper macros (stm32f4xx_ll_adc.h, copied unchanged) ----------------
// Calibration values: on the chip they are read from the system memory (VREFINT_CAL 0x1FFF7A2A, TS_CAL1 0x1FFF7A2C,
// TS_CAL2 0x1FFF7A2E, stm32f4xx_ll_adc.h). On the host the address macros point to variables set by the test
// (HostTarget::adcCalibration()); the formulas are the same.
extern uint16_t hosttest_adc_vrefint_cal, hosttest_adc_ts_cal1, hosttest_adc_ts_cal2;
#define VREFINT_CAL_ADDR (&hosttest_adc_vrefint_cal)
#define VREFINT_CAL_VREF (3300UL)
#define TEMPSENSOR_CAL1_ADDR (&hosttest_adc_ts_cal1)
#define TEMPSENSOR_CAL2_ADDR (&hosttest_adc_ts_cal2)
#define TEMPSENSOR_CAL1_TEMP ((int32_t)30)
#define TEMPSENSOR_CAL2_TEMP ((int32_t)110)
#define TEMPSENSOR_CAL_VREFANALOG (3300UL)
#define ADC_CR1_RES_BITOFFSET_POS (24UL)
#define LL_ADC_RESOLUTION_12B 0x00000000UL
#define LL_ADC_RESOLUTION_10B (ADC_CR1_RES_0)
#define LL_ADC_RESOLUTION_8B (ADC_CR1_RES_1)
#define LL_ADC_RESOLUTION_6B (ADC_CR1_RES_1 | ADC_CR1_RES_0)
#define __LL_ADC_DIGITAL_SCALE(__ADC_RESOLUTION__) \
	(0xFFFU >> ((__ADC_RESOLUTION__) >> (ADC_CR1_RES_BITOFFSET_POS - 1UL)))
#define __LL_ADC_CONVERT_DATA_RESOLUTION(__DATA__, __ADC_RESOLUTION_CURRENT__, __ADC_RESOLUTION_TARGET__) \
	(((__DATA__)                                                                                       \
	  << ((__ADC_RESOLUTION_CURRENT__) >> (ADC_CR1_RES_BITOFFSET_POS - 1UL)))                          \
	 >> ((__ADC_RESOLUTION_TARGET__) >> (ADC_CR1_RES_BITOFFSET_POS - 1UL)))
#define __LL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, __ADC_DATA__, __ADC_RESOLUTION__) \
	((__ADC_DATA__) * (__VREFANALOG_VOLTAGE__)                                                  \
	 / __LL_ADC_DIGITAL_SCALE(__ADC_RESOLUTION__))
#define __LL_ADC_CALC_VREFANALOG_VOLTAGE(__VREFINT_ADC_DATA__, __ADC_RESOLUTION__) \
	(((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF)                            \
	 / __LL_ADC_CONVERT_DATA_RESOLUTION((__VREFINT_ADC_DATA__),                    \
										(__ADC_RESOLUTION__),                      \
										LL_ADC_RESOLUTION_12B))
#define __LL_ADC_CALC_TEMPERATURE(__VREFANALOG_VOLTAGE__, __TEMPSENSOR_ADC_DATA__, __ADC_RESOLUTION__) \
	(((( ((int32_t)((__LL_ADC_CONVERT_DATA_RESOLUTION((__TEMPSENSOR_ADC_DATA__),                     \
													  (__ADC_RESOLUTION__),                          \
													  LL_ADC_RESOLUTION_12B)                         \
					 * (__VREFANALOG_VOLTAGE__))                                                     \
					/ TEMPSENSOR_CAL_VREFANALOG)                                                     \
		  - (int32_t) *TEMPSENSOR_CAL1_ADDR)                                                         \
		) * (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)                                   \
	   ) / (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR)                \
	  ) + TEMPSENSOR_CAL1_TEMP)

// ---------------- IWDG (stm32f4xx_hal_iwdg.h; MX_IWDG_Init in main.c, HAL_IWDG_MODULE_ENABLED in stm32f4xx_hal_conf.h) ----------------
#define HAL_IWDG_MODULE_ENABLED
typedef struct { __IO uint32_t KR; __IO uint32_t PR; __IO uint32_t RLR; __IO uint32_t SR; } IWDG_TypeDef;
typedef struct { uint32_t Prescaler; uint32_t Reload; } IWDG_InitTypeDef;
typedef struct { IWDG_TypeDef *Instance; IWDG_InitTypeDef Init; } IWDG_HandleTypeDef;
#define IWDG_KEY_RELOAD 0x0000AAAAU
extern IWDG_HandleTypeDef hiwdg;
/** Models HAL_IWDG_Refresh (stm32f4xx_hal_iwdg.c): writes IWDG_KEY_RELOAD to KR, HAL_OK. Counted (HostTarget::state().watchdogRefreshes) */
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);

// ---------------- SPI (link stubs only. Fake SPI: F4, later phase) ----------------
typedef struct {
	uint32_t Mode;
	uint32_t Direction;
	uint32_t DataSize;
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
	uint32_t NSS;
	uint32_t BaudRatePrescaler;
	uint32_t FirstBit;
	uint32_t TIMode;
	uint32_t CRCCalculation;
	uint32_t CRCPolynomial;
} SPI_InitTypeDef;
typedef enum {
	HAL_SPI_STATE_RESET = 0x00U,
	HAL_SPI_STATE_READY = 0x01U,
	HAL_SPI_STATE_BUSY = 0x02U,
	HAL_SPI_STATE_BUSY_TX = 0x03U,
	HAL_SPI_STATE_BUSY_RX = 0x04U,
	HAL_SPI_STATE_BUSY_TX_RX = 0x05U,
	HAL_SPI_STATE_ERROR = 0x06U,
	HAL_SPI_STATE_ABORT = 0x07U
} HAL_SPI_StateTypeDef;
typedef struct {
	void *Instance;
	SPI_InitTypeDef Init;
	__IO HAL_SPI_StateTypeDef State;
	__IO uint32_t ErrorCode;
} SPI_HandleTypeDef;

#define SPI_MODE_MASTER 0x00000104U
#define SPI_DIRECTION_2LINES 0x00000000U
#define SPI_DATASIZE_8BIT 0x00000000U
#define SPI_POLARITY_LOW 0x00000000U
#define SPI_POLARITY_HIGH 0x00000002U // SPI_CR1_CPOL
#define SPI_PHASE_1EDGE 0x00000000U
#define SPI_PHASE_2EDGE 0x00000001U // SPI_CR1_CPHA
#define SPI_NSS_SOFT 0x00000200U
#define SPI_BAUDRATEPRESCALER_2 0x00000000U
#define SPI_BAUDRATEPRESCALER_4 0x00000008U
#define SPI_BAUDRATEPRESCALER_8 0x00000010U
#define SPI_BAUDRATEPRESCALER_16 0x00000018U
#define SPI_BAUDRATEPRESCALER_32 0x00000020U
#define SPI_BAUDRATEPRESCALER_64 0x00000028U
#define SPI_BAUDRATEPRESCALER_128 0x00000030U
#define SPI_BAUDRATEPRESCALER_256 0x00000038U
#define SPI_FIRSTBIT_MSB 0x00000000U
#define SPI_FIRSTBIT_LSB 0x00000080U // SPI_CR1_LSBFIRST
#define SPI_TIMODE_DISABLE 0x00000000U
#define SPI_CRCCALCULATION_DISABLE 0x00000000U

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

// ---------------- I2C (link stubs only. Fake I2C: F5, later phase) ----------------
typedef struct {
	uint32_t ClockSpeed;
	uint32_t DutyCycle;
	uint32_t OwnAddress1;
	uint32_t AddressingMode;
	uint32_t DualAddressMode;
	uint32_t OwnAddress2;
	uint32_t GeneralCallMode;
	uint32_t NoStretchMode;
} I2C_InitTypeDef;
typedef enum {
	HAL_I2C_STATE_RESET = 0x00U,
	HAL_I2C_STATE_READY = 0x20U,
	HAL_I2C_STATE_BUSY = 0x24U,
	HAL_I2C_STATE_BUSY_TX = 0x21U,
	HAL_I2C_STATE_BUSY_RX = 0x22U,
	HAL_I2C_STATE_LISTEN = 0x28U,
	HAL_I2C_STATE_BUSY_TX_LISTEN = 0x29U,
	HAL_I2C_STATE_BUSY_RX_LISTEN = 0x2AU,
	HAL_I2C_STATE_ABORT = 0x60U,
	HAL_I2C_STATE_TIMEOUT = 0xA0U,
	HAL_I2C_STATE_ERROR = 0xE0U
} HAL_I2C_StateTypeDef;
typedef struct {
	void *Instance;
	I2C_InitTypeDef Init;
	__IO HAL_I2C_StateTypeDef State;
	__IO uint32_t ErrorCode;
} I2C_HandleTypeDef;

#define I2C_DUTYCYCLE_2 0x00000000U
#define I2C_ADDRESSINGMODE_7BIT 0x00004000U
#define I2C_DUALADDRESS_DISABLE 0x00000000U
#define I2C_GENERALCALL_DISABLE 0x00000000U
#define I2C_NOSTRETCH_DISABLE 0x00000000U
#define I2C_NOSTRETCH_ENABLE 0x00000080U // I2C_CR1_NOSTRETCH

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

// ---------------- UART (same as platform/include/main.h) ----------------
typedef struct { uint32_t SR; uint32_t DR; } USART_TypeDef;
typedef struct { uint32_t BaudRate; uint32_t WordLength; uint32_t StopBits; uint32_t Parity; uint32_t Mode; uint32_t HwFlowCtl; uint32_t OverSampling; } UART_InitTypeDef;
typedef struct { USART_TypeDef *Instance; UART_InitTypeDef Init; uint32_t ErrorCode; } UART_HandleTypeDef;

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart);

// ---------------- bxCAN (same as platform/include/main.h). Selects the CAN 2.0B variant of CAN.h ----------------
#define CAN1 ((void *)0x40006400)
#define CAN_ID_STD (0x00000000U)
#define CAN_ID_EXT (0x00000004U)
#define CAN_RTR_DATA (0x00000000U)
#define CAN_RTR_REMOTE (0x00000002U)
typedef struct { uint32_t StdId; uint32_t ExtId; uint32_t IDE; uint32_t RTR; uint32_t DLC; uint32_t Timestamp; uint32_t FilterMatchIndex; } CAN_RxHeaderTypeDef;
// ffb_target_tests: State and ErrorCode of the real CAN_HandleTypeDef (stm32f4xx_hal_can.h), used by the receive FIFO
// fake (host_target_system.cpp) and the error callback of global_callbacks.cpp
typedef enum {
	HAL_CAN_STATE_RESET = 0x00U,
	HAL_CAN_STATE_READY = 0x01U,
	HAL_CAN_STATE_LISTENING = 0x02U,
	HAL_CAN_STATE_SLEEP_PENDING = 0x03U,
	HAL_CAN_STATE_SLEEP_ACTIVE = 0x04U,
	HAL_CAN_STATE_ERROR = 0x05U
} HAL_CAN_StateTypeDef;
typedef struct { void *Instance; __IO HAL_CAN_StateTypeDef State; __IO uint32_t ErrorCode; } CAN_HandleTypeDef;
#define CAN_RX_FIFO0 (0x00000000U)
#define CAN_RX_FIFO1 (0x00000001U)
#define HAL_CAN_ERROR_NONE (0x00000000U)
#define HAL_CAN_ERROR_RX_FOV0 (0x00000200U)
#define HAL_CAN_ERROR_RX_FOV1 (0x00000400U)
#define HAL_CAN_ERROR_NOT_INITIALIZED (0x00040000U)
#define HAL_CAN_ERROR_PARAM (0x00200000U)
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan, uint32_t RxFifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
// CAN interrupt callbacks (weak in stm32f4xx_hal_can.c), defined by the firmware (global_callbacks.cpp)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

// ---------------- Other interrupt callbacks (weak in the HAL), defined by the firmware (global_callbacks.cpp) ----------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

// ---------------- System ----------------
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_NVIC_SystemReset(void);
/** Error_Handler of main.c. ffb_target_tests: modelled in host_target_system.cpp (error LED off, counted, returns) */
void Error_Handler(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
uint32_t HAL_GetDEVID(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
extern uint32_t SystemCoreClock;

/**
 * Models NVIC_SystemReset (core_cm4.h). On the target it never returns. The fake records the request
 * (HostTarget::state().systemResets) and returns, so firmware code after the call is executed on the host.
 */
void NVIC_SystemReset(void);

// Declared by Targets/F407VG/Core/Inc/main.h (defined in main.c). Recorded by the fake, see HostTarget.h
void RebootDFU(void);
void ExitDFU(void);

// ---------------- CMSIS intrinsics (cmsis_gcc.h semantics, portable C) ----------------
/** __REV: reverses the byte order of a 32 bit value */
static inline uint32_t __REV(uint32_t value) {
	return ((value & 0x000000FFU) << 24) | ((value & 0x0000FF00U) << 8) | ((value & 0x00FF0000U) >> 8) | ((value & 0xFF000000U) >> 24);
}
/** __REV16: reverses the byte order within each halfword of a 32 bit value */
static inline uint32_t __REV16(uint32_t value) {
	return ((value & 0x00FF00FFU) << 8) | ((value & 0xFF00FF00U) >> 8);
}
/** __REVSH: reverses the byte order of the lower halfword and sign extends it (int16_t result) */
static inline int16_t __REVSH(int16_t value) {
	uint16_t v = (uint16_t)value;
	return (int16_t)(uint16_t)(((v & 0x00FFU) << 8) | ((v & 0xFF00U) >> 8));
}

// ---------------- Pin names copied unchanged from Targets/F407VG/Core/Inc/main.h ----------------
#define DIN7_Pin GPIO_PIN_2
#define DIN7_GPIO_Port GPIOE
#define DIN6_Pin GPIO_PIN_3
#define DIN6_GPIO_Port GPIOE
#define DIN5_Pin GPIO_PIN_4
#define DIN5_GPIO_Port GPIOE
#define DIN4_Pin GPIO_PIN_5
#define DIN4_GPIO_Port GPIOE
#define DIN3_Pin GPIO_PIN_6
#define DIN3_GPIO_Port GPIOE
#define DIN2_Pin GPIO_PIN_13
#define DIN2_GPIO_Port GPIOC
#define DIN1_Pin GPIO_PIN_14
#define DIN1_GPIO_Port GPIOC
#define DIN0_Pin GPIO_PIN_15
#define DIN0_GPIO_Port GPIOC
#define AIN5_Pin GPIO_PIN_0
#define AIN5_GPIO_Port GPIOC
#define AIN4_Pin GPIO_PIN_1
#define AIN4_GPIO_Port GPIOC
#define AIN3_Pin GPIO_PIN_2
#define AIN3_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_3
#define AIN2_GPIO_Port GPIOC
#define GP6_Pin GPIO_PIN_0
#define GP6_GPIO_Port GPIOA
#define GP5_Pin GPIO_PIN_1
#define GP5_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_2
#define AIN1_GPIO_Port GPIOA
#define AIN0_Pin GPIO_PIN_3
#define AIN0_GPIO_Port GPIOA
#define SPI1_SS1_Pin GPIO_PIN_4
#define SPI1_SS1_GPIO_Port GPIOA
#define ADC_VEXT_Pin GPIO_PIN_4
#define ADC_VEXT_GPIO_Port GPIOC
#define ADC_VINT_Pin GPIO_PIN_5
#define ADC_VINT_GPIO_Port GPIOC
#define SPI1_SS2_Pin GPIO_PIN_0
#define SPI1_SS2_GPIO_Port GPIOB
#define SPI1_SS3_Pin GPIO_PIN_1
#define SPI1_SS3_GPIO_Port GPIOB
#define BUTTON_A_Pin GPIO_PIN_2
#define BUTTON_A_GPIO_Port GPIOB
#define BUTTON_A_EXTI_IRQn EXTI2_IRQn
#define DRV_ENABLE_Pin GPIO_PIN_7
#define DRV_ENABLE_GPIO_Port GPIOE
#define FLAG_Pin GPIO_PIN_8
#define FLAG_GPIO_Port GPIOE
#define FLAG_EXTI_IRQn EXTI9_5_IRQn
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define DRV_BRAKE_Pin GPIO_PIN_10
#define DRV_BRAKE_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define DRV_GP1_Pin GPIO_PIN_12
#define DRV_GP1_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define GP7_Pin GPIO_PIN_10
#define GP7_GPIO_Port GPIOB
#define GP8_Pin GPIO_PIN_11
#define GP8_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI2_SS2_Pin GPIO_PIN_8
#define SPI2_SS2_GPIO_Port GPIOD
#define SPI2_SS3_Pin GPIO_PIN_9
#define SPI2_SS3_GPIO_Port GPIOD
#define ENCODER_Z_Pin GPIO_PIN_15
#define ENCODER_Z_GPIO_Port GPIOD
#define ENCODER_Z_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOC
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOC
#define ENCODER_Z_1_2_1_Pin GPIO_PIN_8
#define ENCODER_Z_1_2_1_GPIO_Port GPIOC
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI3_SS1_Pin GPIO_PIN_15
#define SPI3_SS1_GPIO_Port GPIOA
#define SPI3_SS2_Pin GPIO_PIN_2
#define SPI3_SS2_GPIO_Port GPIOD
#define SPI3_SS3_Pin GPIO_PIN_3
#define SPI3_SS3_GPIO_Port GPIOD
#define CAN_S_Pin GPIO_PIN_4
#define CAN_S_GPIO_Port GPIOD
#define E_STOP_Pin GPIO_PIN_5
#define E_STOP_GPIO_Port GPIOD
#define E_STOP_EXTI_IRQn EXTI9_5_IRQn
#define GP1_Pin GPIO_PIN_6
#define GP1_GPIO_Port GPIOD
#define LED_SYS_Pin GPIO_PIN_7
#define LED_SYS_GPIO_Port GPIOD
#define GP4_Pin GPIO_PIN_3
#define GP4_GPIO_Port GPIOB
#define GP2_Pin GPIO_PIN_4
#define GP2_GPIO_Port GPIOB
#define GP3_Pin GPIO_PIN_5
#define GP3_GPIO_Port GPIOB
#define LED_CLIP_Pin GPIO_PIN_0
#define LED_CLIP_GPIO_Port GPIOE
#define LED_ERR_Pin GPIO_PIN_1
#define LED_ERR_GPIO_Port GPIOE

#ifdef __cplusplus
}
#endif

// FLASH HAL (stm32f4xx_hal_flash.h) with the fake flash of ffb_flash_tests. Only ffb_flash_tests defines
// HOSTTEST_FLASH_HAL; for ffb_target_tests this header is unchanged (no FLASH_OTP_BASE, so no OTPMEMORY).
#ifdef HOSTTEST_FLASH_HAL
#include "host_flash_hal.h"
#endif

#endif
