/*
 * I2cTestSupport.h (P12 T4c): the strong HAL_I2C_* callbacks of the I2C test files, defined once in test_hal_i2c_support.cpp, plus
 * the helpers every I2C test file uses. The callbacks count their calls and remember the handle of the last call. I2cFixture resets
 * the counters at the start of each case.
 */
#ifndef FFB_TESTS_I2C_TEST_SUPPORT_H
#define FFB_TESTS_I2C_TEST_SUPPORT_H

#include "HalFixture.h"
#include "HalI2c.h"

struct I2cCallbackCounts {
	unsigned mspInit, mspDeInit;
	unsigned masterTx, masterRx, slaveTx, slaveRx, listen, memTx, memRx, error, abortCplt, addr;
	I2C_HandleTypeDef *last;                /* handle of the most recent callback */
	uint32_t errorCodeAtError;              /* hi2c->ErrorCode seen by the most recent ErrorCallback */
	HAL_I2C_StateTypeDef stateAtLast;       /* state seen by the most recent callback */
	HAL_I2C_ModeTypeDef modeAtLast;         /* mode seen by the most recent callback */
	uint8_t addrDirection;                  /* TransferDirection of the most recent AddrCallback */
	uint16_t addrMatchCode;                 /* AddrMatchCode of the most recent AddrCallback */
	unsigned order[32];                     /* callback kinds in call order (I2C_CB_*), at most 32 */
	unsigned orderCount;
};

enum { I2C_CB_MASTER_TX = 1, I2C_CB_MASTER_RX, I2C_CB_SLAVE_TX, I2C_CB_SLAVE_RX, I2C_CB_LISTEN, I2C_CB_MEM_TX, I2C_CB_MEM_RX, I2C_CB_ERROR, I2C_CB_ABORT, I2C_CB_ADDR };

extern I2cCallbackCounts i2cCounts;
void i2cSupportReset(void);

/* The interrupt controller the host does not have: calls HAL_I2C_EV_IRQHandler while the event request line is pending and
 * HAL_I2C_ER_IRQHandler while the error line is pending (error first, as the NVIC would pick the higher priority is not
 * modelled; both are served until neither is pending). Returns the number of handler calls; stops at `limit`. */
unsigned i2cRunIrqs(I2C_HandleTypeDef *hi2c, unsigned i2cIndex, unsigned limit = 64);

/* A blocking HAL call (polling slave functions, anything that waits for the bus) cannot be interleaved with the test body, so the
 * external controller is scripted: the step of HAL_GetTick (hostTickSetStep) runs HAL_IncTick and then every action whose tick has been
 * reached (uwTick >= atTick), once, in list order. i2cScriptStart copies up to 16 actions; i2cScriptStop restores the default step.
 * Actions run inside a HAL_GetTick call: they may use the hostI2c* API and plain registers, not doctest macros. */
struct I2cScriptAction {
	uint32_t atTick;
	void (*fn)(void *);
	void *arg;
};
void i2cScriptStart(const I2cScriptAction *actions, unsigned count);
void i2cScriptStop(void);
unsigned i2cScriptDone(void); /* actions already run */

/* Index (0..2) of an I2C instance. */
unsigned i2cIndexOf(const I2C_TypeDef *inst);

class I2cFixture : public HalFixture {
public:
	I2cFixture() { i2cSupportReset(); }
};

#endif /* FFB_TESTS_I2C_TEST_SUPPORT_H */
