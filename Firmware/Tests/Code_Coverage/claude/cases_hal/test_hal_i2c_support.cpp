// P12 T4c: the strong I2C callbacks shared by the I2C test files (see I2cTestSupport.h).
#include "I2cTestSupport.h"

#include <cstring>

I2cCallbackCounts i2cCounts;

void i2cScriptStop(void);
void i2cSupportReset(void) {
	std::memset(&i2cCounts, 0, sizeof i2cCounts);
	i2cScriptStop();
}

unsigned i2cIndexOf(const I2C_TypeDef *inst) { return inst == I2C1 ? 0u : (inst == I2C2 ? 1u : 2u); }

unsigned i2cRunIrqs(I2C_HandleTypeDef *hi2c, unsigned idx, unsigned limit) {
	unsigned calls = 0;
	while (calls < limit && (hostI2cErrorPending(idx) || hostI2cEventPending(idx))) {
		if (hostI2cErrorPending(idx)) {
			HAL_I2C_ER_IRQHandler(hi2c);
		} else {
			HAL_I2C_EV_IRQHandler(hi2c);
		}
		++calls;
	}
	return calls;
}

namespace {
I2cScriptAction scriptActions[16];
bool scriptFired[16];
unsigned scriptCount, scriptFiredCount;
void scriptStep(void) {
	HAL_IncTick();
	for (unsigned k = 0; k < scriptCount; ++k) {
		if (!scriptFired[k] && uwTick >= scriptActions[k].atTick) {
			scriptFired[k] = true;
			++scriptFiredCount;
			scriptActions[k].fn(scriptActions[k].arg);
		}
	}
}
} // namespace

void i2cScriptStart(const I2cScriptAction *actions, unsigned count) {
	scriptCount = count > 16u ? 16u : count;
	scriptFiredCount = 0;
	for (unsigned k = 0; k < scriptCount; ++k) {
		scriptActions[k] = actions[k];
		scriptFired[k] = false;
	}
	hostTickSetStep(scriptStep);
}
void i2cScriptStop(void) {
	hostTickSetStep(nullptr);
	scriptCount = scriptFiredCount = 0;
}
unsigned i2cScriptDone(void) { return scriptFiredCount; }

extern "C" {
static void note(I2C_HandleTypeDef *h, unsigned kind) {
	i2cCounts.last = h;
	i2cCounts.stateAtLast = h->State;
	i2cCounts.modeAtLast = h->Mode;
	if (kind != 0u && i2cCounts.orderCount < 32u) i2cCounts.order[i2cCounts.orderCount++] = kind;
}
void HAL_I2C_MspInit(I2C_HandleTypeDef *h) { i2cCounts.mspInit++; note(h, 0); }
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *h) { i2cCounts.mspDeInit++; note(h, 0); }
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.masterTx++; note(h, I2C_CB_MASTER_TX); }
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.masterRx++; note(h, I2C_CB_MASTER_RX); }
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.slaveTx++; note(h, I2C_CB_SLAVE_TX); }
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.slaveRx++; note(h, I2C_CB_SLAVE_RX); }
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.listen++; note(h, I2C_CB_LISTEN); }
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.memTx++; note(h, I2C_CB_MEM_TX); }
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.memRx++; note(h, I2C_CB_MEM_RX); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h) {
	i2cCounts.error++;
	i2cCounts.errorCodeAtError = h->ErrorCode;
	note(h, I2C_CB_ERROR);
}
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *h) { i2cCounts.abortCplt++; note(h, I2C_CB_ABORT); }
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *h, uint8_t dir, uint16_t code) {
	i2cCounts.addr++;
	i2cCounts.addrDirection = dir;
	i2cCounts.addrMatchCode = code;
	note(h, I2C_CB_ADDR);
}
}
