#include "doctest.h"
#include "stm32f4xx_hal.h"
#include <array>

namespace {
using Callback = void (*)(DMA_HandleTypeDef *);
struct Slot { HAL_DMA_CallbackIDTypeDef id; Callback DMA_HandleTypeDef::*member; };
constexpr std::array<Slot, 6> slots{{
    {HAL_DMA_XFER_CPLT_CB_ID, &DMA_HandleTypeDef::XferCpltCallback},
    {HAL_DMA_XFER_HALFCPLT_CB_ID, &DMA_HandleTypeDef::XferHalfCpltCallback},
    {HAL_DMA_XFER_M1CPLT_CB_ID, &DMA_HandleTypeDef::XferM1CpltCallback},
    {HAL_DMA_XFER_M1HALFCPLT_CB_ID, &DMA_HandleTypeDef::XferM1HalfCpltCallback},
    {HAL_DMA_XFER_ERROR_CB_ID, &DMA_HandleTypeDef::XferErrorCallback},
    {HAL_DMA_XFER_ABORT_CB_ID, &DMA_HandleTypeDef::XferAbortCallback},
}};
void originalCallback(DMA_HandleTypeDef *) {}
void replacementCallback(DMA_HandleTypeDef *) {}
struct DmaFixture {
    DMA_HandleTypeDef handle{};
    DmaFixture() {
        p12_reset_tick_boundary();
        handle.State = HAL_DMA_STATE_READY;
        handle.Lock = HAL_UNLOCKED;
        handle.ErrorCode = HAL_DMA_ERROR_TE | HAL_DMA_ERROR_DME;
        for (const auto &slot : slots) handle.*(slot.member) = originalCallback;
        // Instance stays null: this group must not execute register paths or
        // truncate a 64-bit host pointer into the DMA's 32-bit address fields.
    }
    ~DmaFixture() { p12_reset_tick_boundary(); }
    void originalCallbacksRemain() {
        for (const auto &slot : slots) CHECK(handle.*(slot.member) == originalCallback);
    }
};
}
TEST_SUITE("P12RealDmaState") {
TEST_CASE_FIXTURE(DmaFixture, "ready DMA registration replaces only the selected callback slot") {
    for (const auto &selected : slots) {
        for (const auto &slot : slots) handle.*(slot.member) = originalCallback;
        REQUIRE(HAL_DMA_RegisterCallback(&handle, selected.id, replacementCallback) == HAL_OK);
        for (const auto &slot : slots)
            CHECK(handle.*(slot.member) == (slot.id == selected.id ? replacementCallback : originalCallback));
        CHECK(handle.Lock == HAL_UNLOCKED);
        CHECK(handle.State == HAL_DMA_STATE_READY);
        CHECK(handle.ErrorCode == (HAL_DMA_ERROR_TE | HAL_DMA_ERROR_DME));
    }
}
TEST_CASE_FIXTURE(DmaFixture, "ready DMA unregistration clears only the selected callback slot") {
    for (const auto &selected : slots) {
        for (const auto &slot : slots) handle.*(slot.member) = originalCallback;
        REQUIRE(HAL_DMA_UnRegisterCallback(&handle, selected.id) == HAL_OK);
        for (const auto &slot : slots)
            CHECK(handle.*(slot.member) == (slot.id == selected.id ? nullptr : originalCallback));
        CHECK(handle.Lock == HAL_UNLOCKED);
        CHECK(handle.State == HAL_DMA_STATE_READY);
    }
}
TEST_CASE_FIXTURE(DmaFixture, "ready DMA unregister all clears every callback slot") {
    REQUIRE(HAL_DMA_UnRegisterCallback(&handle, HAL_DMA_XFER_ALL_CB_ID) == HAL_OK);
    for (const auto &slot : slots) CHECK(handle.*(slot.member) == nullptr);
    CHECK(handle.Lock == HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(DmaFixture, "busy DMA rejects callback registration without changing the old callbacks") {
    handle.State = HAL_DMA_STATE_BUSY;
    CHECK(HAL_DMA_RegisterCallback(&handle, HAL_DMA_XFER_CPLT_CB_ID, replacementCallback) == HAL_ERROR);
    originalCallbacksRemain();
    CHECK(handle.State == HAL_DMA_STATE_BUSY);
    CHECK(handle.Lock == HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(DmaFixture, "busy DMA rejects callback unregistration without changing the old callbacks") {
    handle.State = HAL_DMA_STATE_BUSY;
    CHECK(HAL_DMA_UnRegisterCallback(&handle, HAL_DMA_XFER_ALL_CB_ID) == HAL_ERROR);
    originalCallbacksRemain();
    CHECK(handle.State == HAL_DMA_STATE_BUSY);
    CHECK(handle.Lock == HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(DmaFixture, "locked DMA callback registration reports busy without releasing another owner") {
    handle.Lock = HAL_LOCKED;
    CHECK(HAL_DMA_RegisterCallback(&handle, HAL_DMA_XFER_CPLT_CB_ID, replacementCallback) == HAL_BUSY);
    originalCallbacksRemain();
    CHECK(handle.Lock == HAL_LOCKED);
}
TEST_CASE_FIXTURE(DmaFixture, "locked DMA callback unregistration reports busy without releasing another owner") {
    handle.Lock = HAL_LOCKED;
    CHECK(HAL_DMA_UnRegisterCallback(&handle, HAL_DMA_XFER_ALL_CB_ID) == HAL_BUSY);
    originalCallbacksRemain();
    CHECK(handle.Lock == HAL_LOCKED);
}
TEST_CASE_FIXTURE(DmaFixture, "DMA state and error accessors preserve distinct public handle values") {
    handle.State = HAL_DMA_STATE_TIMEOUT;
    CHECK(HAL_DMA_GetState(&handle) == HAL_DMA_STATE_TIMEOUT);
    CHECK(HAL_DMA_GetError(&handle) == (HAL_DMA_ERROR_TE | HAL_DMA_ERROR_DME));
    CHECK(handle.Lock == HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(DmaFixture, "DMA initialization rejects a null handle") {
    CHECK(HAL_DMA_Init(nullptr) == HAL_ERROR);
}
TEST_CASE_FIXTURE(DmaFixture, "DMA polling without an active transfer reports no transfer and releases the handle") {
    handle.State = HAL_DMA_STATE_READY;
    handle.Lock = HAL_LOCKED;
    CHECK(HAL_DMA_PollForTransfer(&handle, HAL_DMA_FULL_TRANSFER, 1) == HAL_ERROR);
    CHECK(handle.ErrorCode == HAL_DMA_ERROR_NO_XFER);
    CHECK(handle.Lock == HAL_UNLOCKED);
    CHECK(handle.State == HAL_DMA_STATE_READY);
}
}
