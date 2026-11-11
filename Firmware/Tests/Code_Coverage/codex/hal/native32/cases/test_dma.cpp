#include "native.h"
namespace {
unsigned complete, half, error, aborts;
void done(DMA_HandleTypeDef*) { ++complete; }
void halfway(DMA_HandleTypeDef*) { ++half; }
void failed(DMA_HandleTypeDef*) { ++error; }
void aborted(DMA_HandleTypeDef*) { ++aborts; }
struct Dma : native::Fixture {
    DMA_HandleTypeDef h{};
    Dma() {
        complete=half=error=aborts=0;
        h.Instance=DMA1_Stream0;
        h.Init.Channel=DMA_CHANNEL_2;
        h.Init.Direction=DMA_MEMORY_TO_PERIPH;
        h.Init.MemInc=DMA_MINC_ENABLE;
        h.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;
        h.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;
        h.Init.Priority=DMA_PRIORITY_HIGH;
        h.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
        h.XferCpltCallback=done; h.XferHalfCpltCallback=halfway;
        h.XferErrorCallback=failed; h.XferAbortCallback=aborted;
    }
};
}
TEST_SUITE("P12NativeDMA") {
TEST_CASE_FIXTURE(Dma,"DMA initialization programs channel direction alignment and increment requests") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    CHECK((h.Instance->CR&DMA_SxCR_CHSEL)==DMA_CHANNEL_2);
    CHECK((h.Instance->CR&DMA_SxCR_DIR)==DMA_MEMORY_TO_PERIPH);
    CHECK((h.Instance->CR&DMA_SxCR_MSIZE)==DMA_MDATAALIGN_HALFWORD);
    CHECK((h.Instance->CR&DMA_SxCR_PSIZE)==DMA_PDATAALIGN_HALFWORD);
    CHECK((h.Instance->CR&DMA_SxCR_MINC)!=0);
    CHECK(h.State==HAL_DMA_STATE_READY);
}
TEST_CASE_FIXTURE(Dma,"DMA stream bank selection distinguishes all eight flag groups") {
    const unsigned shifts[8]={0,6,16,22,0,6,16,22}; // RM0090 DMA flag groups
    for(unsigned i=0;i<8;++i) {
        h.Instance=reinterpret_cast<DMA_Stream_TypeDef*>(DMA1_Stream0_BASE+24*i);
        REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
        CHECK(h.StreamIndex==shifts[i]);
        CHECK(h.StreamBaseAddress==DMA1_BASE+(i>=4?4:0));
        CHECK((i>=4?DMA1->HIFCR:DMA1->LIFCR)==(0x3fU<<shifts[i]));
    }
}
TEST_CASE_FIXTURE(Dma,"DMA memory to peripheral start assigns source destination and element count") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start(&h,0x20000100,0x4001300c,7)==HAL_OK);
    CHECK(h.Instance->M0AR==0x20000100); CHECK(h.Instance->PAR==0x4001300c);
    CHECK(h.Instance->NDTR==7); CHECK((h.Instance->CR&DMA_SxCR_EN)!=0);
    CHECK(h.State==HAL_DMA_STATE_BUSY); CHECK(h.Lock==HAL_LOCKED);
}
TEST_CASE_FIXTURE(Dma,"DMA peripheral to memory start reverses address ownership") {
    h.Init.Direction=DMA_PERIPH_TO_MEMORY;
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start(&h,0x4001300c,0x20000200,3)==HAL_OK);
    CHECK(h.Instance->PAR==0x4001300c); CHECK(h.Instance->M0AR==0x20000200);
}
TEST_CASE_FIXTURE(Dma,"busy DMA rejects a second start without replacing its addresses") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start(&h,0x20000100,0x4001300c,7)==HAL_OK);
    CHECK(HAL_DMA_Start(&h,0x20000500,0x4000300c,2)==HAL_BUSY);
    CHECK(h.Instance->M0AR==0x20000100); CHECK(h.Instance->NDTR==7);
}
TEST_CASE_FIXTURE(Dma,"DMA completion IRQ requests flag clearing and releases a normal transfer before callback") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start_IT(&h,0x20000100,0x4001300c,7)==HAL_OK);
    DMA1->LISR=DMA_FLAG_TCIF0_4;
    HAL_DMA_IRQHandler(&h);
    CHECK(complete==1); CHECK(h.State==HAL_DMA_STATE_READY); CHECK(h.Lock==HAL_UNLOCKED);
    CHECK(DMA1->LIFCR==DMA_FLAG_TCIF0_4);
}
TEST_CASE_FIXTURE(Dma,"DMA half completion callback preserves the active transfer") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start_IT(&h,0x20000100,0x4001300c,7)==HAL_OK);
    DMA1->LISR=DMA_FLAG_HTIF0_4;
    HAL_DMA_IRQHandler(&h);
    CHECK(half==1); CHECK(complete==0); CHECK(h.State==HAL_DMA_STATE_BUSY);
}
TEST_CASE_FIXTURE(Dma,"DMA transfer error IRQ records the error and invokes its callback") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start_IT(&h,0x20000100,0x4001300c,7)==HAL_OK);
    DMA1->LISR=DMA_FLAG_TEIF0_4;
    HAL_DMA_IRQHandler(&h);
    CHECK(error==1); CHECK((h.ErrorCode&HAL_DMA_ERROR_TE)!=0);
}
TEST_CASE_FIXTURE(Dma,"DMA polling observes an explicitly injected completion flag") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start(&h,0x20000100,0x4001300c,7)==HAL_OK);
    native::event([](uint32_t){DMA1->LISR=DMA_FLAG_TCIF0_4;});
    CHECK(HAL_DMA_PollForTransfer(&h,HAL_DMA_FULL_TRANSFER,5)==HAL_OK);
    CHECK(h.State==HAL_DMA_STATE_READY); CHECK(h.Lock==HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(Dma,"DMA polling times out when no completion event arrives") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start(&h,0x20000100,0x4001300c,7)==HAL_OK);
    CHECK(HAL_DMA_PollForTransfer(&h,HAL_DMA_FULL_TRANSFER,2)==HAL_TIMEOUT);
    CHECK((h.ErrorCode&HAL_DMA_ERROR_TIMEOUT)!=0); CHECK(h.Lock==HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(Dma,"DMA abort disables the stream and makes the handle reusable") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start(&h,0x20000100,0x4001300c,7)==HAL_OK);
    REQUIRE(HAL_DMA_Abort(&h)==HAL_OK);
    CHECK((h.Instance->CR&DMA_SxCR_EN)==0); CHECK(h.State==HAL_DMA_STATE_READY);
    CHECK(HAL_DMA_Start(&h,0x20000300,0x4001300c,4)==HAL_OK);
}
TEST_CASE_FIXTURE(Dma,"DMA double buffer start preserves two distinct memory endpoints") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMAEx_MultiBufferStart(&h,0x20000100,0x4001300c,0x20000500,6)==HAL_OK);
    CHECK(h.Instance->M0AR==0x20000100); CHECK(h.Instance->M1AR==0x20000500);
    CHECK((h.Instance->CR&DMA_SxCR_DBM)!=0);
}
TEST_CASE_FIXTURE(Dma,"DMA double buffer memory change targets only the requested bank") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    h.Instance->M0AR=0x20000100;
    REQUIRE(HAL_DMAEx_ChangeMemory(&h,0x20000800,MEMORY1)==HAL_OK);
    CHECK(h.Instance->M0AR==0x20000100); CHECK(h.Instance->M1AR==0x20000800);
}
TEST_CASE_FIXTURE(Dma,"DMA deinitialization resets a stopped stream and its state") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    CHECK(HAL_DMA_DeInit(&h)==HAL_OK);
    CHECK(h.State==HAL_DMA_STATE_RESET); CHECK(h.Instance->CR==0);
}
}
TEST_SUITE("P12NativeDMARecovery") {
TEST_CASE_FIXTURE(Dma,"DMA FIFO threshold accepts only bursts that fit the configured capacity") {
    // RM0090 DMA FIFO table: capacities 4/8/12/16 bytes; bursts must fit
    // in whole threshold-sized groups. SINGLE bypasses the burst check.
    for(unsigned width=0;width<3;++width) for(unsigned threshold=0;threshold<4;++threshold)
    for(unsigned burst=1;burst<4;++burst) {
        CAPTURE(width);CAPTURE(threshold);CAPTURE(burst);native::reset();h.Instance=DMA1_Stream0;
        h.Init.FIFOMode=DMA_FIFOMODE_ENABLE;h.Init.FIFOThreshold=threshold;
        h.Init.MemDataAlignment=width<<13;h.Init.MemBurst=burst<<23;
        unsigned capacity=4*(threshold+1),bytes=(1U<<width)*(1U<<(burst+1));
        bool fits=bytes<=capacity&&capacity%bytes==0;
        CHECK(HAL_DMA_Init(&h)==(fits?HAL_OK:HAL_ERROR));
        CHECK(h.State==HAL_DMA_STATE_READY);
        CHECK(HAL_DMA_GetError(&h)==(fits?HAL_DMA_ERROR_NONE:HAL_DMA_ERROR_PARAM));
    }
}
TEST_CASE_FIXTURE(Dma,"DMA callback registration and removal affect only the selected callback") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);h.XferCpltCallback=nullptr;
    REQUIRE(HAL_DMA_RegisterCallback(&h,HAL_DMA_XFER_CPLT_CB_ID,done)==HAL_OK);
    CHECK(h.XferCpltCallback==done);CHECK(h.XferHalfCpltCallback==halfway);
    REQUIRE(HAL_DMA_UnRegisterCallback(&h,HAL_DMA_XFER_CPLT_CB_ID)==HAL_OK);
    CHECK(h.XferCpltCallback==nullptr);CHECK(h.XferHalfCpltCallback==halfway);
}
TEST_CASE_FIXTURE(Dma,"DMA busy callback registration cannot replace an active completion callback") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);REQUIRE(HAL_DMA_Start_IT(&h,0x20000100,0x4001300c,2)==HAL_OK);
    CHECK(HAL_DMA_RegisterCallback(&h,HAL_DMA_XFER_CPLT_CB_ID,failed)==HAL_BUSY);
    CHECK(h.XferCpltCallback==done);CHECK(HAL_DMA_GetState(&h)==HAL_DMA_STATE_BUSY);
}
TEST_CASE_FIXTURE(Dma,"DMA polling rejects circular mode without declaring completion") {
    h.Init.Mode=DMA_CIRCULAR;REQUIRE(HAL_DMA_Init(&h)==HAL_OK);
    REQUIRE(HAL_DMA_Start(&h,0x20000100,0x4001300c,2)==HAL_OK);
    CHECK(HAL_DMA_PollForTransfer(&h,HAL_DMA_FULL_TRANSFER,2)==HAL_ERROR);
    CHECK((h.ErrorCode&HAL_DMA_ERROR_NOT_SUPPORTED)!=0);
}
TEST_CASE_FIXTURE(Dma,"DMA polling a half event leaves the transfer busy for its final event") {
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);REQUIRE(HAL_DMA_Start(&h,0x20000100,0x4001300c,2)==HAL_OK);
    DMA1->LISR=DMA_FLAG_HTIF0_4;CHECK(HAL_DMA_PollForTransfer(&h,HAL_DMA_HALF_TRANSFER,2)==HAL_OK);
    CHECK(h.State==HAL_DMA_STATE_BUSY);native::flush();CHECK((DMA1->LISR&DMA_FLAG_HTIF0_4)==0);
}
TEST_CASE_FIXTURE(Dma,"DMA FIFO and direct-mode interrupt errors reach the error callback") {
    for(auto flag:{DMA_FLAG_FEIF0_4,DMA_FLAG_DMEIF0_4}) {
        CAPTURE(flag);native::reset();h.Instance=DMA1_Stream0;error=0;
        REQUIRE(HAL_DMA_Init(&h)==HAL_OK);REQUIRE(HAL_DMA_Start_IT(&h,0x20000100,0x4001300c,2)==HAL_OK);
        h.Instance->FCR=h.Instance->FCR|DMA_SxFCR_FEIE;DMA1->LISR=flag;HAL_DMA_IRQHandler(&h);native::flush();
        CHECK(error==1);CHECK((h.ErrorCode&(flag==DMA_FLAG_FEIF0_4?HAL_DMA_ERROR_FE:HAL_DMA_ERROR_DME))!=0);
        CHECK((DMA1->LISR&flag)==0);
    }
}
TEST_CASE_FIXTURE(Dma,"DMA circular completion preserves the enabled stream and reloads its transfer count") {
    h.Init.Direction=DMA_PERIPH_TO_MEMORY;h.Init.Mode=DMA_CIRCULAR;
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK);uint16_t target[2]{};TIM2->CCR1=0x123;
    REQUIRE(HAL_DMA_Start_IT(&h,reinterpret_cast<uint32_t>(&TIM2->CCR1),reinterpret_cast<uint32_t>(target),2)==HAL_OK);
    native::completeDMA(h);HAL_DMA_IRQHandler(&h);CHECK(complete==1);CHECK(h.State==HAL_DMA_STATE_BUSY);
    CHECK(h.Instance->NDTR==2);CHECK((h.Instance->CR&DMA_SxCR_EN)!=0);CHECK(target[0]==0x123);CHECK(target[1]==0x123);
}
}
