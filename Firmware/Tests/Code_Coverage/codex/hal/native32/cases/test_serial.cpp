#include "native.h"
#include "TestHelpers.h"
namespace {
unsigned spiTx, spiRx, spiError, uartTx, uartRx, uartError;
unsigned i2cTx, i2cRx, i2cError, memTx, memRx;
struct Spi : native::Fixture {
    SPI_HandleTypeDef h{};
    Spi() {
        spiTx=spiRx=spiError=0; h.Instance=SPI1; SPI1->SR=SPI_SR_TXE;
        h.Init.Mode=SPI_MODE_MASTER; h.Init.Direction=SPI_DIRECTION_2LINES;
        h.Init.DataSize=SPI_DATASIZE_8BIT; h.Init.CLKPolarity=SPI_POLARITY_HIGH;
        h.Init.CLKPhase=SPI_PHASE_2EDGE; h.Init.NSS=SPI_NSS_SOFT;
        h.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;
        h.Init.FirstBit=SPI_FIRSTBIT_MSB; h.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
        h.Init.CRCPolynomial=7;
    }
};
struct Uart : native::Fixture {
    UART_HandleTypeDef h{};
    Uart() {
        uartTx=uartRx=uartError=0; h.Instance=USART2; USART2->SR=USART_SR_TXE|USART_SR_TC;
        h.Init.BaudRate=1000000; h.Init.WordLength=UART_WORDLENGTH_8B;
        h.Init.StopBits=UART_STOPBITS_1; h.Init.Parity=UART_PARITY_NONE;
        h.Init.Mode=UART_MODE_TX_RX; h.Init.HwFlowCtl=UART_HWCONTROL_NONE;
        h.Init.OverSampling=UART_OVERSAMPLING_16;
    }
};
struct I2c : native::Fixture {
    I2C_HandleTypeDef h{};
    I2c() {
        i2cTx=i2cRx=i2cError=memTx=memRx=0;
        h.Instance=I2C1; h.Init.ClockSpeed=100000; h.Init.DutyCycle=I2C_DUTYCYCLE_2;
        h.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT; h.Init.OwnAddress1=0x52;
        h.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE; h.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
        h.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    }
};
}
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef*) { ++spiTx; }
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef*) { ++spiRx; }
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef*) { ++spiError; }
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef*) { ++uartTx; }
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef*) { ++uartRx; }
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*) { ++uartError; }
extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef*) { ++i2cTx; }
extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef*) { ++i2cRx; }
extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef*) { ++i2cError; }
TEST_SUITE("P12NativeSPI") {
TEST_CASE_FIXTURE(Spi,"SPI initialization requests the selected clock mode word size and baud divider") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK);
    CHECK((SPI1->CR1&SPI_CR1_BR)==SPI_BAUDRATEPRESCALER_16);
    CHECK((SPI1->CR1&(SPI_CR1_CPOL|SPI_CR1_CPHA))==(SPI_CR1_CPOL|SPI_CR1_CPHA));
    CHECK((SPI1->CR1&SPI_CR1_MSTR)!=0); CHECK(h.State==HAL_SPI_STATE_READY);
}
TEST_CASE_FIXTURE(Spi,"SPI blocking transmission writes all requested bytes and releases its state") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); SPI1->SR=SPI_SR_TXE;
    uint8_t data[]={0x12,0x34,0x9a};
    CHECK(HAL_SPI_Transmit(&h,data,3,5)==HAL_OK);
    CHECK((SPI1->DR&255)==0x9a); CHECK(h.TxXferCount==0);
    CHECK(h.State==HAL_SPI_STATE_READY); CHECK(h.Lock==HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(Spi,"SPI missing transmit-ready flag returns timeout and releases the handle") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); SPI1->SR=0; uint8_t data[]={1,2};
    CHECK(HAL_SPI_Transmit(&h,data,2,2)!=HAL_OK);
    CHECK(h.State==HAL_SPI_STATE_READY); CHECK(h.Lock==HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(Spi,"SPI receive-only transfer delivers the explicitly supplied data word") {
    h.Init.Mode=SPI_MODE_SLAVE; h.Init.Direction=SPI_DIRECTION_2LINES_RXONLY;
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); SPI1->DR=0x6d; SPI1->SR=SPI_SR_RXNE;
    uint8_t data=0;
    CHECK(HAL_SPI_Receive(&h,&data,1,2)==HAL_OK); CHECK(data==0x6d);
}
TEST_CASE_FIXTURE(Spi,"SPI null or empty transmit requests are rejected without starting") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); uint8_t data=1;
    CHECK(HAL_SPI_Transmit(&h,nullptr,1,0)==HAL_ERROR);
    CHECK(HAL_SPI_Transmit(&h,&data,0,0)==HAL_ERROR);
    CHECK(h.State==HAL_SPI_STATE_READY);
}
TEST_CASE_FIXTURE(Spi,"SPI interrupt transmission writes its payload and invokes completion once") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); uint8_t data=0xa7;
    REQUIRE(HAL_SPI_Transmit_IT(&h,&data,1)==HAL_OK);
    SPI1->SR=SPI_SR_TXE; HAL_SPI_IRQHandler(&h);
    CHECK((SPI1->DR&255)==0xa7); CHECK(spiTx==1); CHECK(h.State==HAL_SPI_STATE_READY);
}
TEST_CASE_FIXTURE(Spi,"SPI mode-fault interrupt reports the error through its callback") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK);
    SPI1->CR2=SPI_CR2_ERRIE; SPI1->SR=SPI_SR_MODF;
    HAL_SPI_IRQHandler(&h);
    CHECK(spiError==1); CHECK((h.ErrorCode&HAL_SPI_ERROR_MODF)!=0);
}
TEST_CASE_FIXTURE(Spi,"SPI DMA start uses the real DMA endpoint and enables peripheral requests") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK);
    DMA_HandleTypeDef dma{}; dma.Instance=DMA1_Stream3; dma.Init.Direction=DMA_MEMORY_TO_PERIPH;
    REQUIRE(HAL_DMA_Init(&dma)==HAL_OK); h.hdmatx=&dma; dma.Parent=&h;
    uint8_t data[]={0x4a,0x5b};
    REQUIRE(HAL_SPI_Transmit_DMA(&h,data,2)==HAL_OK);
    CHECK(dma.Instance->PAR==reinterpret_cast<uint32_t>(&SPI1->DR));
    CHECK(dma.Instance->M0AR==reinterpret_cast<uint32_t>(data));
    CHECK((SPI1->CR2&SPI_CR2_TXDMAEN)!=0);
}
TEST_CASE_FIXTURE(Spi,"SPI deinitialization disables the peripheral and resets the public state") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK);
    CHECK(HAL_SPI_DeInit(&h)==HAL_OK); CHECK(h.State==HAL_SPI_STATE_RESET);
    CHECK((SPI1->CR1&SPI_CR1_SPE)==0);
}
TEST_CASE_FIXTURE(Spi,"SPI full duplex polling preserves distinct received bytes and transmitted requests") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); SPI1->SR=SPI_SR_TXE;
    native::spiInput(0,{0x6d,0xa7}); uint8_t tx[]={0x11,0x22},rx[2]{};
    REQUIRE(HAL_SPI_TransmitReceive(&h,tx,rx,2,10)==HAL_OK);
    CHECK(rx[0]==0x6d); CHECK(rx[1]==0xa7); CHECK(h.State==HAL_SPI_STATE_READY);
}
TEST_CASE_FIXTURE(Spi,"SPI sixteen bit transmit retains full word width") {
    h.Init.DataSize=SPI_DATASIZE_16BIT; REQUIRE(HAL_SPI_Init(&h)==HAL_OK); SPI1->SR=SPI_SR_TXE;
    uint16_t tx[]={0xabcd,0x7654};
    REQUIRE(HAL_SPI_Transmit(&h,reinterpret_cast<uint8_t*>(tx),2,10)==HAL_OK); CHECK(SPI1->DR==0x7654);
}
TEST_CASE_FIXTURE(Spi,"SPI sixteen bit receive delivers distinct full-width words") {
    h.Init.Mode=SPI_MODE_SLAVE; h.Init.Direction=SPI_DIRECTION_2LINES_RXONLY; h.Init.DataSize=SPI_DATASIZE_16BIT;
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); native::spiInput(0,{0xabcd,0x7654}); uint16_t rx[2]{};
    REQUIRE(HAL_SPI_Receive(&h,reinterpret_cast<uint8_t*>(rx),2,10)==HAL_OK);
    CHECK(rx[0]==0xabcd); CHECK(rx[1]==0x7654);
}
TEST_CASE_FIXTURE(Spi,"SPI receive interrupt delivers the supplied byte and releases ready state") {
    h.Init.Mode=SPI_MODE_SLAVE; h.Init.Direction=SPI_DIRECTION_2LINES_RXONLY;
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); uint8_t rx=0;
    REQUIRE(HAL_SPI_Receive_IT(&h,&rx,1)==HAL_OK); native::spiInput(0,{0xd3});
    HAL_SPI_IRQHandler(&h); CHECK(rx==0xd3); CHECK(spiRx==1); CHECK(h.State==HAL_SPI_STATE_READY);
}
TEST_CASE_FIXTURE(Spi,"SPI transmit DMA completion invokes its caller and releases peripheral requests") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); DMA_HandleTypeDef dma{};
    dma.Instance=DMA1_Stream3; dma.Init.Direction=DMA_MEMORY_TO_PERIPH; dma.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&dma)==HAL_OK); dma.Parent=&h; h.hdmatx=&dma;
    uint8_t tx[]={0x3d,0x94}; REQUIRE(HAL_SPI_Transmit_DMA(&h,tx,2)==HAL_OK);
    native::completeDMA(dma); HAL_DMA_IRQHandler(&dma);
    CHECK(spiTx==1); CHECK(h.State==HAL_SPI_STATE_READY); CHECK((SPI1->CR2&SPI_CR2_TXDMAEN)==0);
}
TEST_CASE_FIXTURE(Spi,"SPI software busy state rejects a second interrupt operation") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); h.State=HAL_SPI_STATE_BUSY_TX; uint8_t tx=1;
    CHECK(HAL_SPI_Transmit_IT(&h,&tx,1)==HAL_BUSY); CHECK(h.State==HAL_SPI_STATE_BUSY_TX);
}
}
TEST_SUITE("P12NativeUART") {
TEST_CASE_FIXTURE(Uart,"UART initialization computes an exact sixteen-clock baud divisor") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    CHECK(USART2->BRR==16); CHECK((USART2->CR1&(USART_CR1_UE|USART_CR1_TE|USART_CR1_RE))==(USART_CR1_UE|USART_CR1_TE|USART_CR1_RE));
}
TEST_CASE_FIXTURE(Uart,"UART blocking transmit writes the final byte and returns ready") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); USART2->SR=USART_SR_TXE|USART_SR_TC;
    uint8_t data[]={0x35,0x79};
    CHECK(HAL_UART_Transmit(&h,data,2,5)==HAL_OK); CHECK((USART2->DR&255)==0x79);
    CHECK(h.gState==HAL_UART_STATE_READY); CHECK(h.Lock==HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(Uart,"UART blocking receive delivers an explicit receive register byte") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); USART2->SR=USART_SR_RXNE; USART2->DR=0xc3;
    uint8_t data=0;
    CHECK(HAL_UART_Receive(&h,&data,1,5)==HAL_OK); CHECK(data==0xc3);
}
TEST_CASE_FIXTURE(Uart,"UART blocking transfer times out without transmit-ready input") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); USART2->SR=0; uint8_t data=1;
    CHECK(HAL_UART_Transmit(&h,&data,1,2)==HAL_TIMEOUT); CHECK(h.gState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART interrupt transmit separates data-ready from transfer-complete callbacks") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); uint8_t data=0x69;
    REQUIRE(HAL_UART_Transmit_IT(&h,&data,1)==HAL_OK);
    USART2->SR=USART_SR_TXE; HAL_UART_IRQHandler(&h);
    CHECK((USART2->DR&255)==0x69); CHECK(uartTx==0);
    USART2->SR=USART_SR_TC; HAL_UART_IRQHandler(&h);
    CHECK(uartTx==1); CHECK(h.gState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART interrupt receive delivers the byte before its completion callback") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); uint8_t data=0;
    REQUIRE(HAL_UART_Receive_IT(&h,&data,1)==HAL_OK);
    USART2->DR=0x8e; USART2->SR=USART_SR_RXNE; HAL_UART_IRQHandler(&h);
    CHECK(data==0x8e); CHECK(uartRx==1); CHECK(h.RxState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART null transmit buffer is rejected without changing ready state") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    CHECK(HAL_UART_Transmit(&h,nullptr,1,0)==HAL_ERROR); CHECK(h.gState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART DMA transmission programs the real DMA buffer and request bit") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    DMA_HandleTypeDef dma{}; dma.Instance=DMA1_Stream6; dma.Init.Direction=DMA_MEMORY_TO_PERIPH;
    REQUIRE(HAL_DMA_Init(&dma)==HAL_OK); h.hdmatx=&dma; dma.Parent=&h;
    uint8_t data[]={0x12,0x34};
    REQUIRE(HAL_UART_Transmit_DMA(&h,data,2)==HAL_OK);
    CHECK(dma.Instance->PAR==reinterpret_cast<uint32_t>(&USART2->DR));
    CHECK(dma.Instance->M0AR==reinterpret_cast<uint32_t>(data));
    CHECK((USART2->CR3&USART_CR3_DMAT)!=0);
}
TEST_CASE_FIXTURE(Uart,"UART abort restores independent transmit and receive ready states") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    h.gState=HAL_UART_STATE_BUSY_TX; h.RxState=HAL_UART_STATE_BUSY_RX;
    CHECK(HAL_UART_Abort(&h)==HAL_OK); CHECK(h.gState==HAL_UART_STATE_READY); CHECK(h.RxState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART deinitialization disables the peripheral") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    CHECK(HAL_UART_DeInit(&h)==HAL_OK); CHECK((USART2->CR1&USART_CR1_UE)==0);
}
TEST_CASE_FIXTURE(Uart,"UART receive DMA completion delivers the supplied data and callback") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); DMA_HandleTypeDef dma{};
    dma.Instance=DMA1_Stream1; dma.Init.Direction=DMA_PERIPH_TO_MEMORY; dma.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&dma)==HAL_OK); dma.Parent=&h; h.hdmarx=&dma;
    uint8_t rx=0; REQUIRE(HAL_UART_Receive_DMA(&h,&rx,1)==HAL_OK); USART2->DR=0xe7; USART2->SR=USART_SR_RXNE;
    native::completeDMA(dma); HAL_DMA_IRQHandler(&dma);
    CHECK(rx==0xe7); CHECK(uartRx==1); CHECK(h.RxState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART transmitter completion follows real DMA completion and final TC event") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); DMA_HandleTypeDef dma{};
    dma.Instance=DMA1_Stream6; dma.Init.Direction=DMA_MEMORY_TO_PERIPH; dma.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&dma)==HAL_OK); dma.Parent=&h; h.hdmatx=&dma;
    uint8_t tx[]={0x13,0x57}; REQUIRE(HAL_UART_Transmit_DMA(&h,tx,2)==HAL_OK);
    native::completeDMA(dma); HAL_DMA_IRQHandler(&dma); CHECK(uartTx==0);
    USART2->SR=USART_SR_TC; HAL_UART_IRQHandler(&h); CHECK(uartTx==1);
}
TEST_CASE_FIXTURE(Uart,"UART missing receive data returns timeout without changing the caller byte") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); uint8_t rx=0x5a;
    CHECK(HAL_UART_Receive(&h,&rx,1,2)==HAL_TIMEOUT); CHECK(rx==0x5a); CHECK(h.RxState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART nine-bit receive preserves the complete data field") {
    h.Init.WordLength=UART_WORDLENGTH_9B; REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    USART2->SR=USART_SR_RXNE; USART2->DR=0x1a7; uint16_t rx=0;
    REQUIRE(HAL_UART_Receive(&h,reinterpret_cast<uint8_t*>(&rx),1,5)==HAL_OK); CHECK(rx==0x1a7);
}
TEST_CASE_FIXTURE(Uart,"UART half-duplex direction changes preserve peripheral enable") {
    REQUIRE(HAL_HalfDuplex_Init(&h)==HAL_OK); CHECK((USART2->CR3&USART_CR3_HDSEL)!=0);
    CHECK(HAL_HalfDuplex_EnableTransmitter(&h)==HAL_OK); CHECK((USART2->CR1&USART_CR1_TE)!=0);
    CHECK(HAL_HalfDuplex_EnableReceiver(&h)==HAL_OK); CHECK((USART2->CR1&USART_CR1_RE)!=0);
}
}
TEST_SUITE("P12NativeI2C") {
TEST_CASE_FIXTURE(I2c,"I2C standard-mode initialization derives clock period and rise time") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);
    CHECK((I2C1->CR2&I2C_CR2_FREQ)==16); CHECK(I2C1->CCR==80); CHECK(I2C1->TRISE==17);
    CHECK((I2C1->CR1&I2C_CR1_PE)!=0); CHECK(h.State==HAL_I2C_STATE_READY);
}
TEST_CASE_FIXTURE(I2c,"I2C interrupt start preserves the exact payload and requests START") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); uint8_t data[]={0x11,0x22};
    REQUIRE(HAL_I2C_Master_Transmit_IT(&h,0x52,data,2)==HAL_OK);
    CHECK((I2C1->CR1&I2C_CR1_START)!=0); CHECK(h.pBuffPtr==data); CHECK(h.XferCount==2);
}
TEST_CASE_FIXTURE(I2c,"I2C start-bit event emits the requested write address") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); uint8_t data=0x33;
    REQUIRE(HAL_I2C_Master_Transmit_IT(&h,0x52,&data,1)==HAL_OK);
    I2C1->SR1=I2C_SR1_SB; I2C1->SR2=I2C_SR2_MSL|I2C_SR2_TRA;
    HAL_I2C_EV_IRQHandler(&h); CHECK((I2C1->DR&255)==0x52);
}
TEST_CASE_FIXTURE(I2c,"I2C bus-busy timeout returns without starting a new transfer") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); I2C1->SR2=I2C_SR2_BUSY; uint8_t data=0;
    CHECK(HAL_I2C_Master_Transmit(&h,0x52,&data,1,2)!=HAL_OK);
    CHECK((I2C1->CR1&I2C_CR1_START)==0);
}
#if defined(I2C_FLTR_ANOFF) && defined(I2C_FLTR_DNF)
TEST_CASE_FIXTURE(I2c,"I2C analog filter configuration changes its requested bit without losing ready state") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);
    CHECK(HAL_I2CEx_ConfigAnalogFilter(&h,I2C_ANALOGFILTER_DISABLE)==HAL_OK);
    CHECK((I2C1->FLTR&I2C_FLTR_ANOFF)!=0); CHECK(h.State==HAL_I2C_STATE_READY);
    CHECK(HAL_I2CEx_ConfigAnalogFilter(&h,I2C_ANALOGFILTER_ENABLE)==HAL_OK);
    CHECK((I2C1->FLTR&I2C_FLTR_ANOFF)==0);
}
TEST_CASE_FIXTURE(I2c,"I2C digital filter uses the requested nondefault cycle count") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);
    CHECK(HAL_I2CEx_ConfigDigitalFilter(&h,9)==HAL_OK); CHECK((I2C1->FLTR&I2C_FLTR_DNF)==9);
}
#endif
TEST_CASE_FIXTURE(I2c,"I2C error interrupt records a bus error") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); I2C1->CR2|=I2C_CR2_ITERREN; I2C1->SR1=I2C_SR1_BERR;
    HAL_I2C_ER_IRQHandler(&h); CHECK((HAL_I2C_GetError(&h)&HAL_I2C_ERROR_BERR)!=0);
}
TEST_CASE_FIXTURE(I2c,"I2C deinitialization releases the enabled peripheral") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); CHECK(HAL_I2C_DeInit(&h)==HAL_OK);
    CHECK((I2C1->CR1&I2C_CR1_PE)==0); CHECK(h.State==HAL_I2C_STATE_RESET);
}
TEST_CASE_FIXTURE(I2c,"I2C blocking master write emits the address and every distinct payload byte") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); native::i2cPeer(0,0x29);
    uint8_t tx[]={0x12,0x34,0x56,0x78};
    REQUIRE(HAL_I2C_Master_Transmit(&h,0x52,tx,4,10)==HAL_OK);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x12,0x34,0x56,0x78});
    CHECK(h.State==HAL_I2C_STATE_READY); CHECK((I2C1->CR1&I2C_CR1_STOP)!=0);
}
TEST_CASE_FIXTURE(I2c,"I2C blocking master read handles one two three and longer byte tails") {
    for(unsigned length : {1U,2U,3U,5U}) {
        CAPTURE(length); native::reset(); h={}; h.Instance=I2C1; h.Init.ClockSpeed=100000;
        h.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT; h.Init.DutyCycle=I2C_DUTYCYCLE_2;
        REQUIRE(HAL_I2C_Init(&h)==HAL_OK);
        std::vector<uint8_t> expected; for(unsigned i=0;i<length;++i) expected.push_back(0x30+i*7);
        native::i2cPeer(0,0x29,expected); uint8_t guard[8]; std::fill(std::begin(guard),std::end(guard),0xa5);
        REQUIRE(HAL_I2C_Master_Receive(&h,0x52,guard+1,length,10)==HAL_OK);
        for(unsigned i=0;i<length;++i) CHECK(guard[1+i]==expected[i]);
        CHECK(guard[0]==0xa5); CHECK(guard[length+1]==0xa5);
    }
}
TEST_CASE_FIXTURE(I2c,"I2C memory write emits a sixteen-bit address most significant byte first") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); native::i2cPeer(0,0x29); uint8_t tx[]={0x9a,0xbc};
    REQUIRE(HAL_I2C_Mem_Write(&h,0x52,0x1234,I2C_MEMADD_SIZE_16BIT,tx,2,10)==HAL_OK);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x12,0x34,0x9a,0xbc});
}
TEST_CASE_FIXTURE(I2c,"I2C memory read repeats the address phase in the read direction") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); native::i2cPeer(0,0x29,{0xe1,0x37}); uint8_t rx[2]{};
    REQUIRE(HAL_I2C_Mem_Read(&h,0x52,0x1234,I2C_MEMADD_SIZE_16BIT,rx,2,10)==HAL_OK);
    CHECK(rx[0]==0xe1); CHECK(rx[1]==0x37);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x12,0x34,0x53});
}
TEST_CASE_FIXTURE(I2c,"I2C missing peer acknowledgement returns an error without consuming the caller data") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); native::i2cPeer(0,0x2a); uint8_t tx=0x91;
    CHECK(HAL_I2C_Master_Transmit(&h,0x52,&tx,1,10)==HAL_ERROR); CHECK(tx==0x91);
    CHECK((h.ErrorCode&HAL_I2C_ERROR_AF)!=0);
}
TEST_CASE_FIXTURE(I2c,"I2C interrupt master write reaches completion through controlled peripheral events") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); native::i2cPeer(0,0x29); uint8_t tx[]={0xa1,0xb2,0xc3};
    REQUIRE(HAL_I2C_Master_Transmit_IT(&h,0x52,tx,3)==HAL_OK);
    for(unsigned n=0;n<16&&h.State!=HAL_I2C_STATE_READY;++n) HAL_I2C_EV_IRQHandler(&h);
    CHECK(h.State==HAL_I2C_STATE_READY); CHECK(i2cTx==1);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0xa1,0xb2,0xc3});
}
TEST_CASE_FIXTURE(I2c,"I2C interrupt master receive delivers distinct bytes before completion") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); native::i2cPeer(0,0x29,{0x3c,0x7e,0x91}); uint8_t rx[3]{};
    REQUIRE(HAL_I2C_Master_Receive_IT(&h,0x52,rx,3)==HAL_OK);
    for(unsigned n=0;n<16&&h.State!=HAL_I2C_STATE_READY;++n) HAL_I2C_EV_IRQHandler(&h);
    CHECK(h.State==HAL_I2C_STATE_READY); CHECK(i2cRx==1);
    CHECK(rx[0]==0x3c); CHECK(rx[1]==0x7e); CHECK(rx[2]==0x91);
}
}

TEST_SUITE("P12NativeSerialLifecycle") {
TEST_CASE_FIXTURE(Uart,"UART DMA pause resume and stop preserve the transfer lifecycle") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream6; d.Init.Direction=DMA_MEMORY_TO_PERIPH;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK); d.Parent=&h; h.hdmatx=&d;
    uint8_t tx[]={3,7}; REQUIRE(HAL_UART_Transmit_DMA(&h,tx,2)==HAL_OK);
    REQUIRE(HAL_UART_DMAPause(&h)==HAL_OK); CHECK((USART2->CR3&USART_CR3_DMAT)==0);
    CHECK(h.gState==HAL_UART_STATE_BUSY_TX);
    REQUIRE(HAL_UART_DMAResume(&h)==HAL_OK); CHECK((USART2->CR3&USART_CR3_DMAT)!=0);
    REQUIRE(HAL_UART_DMAStop(&h)==HAL_OK); CHECK(h.gState==HAL_UART_STATE_READY);
    CHECK((d.Instance->CR&DMA_SxCR_EN)==0); CHECK((USART2->CR3&USART_CR3_DMAT)==0);
}
TEST_CASE_FIXTURE(Uart,"UART transmit abort leaves an independent receiver active") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); uint8_t tx=2,rx=0;
    REQUIRE(HAL_UART_Transmit_IT(&h,&tx,1)==HAL_OK); REQUIRE(HAL_UART_Receive_IT(&h,&rx,1)==HAL_OK);
    REQUIRE(HAL_UART_AbortTransmit(&h)==HAL_OK);
    CHECK(h.gState==HAL_UART_STATE_READY); CHECK(h.RxState==HAL_UART_STATE_BUSY_RX);
    USART2->DR=0x75; USART2->SR=USART_SR_RXNE; HAL_UART_IRQHandler(&h); CHECK(rx==0x75);
}
TEST_CASE_FIXTURE(Uart,"UART receiver abort leaves an independent transmitter active") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); uint8_t tx=2,rx=0;
    REQUIRE(HAL_UART_Transmit_IT(&h,&tx,1)==HAL_OK); REQUIRE(HAL_UART_Receive_IT(&h,&rx,1)==HAL_OK);
    REQUIRE(HAL_UART_AbortReceive(&h)==HAL_OK);
    CHECK(h.RxState==HAL_UART_STATE_READY); CHECK(h.gState==HAL_UART_STATE_BUSY_TX);
    CHECK((USART2->CR1&USART_CR1_RXNEIE)==0); CHECK((USART2->CR1&USART_CR1_TXEIE)!=0);
}
TEST_CASE_FIXTURE(Uart,"UART overrun terminates reception and reports its blocking error") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK); uint8_t rx=0;
    REQUIRE(HAL_UART_Receive_IT(&h,&rx,1)==HAL_OK); USART2->SR=USART_SR_ORE;
    HAL_UART_IRQHandler(&h); CHECK(uartError==1); CHECK((h.ErrorCode&HAL_UART_ERROR_ORE)!=0);
    CHECK(h.RxState==HAL_UART_STATE_READY); CHECK((USART2->CR1&USART_CR1_RXNEIE)==0);
}
TEST_CASE_FIXTURE(Uart,"UART eight-clock oversampling derives the independently calculated baud divisor") {
    h.Init.OverSampling=UART_OVERSAMPLING_8; REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    CHECK(USART2->BRR==0x20); CHECK((USART2->CR1&USART_CR1_OVER8)!=0);
}
TEST_CASE_FIXTURE(Uart,"UART LIN configuration selects a long break and emits a break request") {
    REQUIRE(HAL_LIN_Init(&h,UART_LINBREAKDETECTLENGTH_11B)==HAL_OK);
    CHECK((USART2->CR2&(USART_CR2_LINEN|USART_CR2_LBDL))==(USART_CR2_LINEN|USART_CR2_LBDL));
    REQUIRE(HAL_LIN_SendBreak(&h)==HAL_OK); CHECK((USART2->CR1&USART_CR1_SBK)!=0);
}
TEST_CASE_FIXTURE(Uart,"UART multiprocessor mode retains its address while requesting mute") {
    REQUIRE(HAL_MultiProcessor_Init(&h,9,UART_WAKEUPMETHOD_ADDRESSMARK)==HAL_OK);
    CHECK((USART2->CR2&USART_CR2_ADD)==9); CHECK((USART2->CR1&USART_CR1_WAKE)!=0);
    REQUIRE(HAL_MultiProcessor_EnterMuteMode(&h)==HAL_OK); CHECK((USART2->CR1&USART_CR1_RWU)!=0);
    REQUIRE(HAL_MultiProcessor_ExitMuteMode(&h)==HAL_OK); CHECK((USART2->CR1&USART_CR1_RWU)==0);
}
TEST_CASE_FIXTURE(Spi,"SPI DMA pause and resume retain active state until explicit stop") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream3; d.Init.Direction=DMA_MEMORY_TO_PERIPH;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK); d.Parent=&h; h.hdmatx=&d; uint8_t tx[]={1,2};
    REQUIRE(HAL_SPI_Transmit_DMA(&h,tx,2)==HAL_OK);
    REQUIRE(HAL_SPI_DMAPause(&h)==HAL_OK); CHECK((SPI1->CR2&SPI_CR2_TXDMAEN)==0);
    REQUIRE(HAL_SPI_DMAResume(&h)==HAL_OK); CHECK((SPI1->CR2&SPI_CR2_TXDMAEN)!=0);
    REQUIRE(HAL_SPI_DMAStop(&h)==HAL_OK); CHECK(h.State==HAL_SPI_STATE_READY);
    CHECK((d.Instance->CR&DMA_SxCR_EN)==0);
}
TEST_CASE_FIXTURE(Spi,"SPI blocking abort reports an unserviced interrupt and releases its handle") {
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK); uint8_t tx[]={1,2};
    REQUIRE(HAL_SPI_Transmit_IT(&h,tx,2)==HAL_OK); REQUIRE(HAL_SPI_Abort(&h)==HAL_ERROR);
    CHECK((h.ErrorCode&HAL_SPI_ERROR_ABORT)!=0);
    CHECK(h.State==HAL_SPI_STATE_READY); CHECK((SPI1->CR2&SPI_CR2_ERRIE)==0);
}
TEST_CASE_FIXTURE(I2c,"I2C device probe distinguishes an acknowledging peer from a missing address") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); native::i2cPeer(0,0x29);
    CHECK(HAL_I2C_IsDeviceReady(&h,0x52,2,10)==HAL_OK); CHECK(h.State==HAL_I2C_STATE_READY);
    native::i2cPeer(0,0x2a); CHECK(HAL_I2C_IsDeviceReady(&h,0x52,2,10)!=HAL_OK);
}
TEST_CASE_FIXTURE(I2c,"I2C listen enable and disable change the address interrupt and public state") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); REQUIRE(HAL_I2C_EnableListen_IT(&h)==HAL_OK);
    CHECK(h.State==HAL_I2C_STATE_LISTEN); CHECK((I2C1->CR1&I2C_CR1_ACK)!=0);
    CHECK((I2C1->CR2&I2C_CR2_ITEVTEN)!=0); CHECK(HAL_I2C_EnableListen_IT(&h)==HAL_BUSY);
    REQUIRE(HAL_I2C_DisableListen_IT(&h)==HAL_OK); CHECK(h.State==HAL_I2C_STATE_READY);
    CHECK((I2C1->CR1&I2C_CR1_ACK)==0);
}
TEST_CASE_FIXTURE(I2c,"I2C fast-mode initialization derives the selected sixteen-to-nine duty cycle") {
    h.Init.ClockSpeed=400000; h.Init.DutyCycle=I2C_DUTYCYCLE_16_9;
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); CHECK(I2C1->CCR==(I2C_CCR_FS|I2C_CCR_DUTY|2)); CHECK(I2C1->TRISE==5);
}
TEST_CASE_FIXTURE(I2c,"I2C arbitration-loss interrupt clears the pending error and releases a master") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK); uint8_t tx=1;
    REQUIRE(HAL_I2C_Master_Transmit_IT(&h,0x52,&tx,1)==HAL_OK);
    I2C1->SR1=I2C_SR1_ARLO; HAL_I2C_ER_IRQHandler(&h);
    CHECK(i2cError==1); CHECK((h.ErrorCode&HAL_I2C_ERROR_ARLO)!=0);
    CHECK((I2C1->SR1&I2C_SR1_ARLO)==0); CHECK(h.State==HAL_I2C_STATE_READY);
}
}

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef*) { ++memTx; }
extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef*) { ++memRx; }
TEST_SUITE("P12NativeI2CTransfers") {
TEST_CASE_FIXTURE(I2c,"I2C memory interrupt write preserves address order through completion") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29);uint8_t tx[]={0x91,0x42};
    REQUIRE(HAL_I2C_Mem_Write_IT(&h,0x52,0x1234,I2C_MEMADD_SIZE_16BIT,tx,2)==HAL_OK);
    for(unsigned n=0;n<24&&h.State!=HAL_I2C_STATE_READY;++n) {
        // Service address TXE promptly, before the shift register also empties.
        if(h.EventCount<2) {native::flush();I2C1->SR1=I2C1->SR1&~I2C_SR1_BTF;}
        HAL_I2C_EV_IRQHandler(&h);
    }
    CHECK(h.State==HAL_I2C_STATE_READY);CHECK(memTx==1);CHECK(i2cTx==0);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x12,0x34,0x91,0x42});
}
TEST_CASE_FIXTURE(I2c,"I2C memory interrupt read preserves repeated address and distinct data") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29,{0x93,0x47,0x65});uint8_t rx[3]{};
    REQUIRE(HAL_I2C_Mem_Read_IT(&h,0x52,0xab,I2C_MEMADD_SIZE_8BIT,rx,3)==HAL_OK);
    for(unsigned n=0;n<24&&h.State!=HAL_I2C_STATE_READY;++n)HAL_I2C_EV_IRQHandler(&h);
    CHECK(h.State==HAL_I2C_STATE_READY);CHECK(memRx==1);CHECK(i2cRx==0);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0xab,0x53});
    CHECK(rx[0]==0x93);CHECK(rx[1]==0x47);CHECK(rx[2]==0x65);
}
TEST_CASE_FIXTURE(I2c,"I2C master DMA write completes after both DMA and peripheral completion events") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29);DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream6;d.Init.Direction=DMA_MEMORY_TO_PERIPH;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmatx=&d;uint8_t tx[]={0x13,0x57,0x9b};
    REQUIRE(HAL_I2C_Master_Transmit_DMA(&h,0x52,tx,3)==HAL_OK);
    HAL_I2C_EV_IRQHandler(&h);HAL_I2C_EV_IRQHandler(&h);native::completeDMA(d);HAL_DMA_IRQHandler(&d);
    CHECK(i2cTx==0);HAL_I2C_EV_IRQHandler(&h);
    CHECK(i2cTx==1);CHECK(h.State==HAL_I2C_STATE_READY);CHECK((I2C1->CR2&I2C_CR2_DMAEN)==0);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x13,0x57,0x9b});
}
TEST_CASE_FIXTURE(I2c,"I2C master DMA read completes with a guarded distinct payload") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29,{0x13,0x57,0x9b});DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmarx=&d;uint8_t rx[]={0xa5,0,0,0,0xa5};
    REQUIRE(HAL_I2C_Master_Receive_DMA(&h,0x52,rx+1,3)==HAL_OK);
    HAL_I2C_EV_IRQHandler(&h);HAL_I2C_EV_IRQHandler(&h);native::completeDMA(d);HAL_DMA_IRQHandler(&d);
    CHECK(i2cRx==1);CHECK(h.State==HAL_I2C_STATE_READY);CHECK((I2C1->CR2&I2C_CR2_DMAEN)==0);
    CHECK(std::vector<uint8_t>(rx,rx+5)==std::vector<uint8_t>{0xa5,0x13,0x57,0x9b,0xa5});
}
TEST_CASE_FIXTURE(I2c,"I2C memory DMA write emits its address before the DMA payload") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29);DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream6;d.Init.Direction=DMA_MEMORY_TO_PERIPH;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmatx=&d;uint8_t tx[]={0x13,0x57};
    REQUIRE(HAL_I2C_Mem_Write_DMA(&h,0x52,0x1234,I2C_MEMADD_SIZE_16BIT,tx,2)==HAL_OK);
    for(unsigned i=0;i<6;++i)HAL_I2C_EV_IRQHandler(&h);
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);HAL_I2C_EV_IRQHandler(&h);
    CHECK(memTx==1);CHECK(h.State==HAL_I2C_STATE_READY);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x12,0x34,0x13,0x57});
}
TEST_CASE_FIXTURE(I2c,"I2C memory DMA read delivers data after its repeated address phase") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29,{0x13,0x57});DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmarx=&d;uint8_t rx[2]{};
    REQUIRE(HAL_I2C_Mem_Read_DMA(&h,0x52,0x1234,I2C_MEMADD_SIZE_16BIT,rx,2)==HAL_OK);
    for(unsigned i=0;i<8;++i)HAL_I2C_EV_IRQHandler(&h);
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);
    CHECK(memRx==1);CHECK(h.State==HAL_I2C_STATE_READY);CHECK(rx[0]==0x13);CHECK(rx[1]==0x57);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x12,0x34,0x53});
}
TEST_CASE_FIXTURE(I2c,"I2C missing DMA linkage returns a parameter error and unlocks the handle") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);uint8_t tx=1;
    CHECK(HAL_I2C_Master_Transmit_DMA(&h,0x52,&tx,1)==HAL_ERROR);
    CHECK((h.ErrorCode&HAL_I2C_ERROR_DMA_PARAM)!=0);CHECK(h.Lock==HAL_UNLOCKED);CHECK(h.State==HAL_I2C_STATE_READY);
}
}

TEST_CASE_FIXTURE(I2c,"characterization: I2C memory interrupt write retains both address bytes when BTF precedes service"
    * KNOWN_ISSUE("P12-I2C-001: BTF during the sixteen-bit memory address phase bypasses the remaining address byte")) {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29);uint8_t tx[]={0x91,0x42};
    REQUIRE(HAL_I2C_Mem_Write_IT(&h,0x52,0x1234,I2C_MEMADD_SIZE_16BIT,tx,2)==HAL_OK);
    for(unsigned n=0;n<24&&h.State!=HAL_I2C_STATE_READY;++n)HAL_I2C_EV_IRQHandler(&h);
    REQUIRE(h.State==HAL_I2C_STATE_READY);REQUIRE(memTx==1);REQUIRE(i2cTx==0);
    const auto observed=native::i2cWritten(0);
    REQUIRE(observed.size()>=4);REQUIRE(observed[0]==0x52);REQUIRE(observed[1]==0x12);
    REQUIRE(observed[observed.size()-2]==0x91);REQUIRE(observed.back()==0x42);
    CHECK(observed==std::vector<uint8_t>{0x52,0x12,0x34,0x91,0x42});
}
namespace {
unsigned slaveTx,slaveRx,txrx;
}
extern "C" void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef*) { ++slaveTx; }
extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef*) { ++slaveRx; }
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef*) { ++txrx; }
TEST_SUITE("P12NativeSerialEventModes") {
TEST_CASE_FIXTURE(I2c,"I2C slave interrupt receive retains each byte and completes on STOP") {
    slaveRx=0;REQUIRE(HAL_I2C_Init(&h)==HAL_OK);uint8_t rx[2]{};
    REQUIRE(HAL_I2C_Slave_Receive_IT(&h,rx,2)==HAL_OK);
    I2C1->SR2=0;I2C1->SR1=I2C_SR1_ADDR;HAL_I2C_EV_IRQHandler(&h);native::flush();
    for(auto byte:{0x27,0x93}) {I2C1->DR=byte;I2C1->SR1=I2C_SR1_RXNE;HAL_I2C_EV_IRQHandler(&h);native::flush();}
    I2C1->SR1=I2C_SR1_STOPF;HAL_I2C_EV_IRQHandler(&h);native::flush();
    CHECK(rx[0]==0x27);CHECK(rx[1]==0x93);CHECK(slaveRx==1);CHECK(h.State==HAL_I2C_STATE_READY);
}
TEST_CASE_FIXTURE(I2c,"I2C slave interrupt transmit completes when the controller NACKs its final byte") {
    slaveTx=0;REQUIRE(HAL_I2C_Init(&h)==HAL_OK);uint8_t tx[]={0x27,0x93};
    REQUIRE(HAL_I2C_Slave_Transmit_IT(&h,tx,2)==HAL_OK);
    I2C1->SR2=I2C_SR2_TRA;I2C1->SR1=I2C_SR1_ADDR;HAL_I2C_EV_IRQHandler(&h);native::flush();
    for(auto byte:tx) {I2C1->SR1=I2C_SR1_TXE;HAL_I2C_EV_IRQHandler(&h);native::flush();CHECK(I2C1->DR==byte);}
    I2C1->SR1=I2C_SR1_AF;HAL_I2C_ER_IRQHandler(&h);native::flush();
    CHECK(slaveTx==1);CHECK(h.State==HAL_I2C_STATE_READY);CHECK(h.ErrorCode==HAL_I2C_ERROR_NONE);
}
TEST_CASE_FIXTURE(I2c,"I2C sequential transmit defers STOP until its last frame") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29);uint8_t first=0x27,last=0x93;
    REQUIRE(HAL_I2C_Master_Seq_Transmit_IT(&h,0x52,&first,1,I2C_FIRST_FRAME)==HAL_OK);
    for(unsigned n=0;n<12&&h.State!=HAL_I2C_STATE_READY;++n)HAL_I2C_EV_IRQHandler(&h);
    REQUIRE(h.State==HAL_I2C_STATE_READY);CHECK(i2cTx==1);CHECK((I2C1->CR1&I2C_CR1_STOP)==0);
    REQUIRE(HAL_I2C_Master_Seq_Transmit_IT(&h,0x52,&last,1,I2C_LAST_FRAME)==HAL_OK);
    for(unsigned n=0;n<12&&h.State!=HAL_I2C_STATE_READY;++n)HAL_I2C_EV_IRQHandler(&h);
    CHECK(h.State==HAL_I2C_STATE_READY);CHECK(i2cTx==2);native::flush();CHECK((I2C1->CR1&I2C_CR1_STOP)!=0);
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x27,0x93});
}
TEST_CASE_FIXTURE(I2c,"I2C sequential receive final frame delivers the complete distinct payload") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29,{0x27,0x93});uint8_t rx[2]{};
    REQUIRE(HAL_I2C_Master_Seq_Receive_IT(&h,0x52,rx,2,I2C_FIRST_AND_LAST_FRAME)==HAL_OK);
    for(unsigned n=0;n<12&&h.State!=HAL_I2C_STATE_READY;++n)HAL_I2C_EV_IRQHandler(&h);
    CHECK(h.State==HAL_I2C_STATE_READY);CHECK(i2cRx==1);CHECK(rx[0]==0x27);CHECK(rx[1]==0x93);
}
TEST_CASE_FIXTURE(Spi,"SPI full duplex interrupt transfer separates received bytes from its transmit buffer") {
    txrx=0;REQUIRE(HAL_SPI_Init(&h)==HAL_OK);uint8_t tx[]={0x31,0x72},rx[2]{};
    REQUIRE(HAL_SPI_TransmitReceive_IT(&h,tx,rx,2)==HAL_OK);native::spiInput(0,{0xa3,0xb7});
    for(unsigned n=0;n<12&&h.State!=HAL_SPI_STATE_READY;++n){native::advanceBus();HAL_SPI_IRQHandler(&h);}
    CHECK(h.State==HAL_SPI_STATE_READY);CHECK(txrx==1);CHECK(rx[0]==0xa3);CHECK(rx[1]==0xb7);
    CHECK(tx[0]==0x31);CHECK(tx[1]==0x72);
}
TEST_CASE_FIXTURE(Spi,"SPI receive DMA completes with the supplied distinct words") {
    h.Init.Mode=SPI_MODE_SLAVE;h.Init.Direction=SPI_DIRECTION_2LINES_RXONLY;
    REQUIRE(HAL_SPI_Init(&h)==HAL_OK);DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmarx=&d;uint8_t rx[2]{};
    REQUIRE(HAL_SPI_Receive_DMA(&h,rx,2)==HAL_OK);native::spiInput(0,{0x27,0x93});
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);CHECK(rx[0]==0x27);CHECK(rx[1]==0x93);
    CHECK(spiRx==1);CHECK(h.State==HAL_SPI_STATE_READY);CHECK((SPI1->CR2&SPI_CR2_RXDMAEN)==0);
}
TEST_CASE_FIXTURE(Uart,"UART parity mode masks the parity position from an eight-bit received word") {
    h.Init.Parity=UART_PARITY_EVEN;REQUIRE(HAL_UART_Init(&h)==HAL_OK);
    USART2->DR=0xe7;USART2->SR=USART_SR_RXNE;uint8_t rx=0;
    REQUIRE(HAL_UART_Receive(&h,&rx,1,5)==HAL_OK);CHECK(rx==0x67);
}
TEST_CASE_FIXTURE(Uart,"UART nine-bit interrupt receive advances by complete halfwords") {
    h.Init.WordLength=UART_WORDLENGTH_9B;REQUIRE(HAL_UART_Init(&h)==HAL_OK);uint16_t rx[2]{};
    REQUIRE(HAL_UART_Receive_IT(&h,reinterpret_cast<uint8_t*>(rx),2)==HAL_OK);
    for(auto word:{0x127,0x193}) {native::flush();USART2->DR=word;USART2->SR=USART_SR_RXNE;HAL_UART_IRQHandler(&h);}
    CHECK(rx[0]==0x127);CHECK(rx[1]==0x193);CHECK(uartRx==1);
}
TEST_CASE_FIXTURE(Uart,"UART nine-bit interrupt transmit reads complete halfwords from its caller") {
    h.Init.WordLength=UART_WORDLENGTH_9B;REQUIRE(HAL_UART_Init(&h)==HAL_OK);uint16_t tx[]={0x127,0x193};
    REQUIRE(HAL_UART_Transmit_IT(&h,reinterpret_cast<uint8_t*>(tx),2)==HAL_OK);
    for(auto word:tx) {native::flush();USART2->SR=USART_SR_TXE;HAL_UART_IRQHandler(&h);native::flush();CHECK(USART2->DR==word);}
    USART2->SR=USART_SR_TC;HAL_UART_IRQHandler(&h);CHECK(uartTx==1);CHECK(h.gState==HAL_UART_STATE_READY);
}
}
namespace {unsigned uartEventCount,uartEventSize,uartAborted;}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*,uint16_t size) {++uartEventCount;uartEventSize=size;}
extern "C" void HAL_UART_AbortCpltCallback(UART_HandleTypeDef*) {++uartAborted;}
TEST_SUITE("P12NativeUARTIdle") {
TEST_CASE_FIXTURE(Uart,"UART receive-to-idle interrupt reports the actual received prefix length") {
    uartEventCount=uartEventSize=0;REQUIRE(HAL_UART_Init(&h)==HAL_OK);uint8_t rx[4]{};
    REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&h,rx,4)==HAL_OK);native::uartInput(1,{0x31,0x97});
    HAL_UART_IRQHandler(&h);HAL_UART_IRQHandler(&h);native::flush();USART2->SR=USART_SR_IDLE;
    HAL_UART_IRQHandler(&h);CHECK(uartEventCount==1);CHECK(uartEventSize==2);CHECK(rx[0]==0x31);CHECK(rx[1]==0x97);
    CHECK(rx[2]==0);CHECK(HAL_UARTEx_GetRxEventType(&h)==HAL_UART_RXEVENT_IDLE);CHECK(h.RxState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART receive-to-idle interrupt reports transfer completion when its buffer fills") {
    uartEventCount=0;REQUIRE(HAL_UART_Init(&h)==HAL_OK);uint8_t rx[2]{};
    REQUIRE(HAL_UARTEx_ReceiveToIdle_IT(&h,rx,2)==HAL_OK);native::uartInput(1,{0x31,0x97});
    HAL_UART_IRQHandler(&h);HAL_UART_IRQHandler(&h);CHECK(uartEventCount==1);CHECK(uartEventSize==2);
    CHECK(HAL_UARTEx_GetRxEventType(&h)==HAL_UART_RXEVENT_TC);CHECK(rx[0]==0x31);CHECK(rx[1]==0x97);
}
TEST_CASE_FIXTURE(Uart,"UART receive-to-idle polling fills its buffer using distinct supplied bytes") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK);uint8_t rx[2]{};uint16_t length=99;
    native::uartInput(1,{0x31,0x97});REQUIRE(HAL_UARTEx_ReceiveToIdle(&h,rx,2,&length,5)==HAL_OK);
    CHECK(length==2);CHECK(rx[0]==0x31);CHECK(rx[1]==0x97);CHECK(h.RxState==HAL_UART_STATE_READY);
}
TEST_CASE_FIXTURE(Uart,"UART receive-to-idle polling times out without fabricating a received prefix") {
    REQUIRE(HAL_UART_Init(&h)==HAL_OK);uint8_t rx[2]={0xa5,0xa5};uint16_t length=99;
    CHECK(HAL_UARTEx_ReceiveToIdle(&h,rx,2,&length,2)==HAL_TIMEOUT);CHECK(length==0);CHECK(rx[0]==0xa5);
}
TEST_CASE_FIXTURE(Uart,"UART receive-to-idle DMA reports its full buffer after real DMA completion") {
    uartEventCount=0;REQUIRE(HAL_UART_Init(&h)==HAL_OK);DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream1;d.Init.Direction=DMA_PERIPH_TO_MEMORY;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmarx=&d;uint8_t rx[2]{};
    REQUIRE(HAL_UARTEx_ReceiveToIdle_DMA(&h,rx,2)==HAL_OK);native::uartInput(1,{0x31,0x97});
    native::completeDMA(d);HAL_DMA_IRQHandler(&d);CHECK(uartEventCount==1);CHECK(uartEventSize==2);
    CHECK(rx[0]==0x31);CHECK(rx[1]==0x97);CHECK(HAL_UARTEx_GetRxEventType(&h)==HAL_UART_RXEVENT_TC);
}
TEST_CASE_FIXTURE(Uart,"UART interrupt abort without DMA releases both directions before its callback") {
    uartAborted=0;REQUIRE(HAL_UART_Init(&h)==HAL_OK);uint8_t tx=7,rx=0;
    REQUIRE(HAL_UART_Transmit_IT(&h,&tx,1)==HAL_OK);REQUIRE(HAL_UART_Receive_IT(&h,&rx,1)==HAL_OK);
    REQUIRE(HAL_UART_Abort_IT(&h)==HAL_OK);CHECK(uartAborted==1);CHECK(h.gState==HAL_UART_STATE_READY);
    CHECK(h.RxState==HAL_UART_STATE_READY);CHECK(HAL_UART_GetError(&h)==HAL_UART_ERROR_NONE);
}
}
TEST_SUITE("P12NativeSPIDuplexDMA") {
TEST_CASE_FIXTURE(Spi,"SPI full duplex DMA delivers independent received data and completes once") {
    txrx=0;REQUIRE(HAL_SPI_Init(&h)==HAL_OK);DMA_HandleTypeDef txd{},rxd{};
    txd.Instance=DMA1_Stream3;txd.Init.Direction=DMA_MEMORY_TO_PERIPH;txd.Init.MemInc=DMA_MINC_ENABLE;
    rxd.Instance=DMA1_Stream0;rxd.Init.Direction=DMA_PERIPH_TO_MEMORY;rxd.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&txd)==HAL_OK);REQUIRE(HAL_DMA_Init(&rxd)==HAL_OK);
    txd.Parent=rxd.Parent=&h;h.hdmatx=&txd;h.hdmarx=&rxd;uint8_t tx[]={0x12,0x34},rx[2]{};
    REQUIRE(HAL_SPI_TransmitReceive_DMA(&h,tx,rx,2)==HAL_OK);native::spiInput(0,{0xa5,0x7c});
    native::completeDMA(txd);HAL_DMA_IRQHandler(&txd);CHECK(txrx==0);
    native::completeDMA(rxd);HAL_DMA_IRQHandler(&rxd);CHECK(txrx==1);CHECK(rx[0]==0xa5);CHECK(rx[1]==0x7c);
    CHECK(h.State==HAL_SPI_STATE_READY);CHECK((SPI1->CR2&(SPI_CR2_RXDMAEN|SPI_CR2_TXDMAEN))==0);
}
TEST_CASE_FIXTURE(Spi,"SPI sixteen-bit full duplex interrupt transfer preserves both word widths") {
    txrx=0;h.Init.DataSize=SPI_DATASIZE_16BIT;REQUIRE(HAL_SPI_Init(&h)==HAL_OK);
    uint16_t tx[]={0x1234,0x5678},rx[2]{};
    REQUIRE(HAL_SPI_TransmitReceive_IT(&h,reinterpret_cast<uint8_t*>(tx),reinterpret_cast<uint8_t*>(rx),2)==HAL_OK);
    native::spiInput(0,{0xa5e7,0x7c39});
    for(unsigned n=0;n<12&&h.State!=HAL_SPI_STATE_READY;++n){native::advanceBus();HAL_SPI_IRQHandler(&h);}
    CHECK(rx[0]==0xa5e7);CHECK(rx[1]==0x7c39);CHECK(txrx==1);CHECK(HAL_SPI_GetState(&h)==HAL_SPI_STATE_READY);
    CHECK(HAL_SPI_GetError(&h)==HAL_SPI_ERROR_NONE);
}
}
TEST_SUITE("P12NativeI2CAdditionalModes") {
TEST_CASE_FIXTURE(I2c,"I2C blocking slave transmission waits for address data-ready and final NACK inputs") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);bool addressed=false;uint8_t tx[]={0x27,0x93};
    native::event([&](uint32_t){if(h.State==HAL_I2C_STATE_BUSY_TX){
        if(!addressed){I2C1->SR1=I2C1->SR1|I2C_SR1_ADDR;addressed=true;}
        I2C1->SR1=I2C1->SR1|(h.XferCount?I2C_SR1_TXE:I2C_SR1_AF);}});
    REQUIRE(HAL_I2C_Slave_Transmit(&h,tx,2,20)==HAL_OK);
    std::vector<uint8_t> sent;for(const auto &w:native::writes())if(w.address==reinterpret_cast<uintptr_t>(&I2C1->DR))sent.push_back(w.value);
    CHECK(sent==std::vector<uint8_t>{0x27,0x93});CHECK(h.State==HAL_I2C_STATE_READY);CHECK(h.Lock==HAL_UNLOCKED);
}
TEST_CASE_FIXTURE(I2c,"I2C blocking slave reception delivers both bytes then observes STOP") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);bool addressed=false;uint8_t rx[2]{};
    native::event([&](uint32_t){if(h.State==HAL_I2C_STATE_BUSY_RX){
        if(!addressed){I2C1->SR1=I2C1->SR1|I2C_SR1_ADDR;addressed=true;}
        if(h.XferCount){I2C1->DR=h.XferCount==2?0x27:0x93;I2C1->SR1=I2C1->SR1|I2C_SR1_RXNE;}
        else I2C1->SR1=I2C1->SR1|I2C_SR1_STOPF;}});
    REQUIRE(HAL_I2C_Slave_Receive(&h,rx,2,20)==HAL_OK);CHECK(rx[0]==0x27);CHECK(rx[1]==0x93);
    CHECK(h.State==HAL_I2C_STATE_READY);native::flush();CHECK((I2C1->SR1&I2C_SR1_STOPF)==0);
}
TEST_CASE_FIXTURE(I2c,"I2C ten-bit interrupt address transmission emits header low address then payload") {
    h.Init.AddressingMode=I2C_ADDRESSINGMODE_10BIT;REQUIRE(HAL_I2C_Init(&h)==HAL_OK);uint8_t tx=0x73;
    REQUIRE(HAL_I2C_Master_Transmit_IT(&h,0x2ab,&tx,1)==HAL_OK);
    for(auto status:{I2C_SR1_SB,I2C_SR1_ADD10,I2C_SR1_ADDR,I2C_SR1_TXE,I2C_SR1_BTF}) {
        native::flush();I2C1->SR1=status;I2C1->SR2=I2C_SR2_MSL|I2C_SR2_TRA;HAL_I2C_EV_IRQHandler(&h);
    }
    std::vector<uint8_t> sent;for(const auto &w:native::writes())if(w.address==reinterpret_cast<uintptr_t>(&I2C1->DR))sent.push_back(w.value);
    CHECK(sent==std::vector<uint8_t>{0xf4,0xab,0x73});CHECK(i2cTx==1);CHECK(h.State==HAL_I2C_STATE_READY);
}
TEST_CASE_FIXTURE(I2c,"I2C sequential DMA write completes its final frame through DMA and BTF") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29);DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream6;d.Init.Direction=DMA_MEMORY_TO_PERIPH;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmatx=&d;uint8_t tx[]={0x27,0x93};
    REQUIRE(HAL_I2C_Master_Seq_Transmit_DMA(&h,0x52,tx,2,I2C_FIRST_AND_LAST_FRAME)==HAL_OK);
    HAL_I2C_EV_IRQHandler(&h);HAL_I2C_EV_IRQHandler(&h);native::completeDMA(d);HAL_DMA_IRQHandler(&d);HAL_I2C_EV_IRQHandler(&h);
    CHECK(i2cTx==1);CHECK(h.State==HAL_I2C_STATE_READY);CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x27,0x93});
}
TEST_CASE_FIXTURE(I2c,"I2C sequential DMA read completes its final frame with independent data") {
    REQUIRE(HAL_I2C_Init(&h)==HAL_OK);native::i2cPeer(0,0x29,{0x27,0x93});DMA_HandleTypeDef d{};
    d.Instance=DMA1_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;d.Init.MemInc=DMA_MINC_ENABLE;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);d.Parent=&h;h.hdmarx=&d;uint8_t rx[2]{};
    REQUIRE(HAL_I2C_Master_Seq_Receive_DMA(&h,0x52,rx,2,I2C_FIRST_AND_LAST_FRAME)==HAL_OK);
    HAL_I2C_EV_IRQHandler(&h);HAL_I2C_EV_IRQHandler(&h);native::completeDMA(d);HAL_DMA_IRQHandler(&d);
    CHECK(i2cRx==1);CHECK(h.State==HAL_I2C_STATE_READY);CHECK(rx[0]==0x27);CHECK(rx[1]==0x93);
}
}
