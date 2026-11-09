#include "native.h"
#include <stdexcept>
extern "C" {
void p12_probe_write32(volatile uint32_t*,uint32_t);
void p12_probe_write_twice(volatile uint32_t*,uint32_t,uint32_t);
uint32_t p12_probe_read32(volatile uint32_t*);
}
TEST_SUITE("P12NativeBusControls") {
TEST_CASE_FIXTURE(native::Fixture,"access observation retains both consecutive volatile writes to the same word") {
    p12_probe_write_twice(&GPIOC->ODR,0x12,0x34); native::flush();
    std::vector<uint64_t> values;
    for(const auto &w:native::writes()) if(w.address==reinterpret_cast<uintptr_t>(&GPIOC->ODR)) values.push_back(w.value);
    REQUIRE(values.size()==2); CHECK(values[0]==0x12); CHECK(values[1]==0x34);
}
TEST_CASE_FIXTURE(native::Fixture,"write-one-to-clear model preserves unselected pending requests") {
    EXTI->PR=(1U<<2)|(1U<<9); p12_probe_write32(&EXTI->PR,1U<<2); native::flush();
    CHECK(EXTI->PR==(1U<<9));
    p12_probe_write32(&EXTI->PR,0); native::flush(); CHECK(EXTI->PR==(1U<<9));
}
TEST_CASE_FIXTURE(native::Fixture,"write-zero-to-clear I2C flags do not create unrelated receive events") {
    I2C1->SR1=I2C_SR1_BERR|I2C_SR1_RXNE;
    p12_probe_write32(&I2C1->SR1,~I2C_SR1_BERR); native::flush(); CHECK(I2C1->SR1==I2C_SR1_RXNE);
    I2C1->DR=0xa7; CHECK(p12_probe_read32(&I2C1->DR)==0xa7); CHECK((I2C1->SR1&I2C_SR1_RXNE)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"flash key model rejects an incomplete or incorrect key sequence") {
    p12_probe_write32(&FLASH->KEYR,FLASH_KEY2); native::flush(); CHECK((FLASH->CR&FLASH_CR_LOCK)!=0);
    p12_probe_write_twice(&FLASH->KEYR,FLASH_KEY1,0); native::flush(); CHECK((FLASH->CR&FLASH_CR_LOCK)!=0);
    p12_probe_write_twice(&FLASH->KEYR,FLASH_KEY1,FLASH_KEY2); native::flush(); CHECK((FLASH->CR&FLASH_CR_LOCK)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"DMA event model rejects completion for an inactive stream") {
    DMA_HandleTypeDef h{}; h.Instance=DMA1_Stream0; h.State=HAL_DMA_STATE_READY;
    CHECK_THROWS_AS(native::completeDMA(h),std::logic_error);
}
TEST_CASE_FIXTURE(native::Fixture,"DMA memory copy model obeys independent addresses widths and increments") {
    DMA_HandleTypeDef h{}; h.Instance=DMA1_Stream0; h.Init.Direction=DMA_MEMORY_TO_MEMORY;
    h.Init.MemInc=DMA_MINC_ENABLE; h.Init.PeriphInc=DMA_PINC_ENABLE;
    h.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD; h.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;
    REQUIRE(HAL_DMA_Init(&h)==HAL_OK); uint16_t from[]={0x1234,0xabcd,0x7654},to[3]{};
    REQUIRE(HAL_DMA_Start(&h,reinterpret_cast<uint32_t>(from),reinterpret_cast<uint32_t>(to),3)==HAL_OK);
    native::completeDMA(h); CHECK(to[0]==0x1234); CHECK(to[1]==0xabcd); CHECK(to[2]==0x7654);
    CHECK(h.Instance->NDTR==0); CHECK((h.Instance->CR&DMA_SxCR_EN)==0);
}
}
TEST_SUITE("P12NativeBusProtocolControls") {
TEST_CASE_FIXTURE(native::Fixture,"USB status W1C leaves receive and endpoint aggregate levels unchanged") {
    auto *u=USB_OTG_FS;u->GINTSTS=USB_OTG_GINTSTS_USBRST|USB_OTG_GINTSTS_RXFLVL|USB_OTG_GINTSTS_IEPINT;
    p12_probe_write32(&u->GINTSTS,0xffffffffU);native::flush();
    CHECK(u->GINTSTS==(USB_OTG_GINTSTS_RXFLVL|USB_OTG_GINTSTS_IEPINT));
}
TEST_CASE_FIXTURE(native::Fixture,"USB reset request clears its command while preserving the hardware idle input") {
    USB_OTG_FS->GRSTCTL=USB_OTG_GRSTCTL_AHBIDL;
    p12_probe_write32(&USB_OTG_FS->GRSTCTL,USB_OTG_GRSTCTL_CSRST);native::flush();
    CHECK(USB_OTG_FS->GRSTCTL==USB_OTG_GRSTCTL_AHBIDL);
}
TEST_CASE_FIXTURE(native::Fixture,"I2C address pending clears only after the required status read pair") {
    I2C1->SR1=I2C_SR1_ADDR|I2C_SR1_BERR;I2C1->SR2=I2C_SR2_MSL;
    CHECK(p12_probe_read32(&I2C1->SR2)==I2C_SR2_MSL);CHECK((I2C1->SR1&I2C_SR1_ADDR)!=0);
    CHECK(p12_probe_read32(&I2C1->SR1)==(I2C_SR1_ADDR|I2C_SR1_BERR));
    CHECK(p12_probe_read32(&I2C1->SR2)==I2C_SR2_MSL);CHECK(I2C1->SR1==I2C_SR1_BERR);
}
TEST_CASE_FIXTURE(native::Fixture,"I2C peer acknowledges the configured address and records the data request") {
    native::i2cPeer(0,0x29);p12_probe_write32(&I2C1->CR1,I2C_CR1_START);native::flush();
    CHECK((I2C1->SR1&I2C_SR1_SB)!=0);p12_probe_write32(&I2C1->DR,0x52);native::flush();
    CHECK((I2C1->SR1&I2C_SR1_ADDR)!=0);CHECK((I2C1->SR2&I2C_SR2_TRA)!=0);
    p12_probe_write32(&I2C1->DR,0x73);native::flush();
    CHECK(native::i2cWritten(0)==std::vector<uint8_t>{0x52,0x73});CHECK((I2C1->SR1&I2C_SR1_BTF)!=0);
}
TEST_CASE_FIXTURE(native::Fixture,"I2C peer refuses an address that does not match its configured endpoint") {
    native::i2cPeer(0,0x29);p12_probe_write32(&I2C1->CR1,I2C_CR1_START);
    p12_probe_write32(&I2C1->DR,0x54);native::flush();
    CHECK((I2C1->SR1&I2C_SR1_AF)!=0);CHECK((I2C1->SR1&I2C_SR1_ADDR)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"SPI input queue consumes distinct words only on data reads") {
    native::spiInput(0,{0x1234,0x789a});CHECK((p12_probe_read32(&SPI1->SR)&SPI_SR_RXNE)!=0);
    CHECK((p12_probe_read32(&SPI1->SR)&SPI_SR_RXNE)!=0);
    CHECK(p12_probe_read32(&SPI1->DR)==0x1234);CHECK((SPI1->SR&SPI_SR_RXNE)!=0);
    CHECK(p12_probe_read32(&SPI1->DR)==0x789a);CHECK((SPI1->SR&SPI_SR_RXNE)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"ADC DMA completion is rejected before the peripheral supplies conversion data") {
    DMA_HandleTypeDef d{};d.Instance=DMA2_Stream0;d.Init.Direction=DMA_PERIPH_TO_MEMORY;
    REQUIRE(HAL_DMA_Init(&d)==HAL_OK);uint8_t rx=0;
    REQUIRE(HAL_DMA_Start(&d,reinterpret_cast<uint32_t>(&ADC1->DR),reinterpret_cast<uint32_t>(&rx),1)==HAL_OK);
    CHECK_THROWS_AS(native::completeDMA(d),std::logic_error);CHECK(rx==0);
}
}
TEST_SUITE("P12NativeSPIFlagControls") {
TEST_CASE_FIXTURE(native::Fixture,"SPI overrun clear requires a data read followed by a status read") {
    SPI1->SR=SPI_SR_OVR|SPI_SR_TXE;SPI1->DR=0x12;
    p12_probe_read32(&SPI1->SR);CHECK((SPI1->SR&SPI_SR_OVR)!=0);
    p12_probe_read32(&SPI1->DR);CHECK((SPI1->SR&SPI_SR_OVR)!=0);
    p12_probe_read32(&SPI1->SR);CHECK(SPI1->SR==SPI_SR_TXE);
}
TEST_CASE_FIXTURE(native::Fixture,"SPI CRC error clear preserves unrelated ready inputs") {
    SPI1->SR=SPI_SR_CRCERR|SPI_SR_TXE;
    p12_probe_write32(&SPI1->SR,~SPI_SR_CRCERR);native::flush();CHECK(SPI1->SR==SPI_SR_TXE);
}
}
