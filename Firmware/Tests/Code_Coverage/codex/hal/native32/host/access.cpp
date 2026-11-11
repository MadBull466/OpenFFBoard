#include "native.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <stdexcept>
// Compiler access callbacks, NOT an ASan memory-safety runtime. Only vendor C
// is instrumented. Each store is finalized before the next instrumented access
// or explicit flush. Tests observe backing words/request traces outside that
// bus; native::flush is required for a final outstanding store.
// Semantics: RM0090 peripheral register descriptions, supplied ST bit masks.
namespace {
struct Pending { uintptr_t a{}; unsigned size{}; uint64_t old{}; } pending;
bool uartPending[6]{}, uartStatusRead[6]{}, spiPending[5]{};
std::deque<uint16_t> uartInputs[6];
unsigned flashKey, optionKey;
bool flashPending;
std::vector<native::Write> requests;
constexpr uintptr_t uartBases[]={
#ifdef USART1_BASE
    USART1_BASE,
#endif
#ifdef USART2_BASE
    USART2_BASE,
#endif
#ifdef USART3_BASE
    USART3_BASE,
#endif
#ifdef UART4_BASE
    UART4_BASE,
#endif
#ifdef UART5_BASE
    UART5_BASE,
#endif
#ifdef USART6_BASE
    USART6_BASE,
#endif
};
constexpr uintptr_t spiBases[]={
#ifdef SPI1_BASE
    SPI1_BASE,
#endif
#ifdef SPI2_BASE
    SPI2_BASE,
#endif
#ifdef SPI3_BASE
    SPI3_BASE,
#endif
#ifdef SPI4_BASE
    SPI4_BASE,
#endif
#ifdef SPI5_BASE
    SPI5_BASE,
#endif
};
constexpr uintptr_t i2cBases[]={
#ifdef I2C1_BASE
    I2C1_BASE,
#endif
#ifdef I2C2_BASE
    I2C2_BASE,
#endif
#ifdef I2C3_BASE
    I2C3_BASE,
#endif
};
constexpr uintptr_t gpioBases[]={
#ifdef GPIOA_BASE
    GPIOA_BASE,
#endif
#ifdef GPIOB_BASE
    GPIOB_BASE,
#endif
#ifdef GPIOC_BASE
    GPIOC_BASE,
#endif
#ifdef GPIOD_BASE
    GPIOD_BASE,
#endif
#ifdef GPIOE_BASE
    GPIOE_BASE,
#endif
#ifdef GPIOF_BASE
    GPIOF_BASE,
#endif
#ifdef GPIOG_BASE
    GPIOG_BASE,
#endif
#ifdef GPIOH_BASE
    GPIOH_BASE,
#endif
#ifdef GPIOI_BASE
    GPIOI_BASE,
#endif
};
constexpr uintptr_t adcBases[]={
#ifdef ADC1_BASE
    ADC1_BASE,
#endif
#ifdef ADC2_BASE
    ADC2_BASE,
#endif
#ifdef ADC3_BASE
    ADC3_BASE,
#endif
};
constexpr uintptr_t timerBases[]={
#ifdef TIM1_BASE
    TIM1_BASE,
#endif
#ifdef TIM2_BASE
    TIM2_BASE,
#endif
#ifdef TIM3_BASE
    TIM3_BASE,
#endif
#ifdef TIM4_BASE
    TIM4_BASE,
#endif
#ifdef TIM5_BASE
    TIM5_BASE,
#endif
#ifdef TIM6_BASE
    TIM6_BASE,
#endif
#ifdef TIM7_BASE
    TIM7_BASE,
#endif
#ifdef TIM8_BASE
    TIM8_BASE,
#endif
#ifdef TIM9_BASE
    TIM9_BASE,
#endif
#ifdef TIM10_BASE
    TIM10_BASE,
#endif
#ifdef TIM11_BASE
    TIM11_BASE,
#endif
#ifdef TIM12_BASE
    TIM12_BASE,
#endif
#ifdef TIM13_BASE
    TIM13_BASE,
#endif
#ifdef TIM14_BASE
    TIM14_BASE,
#endif
};
constexpr uintptr_t usbBases[]={
#ifdef USB_OTG_FS_PERIPH_BASE
    USB_OTG_FS_PERIPH_BASE,
#endif
#ifdef USB_OTG_HS_PERIPH_BASE
    USB_OTG_HS_PERIPH_BASE,
#endif
};
int gpioIndex(uintptr_t a) { for(unsigned i=0;i<sizeof(gpioBases)/sizeof(*gpioBases);++i) if(a>=gpioBases[i]&&a<gpioBases[i]+sizeof(GPIO_TypeDef)) return i; return -1; }
bool i2cReadStatus[3]{};
struct I2cPeer { bool enabled{},addressPhase{},reading{}; uint8_t address{}; std::deque<uint8_t> input; std::vector<uint8_t> output; } peers[3];
std::deque<uint16_t> spiInputs[5];
std::deque<uint32_t> adcInputs[3];
bool spiStatusRead[5]{},spiOverrunDataRead[5]{};
struct GpioLock { unsigned phase{}; uint16_t pins{}; bool locked{}; } gpioLocks[9];
std::deque<uint32_t> usbInputs[2]; bool usbInputEnabled[2]{};
uint64_t load(uintptr_t a,unsigned n) { uint64_t x=0; std::memcpy(&x,reinterpret_cast<void*>(a),n); return x; }
void store(uintptr_t a,unsigned n,uint64_t x) { std::memcpy(reinterpret_cast<void*>(a),&x,n); }
bool peripheral(uintptr_t a) { return (a>=0x40000000U&&a<0x44000000U)||(a>=0x50000000U&&a<0x50040000U)||(a>=0xe000e000U&&a<0xe000ffffU)||(a>=0xe0042000U&&a<0xe0043000U); }
void fault(const char *s) { std::fprintf(stderr,"P12 bus model fault: %s\n",s); std::abort(); }
void effect(uintptr_t a,unsigned n,uint64_t old,uint64_t value) {
    requests.push_back({a,n,value});
    if(a>=PERIPH_BB_BASE&&a<0x43000000U) {
        uintptr_t target=PERIPH_BASE+(a-PERIPH_BB_BASE)/32U;
        unsigned bit=((a-PERIPH_BB_BASE)%32U)/4U;
        uint32_t before=static_cast<uint32_t>(load(target,1));
        store(target,1,(before&~(1U<<bit))|((value&1U)<<bit)); return;
    }
    if(a==reinterpret_cast<uintptr_t>(&FLASH->KEYR)) {
        if(flashKey==1&&value==FLASH_KEY2) { FLASH->CR=FLASH->CR&~FLASH_CR_LOCK; flashKey=0; }
        else flashKey=value==FLASH_KEY1?1:0;
    }
    if(a==reinterpret_cast<uintptr_t>(&FLASH->OPTKEYR)) {
        if(optionKey==1&&value==FLASH_OPT_KEY2) { FLASH->OPTCR=FLASH->OPTCR&~FLASH_OPTCR_OPTLOCK; optionKey=0; }
        else optionKey=value==FLASH_OPT_KEY1?1:0;
    }
    if(a==reinterpret_cast<uintptr_t>(&FLASH->SR)) store(a,n,old&~value);
    if(a==reinterpret_cast<uintptr_t>(&EXTI->PR)) { store(a,n,old&~value); EXTI->SWIER=EXTI->SWIER&~value; }
    if(a==reinterpret_cast<uintptr_t>(&EXTI->SWIER)) EXTI->PR=EXTI->PR|(static_cast<uint32_t>(~old&value)&EXTI->IMR);
    if(gpioIndex(a)>=0) {
        unsigned index=static_cast<unsigned>(gpioIndex(a)),offset=a-gpioBases[index];
        auto *g=reinterpret_cast<GPIO_TypeDef*>(gpioBases[index]); auto &l=gpioLocks[index];
        if(offset==offsetof(GPIO_TypeDef,BSRR)) { g->ODR=(g->ODR&~(value>>16))|(value&0xffffU); }
        if(offset==offsetof(GPIO_TypeDef,LCKR)) {
            if(l.locked) store(a,n,l.pins|GPIO_LCKR_LCKK);
            else {
                bool key=(value&GPIO_LCKR_LCKK)!=0; auto pins=static_cast<uint16_t>(value);
                if(l.phase==0&&key) {l.phase=1;l.pins=pins;}
                else if(l.phase==1&&!key&&l.pins==pins) l.phase=2;
                else if(l.phase==2&&key&&l.pins==pins) l.phase=3;
                else {l.phase=0;l.pins=pins;}
            }
        }
        unsigned width=offset==4?1:offset==0||offset==8||offset==12?2:offset==32||offset==36?4:0;
        if(width&&l.locked) {
            uint32_t mask=0;unsigned first=offset==36?8:0;
            for(unsigned pin=first;pin<16&&pin<first+32/width;++pin)
                if(l.pins&(1U<<pin)) mask|=((1U<<width)-1U)<<((pin-first)*width);
            store(a,n,(value&~mask)|(old&mask));
        }
    }
    for(uintptr_t b : usbBases) {
        if(a==b+offsetof(USB_OTG_GlobalTypeDef,GRSTCTL))
            store(a,n,(old&USB_OTG_GRSTCTL_AHBIDL)|(value&~(USB_OTG_GRSTCTL_AHBIDL|USB_OTG_GRSTCTL_CSRST|USB_OTG_GRSTCTL_TXFFLSH|USB_OTG_GRSTCTL_RXFFLSH)));
        if(a==b+offsetof(USB_OTG_GlobalTypeDef,GINTSTS)) {
            constexpr uint32_t readOnly=USB_OTG_GINTSTS_CMOD|USB_OTG_GINTSTS_OTGINT|USB_OTG_GINTSTS_RXFLVL|
                USB_OTG_GINTSTS_NPTXFE|USB_OTG_GINTSTS_GINAKEFF|USB_OTG_GINTSTS_BOUTNAKEFF|
                USB_OTG_GINTSTS_IEPINT|USB_OTG_GINTSTS_OEPINT|USB_OTG_GINTSTS_HPRTINT|
                USB_OTG_GINTSTS_HCINT|USB_OTG_GINTSTS_PTXFE;
            store(a,n,old&~(value&~readOnly));
        }
        if(a==b+offsetof(USB_OTG_GlobalTypeDef,GOTGINT)) store(a,n,old&~value);
        if(a==b+USB_OTG_DEVICE_BASE+offsetof(USB_OTG_DeviceTypeDef,DAINT)) store(a,n,old);
        if(a==b+offsetof(USB_OTG_GlobalTypeDef,GUSBCFG)) {
            auto *usb=reinterpret_cast<USB_OTG_GlobalTypeDef*>(b);
            if(value&USB_OTG_GUSBCFG_FHMOD) usb->GINTSTS=usb->GINTSTS|USB_OTG_GINTSTS_CMOD;
            if(value&USB_OTG_GUSBCFG_FDMOD) usb->GINTSTS=usb->GINTSTS&~USB_OTG_GINTSTS_CMOD;
        }
        for(unsigned ch=0;ch<12;++ch) {
            uintptr_t base=b+USB_OTG_HOST_CHANNEL_BASE+ch*USB_OTG_HOST_CHANNEL_SIZE;
            if(a==base+8) store(a,n,old&~value);
            if(a==base&&(value&USB_OTG_HCCHAR_CHDIS)) store(a,n,value&~USB_OTG_HCCHAR_CHENA);
        }
        for(unsigned ep=0;ep<6;++ep)
            if(a==b+USB_OTG_IN_ENDPOINT_BASE+ep*USB_OTG_EP_REG_SIZE+8||
               a==b+USB_OTG_OUT_ENDPOINT_BASE+ep*USB_OTG_EP_REG_SIZE+8) store(a,n,old&~value);
    }
    for(uintptr_t b : {uintptr_t(DMA1_BASE),uintptr_t(DMA2_BASE)})
        if(a==b+8||a==b+12) store(b+(a==b+12?4:0),4,load(b+(a==b+12?4:0),4)&~value);
    for(unsigned i=0;i<3;++i) {
        auto *r=reinterpret_cast<I2C_TypeDef*>(i2cBases[i]);
        auto &peer=peers[i];
        if(a==reinterpret_cast<uintptr_t>(&r->CR1)&&i2cReadStatus[i]) {r->SR1=r->SR1&~I2C_SR1_STOPF;i2cReadStatus[i]=false;}
        if(peer.enabled&&a==reinterpret_cast<uintptr_t>(&r->CR1)) {
            if(value&I2C_CR1_START) { peer.addressPhase=true; r->SR1=r->SR1|I2C_SR1_SB; r->SR2=r->SR2|I2C_SR2_BUSY|I2C_SR2_MSL; }
            if(value&I2C_CR1_STOP) r->SR2=r->SR2&~(I2C_SR2_BUSY|I2C_SR2_MSL);
        }
        if(a==reinterpret_cast<uintptr_t>(&r->SR1)) {
            constexpr uint32_t w0c=I2C_SR1_BERR|I2C_SR1_ARLO|I2C_SR1_AF|I2C_SR1_OVR|I2C_SR1_PECERR|I2C_SR1_TIMEOUT|I2C_SR1_SMBALERT;
            store(a,n,old&~((~value)&w0c));
        }
        if(a==reinterpret_cast<uintptr_t>(&r->DR)) {
            r->SR1=r->SR1&~(I2C_SR1_SB|I2C_SR1_TXE|I2C_SR1_BTF);
            if(peer.enabled) {
                peer.output.push_back(static_cast<uint8_t>(value));
                if(peer.addressPhase) {
                    peer.addressPhase=false; peer.reading=(value&1U)!=0; r->CR1=r->CR1&~I2C_CR1_START;
                    if((value>>1)!=peer.address) r->SR1=r->SR1|I2C_SR1_AF;
                    else {
                        r->SR1=r->SR1|I2C_SR1_ADDR;
                        r->SR2=(r->SR2&~I2C_SR2_TRA)|(peer.reading?0:I2C_SR2_TRA);
                        if(!peer.reading) r->SR1=r->SR1|I2C_SR1_TXE;
                    }
                } else r->SR1=r->SR1|I2C_SR1_TXE|I2C_SR1_BTF;
            }
        }
    }
    for(unsigned i=0;i<sizeof(uartBases)/sizeof(*uartBases);++i) {
        auto *r=reinterpret_cast<USART_TypeDef*>(uartBases[i]);
        if(a==reinterpret_cast<uintptr_t>(&r->SR)) store(a,n,old&value);
        if(a==reinterpret_cast<uintptr_t>(&r->DR)) { r->SR=r->SR&~(USART_SR_TXE|USART_SR_TC); uartPending[i]=true; }
    }
    for(unsigned i=0;i<sizeof(spiBases)/sizeof(*spiBases);++i) {
        auto *r=reinterpret_cast<SPI_TypeDef*>(spiBases[i]);
        if(a==reinterpret_cast<uintptr_t>(&r->SR)) store(a,n,old&~((~value)&SPI_SR_CRCERR));
        if(a==reinterpret_cast<uintptr_t>(&r->CR1)&&spiStatusRead[i]) {r->SR=r->SR&~SPI_SR_MODF;spiStatusRead[i]=false;}
        if(a==reinterpret_cast<uintptr_t>(&r->DR)) {r->SR=r->SR&~SPI_SR_TXE;spiPending[i]=true;}
    }
#ifdef CAN1_BASE
    for(auto b:{uintptr_t(CAN1_BASE),uintptr_t(CAN2_BASE)}) {
        auto *c=reinterpret_cast<CAN_TypeDef*>(b);
        if(a==reinterpret_cast<uintptr_t>(&c->TSR)) {
            uint32_t clear=0;for(unsigned box=0;box<3;++box) if(value&(1U<<(box*8)))clear|=15U<<(box*8);
            store(a,n,(old&~clear)|(value&(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2)));
        }
        if(a==reinterpret_cast<uintptr_t>(&c->MSR))store(a,n,old&~(value&(CAN_MSR_ERRI|CAN_MSR_WKUI|CAN_MSR_SLAKI)));
    }
#endif
    for(uintptr_t b : adcBases)
        if(a==b+offsetof(ADC_TypeDef,SR)) store(a,n,old&value);
    for(uintptr_t b : timerBases)
        if(a==b+offsetof(TIM_TypeDef,SR)) store(a,n,old&value);
    if(a>=FLASH_BASE&&a<FLASH_BASE+0x200000U) {
        if((value&~old)!=0) fault("programming must not set erased flash bits");
        FLASH->SR=FLASH->SR|FLASH_SR_BSY; flashPending=true;
    }
}
void beforeRead(uintptr_t a) {
    for(unsigned i=0;i<sizeof(usbBases)/sizeof(*usbBases);++i) {
        uintptr_t b=usbBases[i];
        auto *usb=reinterpret_cast<USB_OTG_GlobalTypeDef*>(b);
        if(a==b+offsetof(USB_OTG_GlobalTypeDef,GRXSTSP)) usb->GINTSTS=usb->GINTSTS&~USB_OTG_GINTSTS_RXFLVL;
        if(a==b+USB_OTG_FIFO_BASE&&usbInputEnabled[i]) {
            if(usbInputs[i].empty()) fault("USB FIFO read beyond supplied packet words");
            store(a,4,usbInputs[i].front()); usbInputs[i].pop_front();
        }
    }
    if(gpioIndex(a)>=0) {
        unsigned index=static_cast<unsigned>(gpioIndex(a)),offset=a-gpioBases[index];
        if(offset==offsetof(GPIO_TypeDef,BSRR)) store(a,4,0);
        if(offset==offsetof(GPIO_TypeDef,LCKR)) {
            auto &l=gpioLocks[index];if(l.phase==3) {l.locked=true;l.phase=4;}
            store(a,4,l.pins|(l.locked?GPIO_LCKR_LCKK:0U));
        }
    }
    for(unsigned i=0;i<3;++i) {
        auto *r=reinterpret_cast<I2C_TypeDef*>(i2cBases[i]);
        if(a==reinterpret_cast<uintptr_t>(&r->SR1)) i2cReadStatus[i]=true;
        if(a==reinterpret_cast<uintptr_t>(&r->SR2)&&i2cReadStatus[i]) {
            r->SR1=r->SR1&~I2C_SR1_ADDR; i2cReadStatus[i]=false;
            if(peers[i].enabled&&peers[i].reading&&!peers[i].input.empty())
                r->SR1=r->SR1|I2C_SR1_RXNE|(peers[i].input.size()>1?I2C_SR1_BTF:0);
        }
        if(a==reinterpret_cast<uintptr_t>(&r->DR)) {
            auto &peer=peers[i];
            if(peer.enabled&&peer.reading) {
                if(peer.input.empty()) fault("I2C read beyond supplied peer input");
                r->DR=peer.input.front(); peer.input.pop_front();
                r->SR1=(r->SR1&~(I2C_SR1_RXNE|I2C_SR1_BTF))|
                    (peer.input.empty()?0:I2C_SR1_RXNE)|(peer.input.size()>1?I2C_SR1_BTF:0);
            } else r->SR1=r->SR1&~I2C_SR1_RXNE;
        }
    }
    for(unsigned i=0;i<sizeof(spiBases)/sizeof(*spiBases);++i) if(a==spiBases[i]+offsetof(SPI_TypeDef,DR)) {
        auto *r=reinterpret_cast<SPI_TypeDef*>(spiBases[i]);
        spiOverrunDataRead[i]=(r->SR&SPI_SR_OVR)!=0;
        if(!spiInputs[i].empty()) { r->DR=spiInputs[i].front(); spiInputs[i].pop_front(); }
        if(spiInputs[i].empty()) r->SR=r->SR&~SPI_SR_RXNE;
    }
    for(unsigned i=0;i<sizeof(spiBases)/sizeof(*spiBases);++i) if(a==spiBases[i]+offsetof(SPI_TypeDef,SR)) {
        auto *r=reinterpret_cast<SPI_TypeDef*>(spiBases[i]);spiStatusRead[i]=true;
        if(spiOverrunDataRead[i]) {r->SR=r->SR&~SPI_SR_OVR;spiOverrunDataRead[i]=false;}
    }
    for(unsigned i=0;i<sizeof(uartBases)/sizeof(*uartBases);++i) {
        auto *r=reinterpret_cast<USART_TypeDef*>(uartBases[i]);
        if(a==reinterpret_cast<uintptr_t>(&r->SR)) uartStatusRead[i]=true;
        if(a==reinterpret_cast<uintptr_t>(&r->DR)) {
            if(!uartInputs[i].empty()) {r->DR=uartInputs[i].front();uartInputs[i].pop_front();}
            if(uartInputs[i].empty()) r->SR=r->SR&~USART_SR_RXNE;
            if(uartStatusRead[i]) r->SR=r->SR&~(USART_SR_IDLE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE);
            uartStatusRead[i]=false;
        }
    }
    for(unsigned i=0;i<sizeof(adcBases)/sizeof(*adcBases);++i) if(a==adcBases[i]+offsetof(ADC_TypeDef,DR)) {
        auto *r=reinterpret_cast<ADC_TypeDef*>(adcBases[i]);
        if(!adcInputs[i].empty()) {r->DR=adcInputs[i].front();adcInputs[i].pop_front();}
        if(adcInputs[i].empty()) r->SR=r->SR&~ADC_SR_EOC;
    }
}
}
namespace native {
void flush() {
    if(!pending.a) return;
    Pending p=pending; pending={};
    effect(p.a,p.size,p.old,load(p.a,p.size));
}
void resetBus() {
    requests.clear();
    pending={}; std::memset(uartPending,0,sizeof uartPending); std::memset(spiPending,0,sizeof spiPending);
    std::memset(i2cReadStatus,0,sizeof i2cReadStatus);
    flashKey=optionKey=0; flashPending=false;
    for(auto &p:peers) p={};
    for(auto &q:spiInputs) q.clear();
    for(auto &q:uartInputs) q.clear();
    for(auto &q:adcInputs) q.clear();
    std::memset(uartStatusRead,0,sizeof uartStatusRead);
    std::memset(spiStatusRead,0,sizeof spiStatusRead);
    std::memset(spiOverrunDataRead,0,sizeof spiOverrunDataRead);
    for(auto &l:gpioLocks) l={};
    for(auto &q:usbInputs) q.clear();for(auto &b:usbInputEnabled) b=false;
}
const std::vector<Write> &writes() { flush(); return requests; }
void usbInput(unsigned index,const std::vector<uint32_t> &words) {
    if(index>=sizeof(usbBases)/sizeof(*usbBases)) fault("invalid USB FIFO");usbInputs[index]={words.begin(),words.end()};usbInputEnabled[index]=true;
}
void i2cPeer(unsigned index,uint8_t address,const std::vector<uint8_t> &input) {
    if(index>=3||address>127) fault("invalid I2C peer");
    peers[index]={true,false,false,address,std::deque<uint8_t>(input.begin(),input.end()),{}};
}
const std::vector<uint8_t> &i2cWritten(unsigned index) { flush(); if(index>=3) fault("invalid I2C peer"); return peers[index].output; }
void adcInput(unsigned index,const std::vector<uint32_t> &input) {
    if(index>=sizeof(adcBases)/sizeof(*adcBases)) fault("invalid ADC input");
    adcInputs[index]={input.begin(),input.end()};
    if(!input.empty()) {auto *r=reinterpret_cast<ADC_TypeDef*>(adcBases[index]);r->SR=r->SR|ADC_SR_EOC;}
}
void uartInput(unsigned index,const std::vector<uint16_t> &input) {
    if(index>=sizeof(uartBases)/sizeof(*uartBases)) fault("invalid UART input");
    uartInputs[index]={input.begin(),input.end()};
    if(!input.empty()) {auto *r=reinterpret_cast<USART_TypeDef*>(uartBases[index]);r->SR=r->SR|USART_SR_RXNE;}
}
void spiInput(unsigned index,const std::vector<uint16_t> &input) {
    if(index>=sizeof(spiBases)/sizeof(*spiBases)) fault("invalid SPI input"); spiInputs[index]={input.begin(),input.end()};
    if(!input.empty()) { auto *r=reinterpret_cast<SPI_TypeDef*>(spiBases[index]); r->SR=r->SR|SPI_SR_RXNE; }
}
void completeDMA(DMA_HandleTypeDef &h) {
    flush();
    if(h.State!=HAL_DMA_STATE_BUSY||h.Instance==nullptr||(h.Instance->CR&DMA_SxCR_EN)==0)
        throw std::logic_error("DMA completion without an enabled active stream");
    auto *s=h.Instance; unsigned direction=(s->CR&DMA_SxCR_DIR)>>6;
    unsigned memoryWidth=1U<<((s->CR&DMA_SxCR_MSIZE)>>13),peripheralWidth=1U<<((s->CR&DMA_SxCR_PSIZE)>>11);
    uintptr_t m=(s->CR&DMA_SxCR_DBM)&&(s->CR&DMA_SxCR_CT)?s->M1AR:s->M0AR,p=s->PAR;
    const unsigned count=s->NDTR;
    for(unsigned i=0;i<count;++i) {
        advanceBus();
        for(auto b:uartBases) if(p==b+offsetof(USART_TypeDef,DR)) {
            auto *u=reinterpret_cast<USART_TypeDef*>(b);
            if(direction==0&&(!(u->CR3&USART_CR3_DMAR)||!(u->SR&USART_SR_RXNE)))
                throw std::logic_error("UART DMA receive without request/data");
            if(direction==1&&!(u->CR3&USART_CR3_DMAT)) throw std::logic_error("UART DMA transmit without request");
        }
        for(auto b:adcBases) if(p==b+offsetof(ADC_TypeDef,DR)) {
            auto *u=reinterpret_cast<ADC_TypeDef*>(b);
            if(direction!=0||!(u->CR2&ADC_CR2_DMA)||!(u->SR&ADC_SR_EOC))
                throw std::logic_error("ADC DMA without enabled conversion data");
        }
        for(auto b:i2cBases) if(p==b+offsetof(I2C_TypeDef,DR)) {
            auto *u=reinterpret_cast<I2C_TypeDef*>(b);
            if(!(u->CR2&I2C_CR2_DMAEN)||(u->SR1&I2C_SR1_ADDR))
                throw std::logic_error("I2C DMA before request/address acknowledgement");
            if(direction==0&&!(u->SR1&I2C_SR1_RXNE)) throw std::logic_error("I2C DMA receive without data");
            if(direction==1&&!(u->SR1&I2C_SR1_TXE)) throw std::logic_error("I2C DMA transmit without ready");
        }
        for(auto b:spiBases) if(p==b+offsetof(SPI_TypeDef,DR)) {
            auto *u=reinterpret_cast<SPI_TypeDef*>(b);
            if(direction==0&&(!(u->CR2&SPI_CR2_RXDMAEN)||!(u->SR&SPI_SR_RXNE)))
                throw std::logic_error("SPI DMA receive without request/data");
            if(direction==1&&!(u->CR2&SPI_CR2_TXDMAEN)) throw std::logic_error("SPI DMA transmit without request");
        }
        if(direction!=1) { beforeRead(p); store(m,memoryWidth,load(p,peripheralWidth)); }
        else { uint64_t old=load(p,peripheralWidth),value=load(m,memoryWidth); store(p,peripheralWidth,value); effect(p,peripheralWidth,old,value); }
        if(s->CR&DMA_SxCR_MINC) m+=memoryWidth;
        if(s->CR&DMA_SxCR_PINC) p+=peripheralWidth;
    }
    if(s->CR&DMA_SxCR_DBM) { s->NDTR=count; s->CR=s->CR^DMA_SxCR_CT; }
    else if(s->CR&DMA_SxCR_CIRC) s->NDTR=count;
    else { s->NDTR=0; s->CR=s->CR&~DMA_SxCR_EN; }
    uintptr_t instance=reinterpret_cast<uintptr_t>(s),base=instance&~uintptr_t(0x3ff);
    unsigned stream=(instance-(base+0x10))/0x18; const unsigned shift[]={0,6,16,22};
    uintptr_t status=base+(stream>=4?4:0); store(status,4,load(status,4)|(DMA_FLAG_TCIF0_4<<shift[stream%4]));
}
void advanceBus() {
    for(unsigned i=0;i<sizeof(uartBases)/sizeof(*uartBases);++i) if(uartPending[i]) {
        auto *r=reinterpret_cast<USART_TypeDef*>(uartBases[i]); r->SR=r->SR|USART_SR_TXE|USART_SR_TC; uartPending[i]=false;
    }
    for(unsigned i=0;i<sizeof(spiBases)/sizeof(*spiBases);++i) if(spiPending[i]) {
        auto *r=reinterpret_cast<SPI_TypeDef*>(spiBases[i]); r->SR=(r->SR|SPI_SR_TXE)&~SPI_SR_BSY; spiPending[i]=false;
    }
    if(flashPending) { FLASH->SR=(FLASH->SR&~FLASH_SR_BSY)|FLASH_SR_EOP; flashPending=false; }
}
}
extern "C" {
void __asan_handle_no_return(void) { native::flush(); }
#define ACCESS(N) \
void __asan_load##N##_noabort(uintptr_t a) { native::flush(); if(peripheral(a)) beforeRead(a); } \
void __asan_store##N##_noabort(uintptr_t a) { native::flush(); if(peripheral(a)||(a>=FLASH_BASE&&a<FLASH_BASE+0x200000U)) { if(N>8) fault("wide peripheral access"); pending={a,N,load(a,N)}; } }
ACCESS(1) ACCESS(2) ACCESS(4) ACCESS(8) ACCESS(16)
#undef ACCESS
void __asan_loadN_noabort(uintptr_t a,size_t) { native::flush(); if(peripheral(a)) beforeRead(a); }
void __asan_storeN_noabort(uintptr_t a,size_t n) { native::flush(); if(peripheral(a)) { if(n>8) fault("wide peripheral access"); pending={a,static_cast<unsigned>(n),load(a,n)}; } }
void *__asan_memcpy(void *d,const void *s,size_t n) { native::flush(); return std::memcpy(d,s,n); }
void *__asan_memset(void *d,int c,size_t n) { native::flush(); return std::memset(d,c,n); }
void *__asan_memmove(void *d,const void *s,size_t n) { native::flush(); return std::memmove(d,s,n); }
}
