#include "native.h"
namespace {
unsigned pendingCalls;
unsigned sofCalls,resetCalls,inCalls,outCalls;uint8_t lastEndpoint;
void pendingCallback() { ++pendingCalls; }
struct Usb : native::Fixture {
    PCD_HandleTypeDef h{};
    Usb() {
        sofCalls=resetCalls=inCalls=outCalls=0;lastEndpoint=0xff;
        h.Instance=USB_OTG_FS; h.Init.dev_endpoints=4; h.Init.speed=PCD_SPEED_FULL;
        h.Init.phy_itface=PCD_PHY_EMBEDDED; h.Init.Sof_enable=ENABLE;
        h.Instance->GRSTCTL=USB_OTG_GRSTCTL_AHBIDL;
    }
    USB_OTG_DeviceTypeDef *device() { return reinterpret_cast<USB_OTG_DeviceTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_DEVICE_BASE); }
    USB_OTG_INEndpointTypeDef *in(unsigned ep) { return reinterpret_cast<USB_OTG_INEndpointTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_IN_ENDPOINT_BASE+ep*USB_OTG_EP_REG_SIZE); }
    USB_OTG_OUTEndpointTypeDef *out(unsigned ep) { return reinterpret_cast<USB_OTG_OUTEndpointTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_OUT_ENDPOINT_BASE+ep*USB_OTG_EP_REG_SIZE); }
    void busReset() {
        h.Instance->GINTSTS=USB_OTG_GINTSTS_USBRST;h.Instance->GINTMSK=h.Instance->GINTMSK|USB_OTG_GINTSTS_USBRST;
        HAL_PCD_IRQHandler(&h);native::flush();
    }
};
}
extern "C" void HAL_PCD_SOFCallback(PCD_HandleTypeDef*) { ++sofCalls; }
extern "C" void HAL_PCD_ResetCallback(PCD_HandleTypeDef*) { ++resetCalls; }
extern "C" void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef*,uint8_t ep) { ++inCalls;lastEndpoint=ep; }
extern "C" void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef*,uint8_t ep) { ++outCalls;lastEndpoint=ep; }
TEST_SUITE("P12NativeEXTI") {
TEST_CASE_FIXTURE(native::Fixture,"EXTI line configuration sets a selected port trigger and interrupt mask") {
    EXTI_HandleTypeDef h{}; EXTI_ConfigTypeDef c{};
    c.Line=EXTI_LINE_5; c.Mode=EXTI_MODE_INTERRUPT; c.Trigger=EXTI_TRIGGER_RISING_FALLING; c.GPIOSel=EXTI_GPIOC;
    REQUIRE(HAL_EXTI_SetConfigLine(&h,&c)==HAL_OK);
    CHECK((EXTI->IMR&(1U<<5))!=0); CHECK((EXTI->RTSR&(1U<<5))!=0); CHECK((EXTI->FTSR&(1U<<5))!=0);
    CHECK(((SYSCFG->EXTICR[1]>>4)&15)==2);
    EXTI_ConfigTypeDef actual{}; REQUIRE(HAL_EXTI_GetConfigLine(&h,&actual)==HAL_OK);
    CHECK(actual.Line==c.Line); CHECK(actual.Mode==c.Mode); CHECK(actual.Trigger==c.Trigger); CHECK(actual.GPIOSel==c.GPIOSel);
}
TEST_CASE_FIXTURE(native::Fixture,"EXTI pending clear preserves a different pending line") {
    EXTI_HandleTypeDef h{}; REQUIRE(HAL_EXTI_GetHandle(&h,EXTI_LINE_5)==HAL_OK);
    EXTI->PR=(1U<<5)|(1U<<7);
    HAL_EXTI_ClearPending(&h,EXTI_TRIGGER_RISING_FALLING); native::flush();
    CHECK(EXTI->PR==(1U<<7)); CHECK(HAL_EXTI_GetPending(&h,EXTI_TRIGGER_RISING_FALLING)==0);
}
TEST_CASE_FIXTURE(native::Fixture,"EXTI pending handler calls the registered callback once") {
    pendingCalls=0; EXTI_HandleTypeDef h{}; REQUIRE(HAL_EXTI_GetHandle(&h,EXTI_LINE_5)==HAL_OK);
    REQUIRE(HAL_EXTI_RegisterCallback(&h,HAL_EXTI_COMMON_CB_ID,pendingCallback)==HAL_OK);
    EXTI->PR=1U<<5; HAL_EXTI_IRQHandler(&h); native::flush();
    CHECK(pendingCalls==1); CHECK((EXTI->PR&(1U<<5))==0);
    HAL_EXTI_IRQHandler(&h); CHECK(pendingCalls==1);
}
TEST_CASE_FIXTURE(native::Fixture,"EXTI clear configuration removes the selected event without changing its neighbor") {
    EXTI_HandleTypeDef h{}; REQUIRE(HAL_EXTI_GetHandle(&h,EXTI_LINE_5)==HAL_OK);
    EXTI->IMR=(1U<<5)|(1U<<7); EXTI->RTSR=(1U<<5)|(1U<<7);
    CHECK(HAL_EXTI_ClearConfigLine(&h)==HAL_OK);
    CHECK(EXTI->IMR==(1U<<7)); CHECK(EXTI->RTSR==(1U<<7));
}
}
TEST_SUITE("P12NativeUSB") {
TEST_CASE_FIXTURE(Usb,"PCD initialization configures full speed and independent endpoint directions") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);
    CHECK(h.State==HAL_PCD_STATE_READY); CHECK(h.IN_ep[1].is_in==1); CHECK(h.OUT_ep[1].is_in==0);
    CHECK((device()->DCFG&USB_OTG_DCFG_DSPD)==USB_OTG_SPEED_FULL);
}
TEST_CASE_FIXTURE(Usb,"PCD initialization rejects a null handle") { CHECK(HAL_PCD_Init(nullptr)==HAL_ERROR); }
TEST_CASE_FIXTURE(Usb,"PCD FIFO allocation derives nonoverlapping word offsets") {
    REQUIRE(HAL_PCDEx_SetRxFiFo(&h,128)==HAL_OK);
    REQUIRE(HAL_PCDEx_SetTxFiFo(&h,0,32)==HAL_OK);
    REQUIRE(HAL_PCDEx_SetTxFiFo(&h,1,64)==HAL_OK);
    CHECK(h.Instance->GRXFSIZ==128); CHECK(h.Instance->DIEPTXF0_HNPTXFSIZ==((32U<<16)|128));
    CHECK(h.Instance->DIEPTXF[0]==((64U<<16)|160));
}
TEST_CASE_FIXTURE(Usb,"PCD device address changes only the address field") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK); REQUIRE(HAL_PCD_SetAddress(&h,37)==HAL_OK);
    CHECK(((device()->DCFG&USB_OTG_DCFG_DAD)>>4)==37); CHECK(h.USB_Address==37);
}
TEST_CASE_FIXTURE(Usb,"PCD bulk endpoint open programs direction type and packet size") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK); REQUIRE(HAL_PCD_EP_Open(&h,0x81,64,EP_TYPE_BULK)==HAL_OK);
    CHECK(h.IN_ep[1].maxpacket==64); CHECK((in(1)->DIEPCTL&USB_OTG_DIEPCTL_MPSIZ)==64);
    CHECK((in(1)->DIEPCTL&USB_OTG_DIEPCTL_USBAEP)!=0);
    CHECK(HAL_PCD_EP_Close(&h,0x81)==HAL_OK);
}
TEST_CASE_FIXTURE(Usb,"PCD output receive rounds the hardware packet request while retaining buffer ownership") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK); REQUIRE(HAL_PCD_EP_Open(&h,1,64,EP_TYPE_BULK)==HAL_OK);
    uint8_t buffer[64]{}; REQUIRE(HAL_PCD_EP_Receive(&h,1,buffer,17)==HAL_OK);
    CHECK(h.OUT_ep[1].xfer_buff==buffer); CHECK(h.OUT_ep[1].xfer_len==17);
    CHECK((out(1)->DOEPTSIZ&USB_OTG_DOEPTSIZ_XFRSIZ)==64);
}
TEST_CASE_FIXTURE(Usb,"PCD input transfer programs independent byte and packet counts") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK); REQUIRE(HAL_PCD_EP_Open(&h,0x81,64,EP_TYPE_BULK)==HAL_OK);
    uint8_t buffer[80]{}; REQUIRE(HAL_PCD_EP_Transmit(&h,0x81,buffer,80)==HAL_OK);
    CHECK((in(1)->DIEPTSIZ&USB_OTG_DIEPTSIZ_XFRSIZ)==80);
    CHECK(((in(1)->DIEPTSIZ&USB_OTG_DIEPTSIZ_PKTCNT)>>19)==2);
}
TEST_CASE_FIXTURE(Usb,"PCD stall state is set and cleared on the selected direction only") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK); REQUIRE(HAL_PCD_EP_Open(&h,0x81,64,EP_TYPE_BULK)==HAL_OK);
    CHECK(HAL_PCD_EP_SetStall(&h,0x81)==HAL_OK); CHECK(h.IN_ep[1].is_stall==1);
    CHECK((in(1)->DIEPCTL&USB_OTG_DIEPCTL_STALL)!=0); CHECK(h.OUT_ep[1].is_stall==0);
    CHECK(HAL_PCD_EP_ClrStall(&h,0x81)==HAL_OK); CHECK(h.IN_ep[1].is_stall==0);
}
TEST_CASE_FIXTURE(Usb,"PCD suspended remote wakeup requests preserve other device control fields") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK); device()->DCTL=USB_OTG_DCTL_SDIS; device()->DSTS=USB_OTG_DSTS_SUSPSTS;
    CHECK(HAL_PCD_ActivateRemoteWakeup(&h)==HAL_OK); CHECK((device()->DCTL&USB_OTG_DCTL_RWUSIG)!=0);
    CHECK(HAL_PCD_DeActivateRemoteWakeup(&h)==HAL_OK); CHECK((device()->DCTL&USB_OTG_DCTL_RWUSIG)==0);
}
TEST_CASE_FIXTURE(Usb,"PCD does not signal remote wakeup on a nonsuspended bus") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);
    CHECK(HAL_PCD_ActivateRemoteWakeup(&h)==HAL_OK); CHECK((device()->DCTL&USB_OTG_DCTL_RWUSIG)==0);
}
TEST_CASE_FIXTURE(Usb,"PCD connect and disconnect control the soft-disconnect request") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);
    CHECK(HAL_PCD_DevConnect(&h)==HAL_OK); CHECK((device()->DCTL&USB_OTG_DCTL_SDIS)==0);
    CHECK(HAL_PCD_DevDisconnect(&h)==HAL_OK); CHECK((device()->DCTL&USB_OTG_DCTL_SDIS)!=0);
}
TEST_CASE_FIXTURE(Usb,"USB packet writer emits independent little-endian FIFO words") {
    uint8_t bytes[8]={1,2,3,4,0x81,0x82,0x83,0x84};
    REQUIRE(USB_WritePacket(h.Instance,bytes,1,8,0)==HAL_OK);
    std::vector<uint64_t> values;
    uintptr_t fifo=USB_OTG_FS_PERIPH_BASE+USB_OTG_FIFO_BASE+USB_OTG_FIFO_SIZE;
    for(const auto &w:native::writes()) if(w.address==fifo) values.push_back(w.value);
    REQUIRE(values.size()==2); CHECK(values[0]==0x04030201); CHECK(values[1]==0x84838281);
}
TEST_CASE_FIXTURE(Usb,"USB packet reader copies the supplied FIFO word to the caller buffer") {
    *reinterpret_cast<uint32_t*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_FIFO_BASE)=0xc3b2a190;
    uint8_t bytes[4]{}; USB_ReadPacket(h.Instance,bytes,4);
    CHECK(bytes[0]==0x90); CHECK(bytes[1]==0xa1); CHECK(bytes[2]==0xb2); CHECK(bytes[3]==0xc3);
}
TEST_CASE_FIXTURE(Usb,"USB global interrupt mask can be enabled and disabled") {
    CHECK(USB_EnableGlobalInt(h.Instance)==HAL_OK); CHECK((h.Instance->GAHBCFG&USB_OTG_GAHBCFG_GINT)!=0);
    CHECK(USB_DisableGlobalInt(h.Instance)==HAL_OK); CHECK((h.Instance->GAHBCFG&USB_OTG_GAHBCFG_GINT)==0);
}
TEST_CASE_FIXTURE(Usb,"USB partial-word packet read preserves the byte beyond the requested length") {
    native::usbInput(0,{0xc3b2a190}); uint8_t guard[8];std::fill(std::begin(guard),std::end(guard),0x5a);
    USB_ReadPacket(h.Instance,guard,3);
    REQUIRE(guard[0]==0x90);REQUIRE(guard[1]==0xa1);REQUIRE(guard[2]==0xb2);
    CHECK(guard[3]==0x5a);
}
TEST_CASE_FIXTURE(Usb,"PCD SOF interrupt delivers one callback and acknowledges its global flag") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);h.Instance->GINTSTS=USB_OTG_GINTSTS_SOF;
    HAL_PCD_IRQHandler(&h);native::flush();CHECK(sofCalls==1);CHECK((h.Instance->GINTSTS&USB_OTG_GINTSTS_SOF)==0);
}
TEST_CASE_FIXTURE(Usb,"PCD enumeration event reports reset using the supplied full-speed status") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);device()->DSTS=USB_OTG_SPEED_FULL<<1;
    h.Instance->GINTSTS=USB_OTG_GINTSTS_ENUMDNE;h.Instance->GINTMSK=h.Instance->GINTMSK|USB_OTG_GINTSTS_ENUMDNE;
    HAL_PCD_IRQHandler(&h);CHECK(resetCalls==1);CHECK(h.Init.speed==USBD_FS_SPEED);
}
TEST_CASE_FIXTURE(Usb,"PCD input endpoint completion dispatches the exact endpoint callback") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);busReset();REQUIRE(HAL_PCD_EP_Open(&h,0x82,64,EP_TYPE_BULK)==HAL_OK);
    device()->DAINT=1U<<2;in(2)->DIEPINT=USB_OTG_DIEPINT_XFRC;
    h.Instance->GINTSTS=USB_OTG_GINTSTS_IEPINT;h.Instance->GINTMSK=h.Instance->GINTMSK|USB_OTG_GINTSTS_IEPINT;
    HAL_PCD_IRQHandler(&h);CHECK(inCalls==1);CHECK(outCalls==0);CHECK(lastEndpoint==2);
}
TEST_CASE_FIXTURE(Usb,"PCD RX FIFO dispatch copies only the supplied complete packet word") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);REQUIRE(HAL_PCD_EP_Open(&h,1,64,EP_TYPE_BULK)==HAL_OK);
    uint8_t buffer[64]{};REQUIRE(HAL_PCD_EP_Receive(&h,1,buffer,64)==HAL_OK);
    native::usbInput(0,{0x78563412});
    h.Instance->GRXSTSP=1|(4U<<4)|(STS_DATA_UPDT<<17);
    h.Instance->GINTSTS=USB_OTG_GINTSTS_RXFLVL;h.Instance->GINTMSK=h.Instance->GINTMSK|USB_OTG_GINTSTS_RXFLVL;
    HAL_PCD_IRQHandler(&h);CHECK(buffer[0]==0x12);CHECK(buffer[1]==0x34);CHECK(buffer[2]==0x56);CHECK(buffer[3]==0x78);
    CHECK(h.OUT_ep[1].xfer_count==4);
}
TEST_CASE_FIXTURE(Usb,"USB global interrupt reader masks unrelated pending flags") {
    h.Instance->GINTSTS=USB_OTG_GINTSTS_SOF|USB_OTG_GINTSTS_USBRST;h.Instance->GINTMSK=USB_OTG_GINTSTS_SOF;
    CHECK(USB_ReadInterrupts(h.Instance)==USB_OTG_GINTSTS_SOF);
}
TEST_CASE_FIXTURE(Usb,"USB host channel configuration preserves device endpoint direction and packet size") {
    REQUIRE(USB_HC_Init(h.Instance,1,0x82,5,USB_OTG_SPEED_FULL,EP_TYPE_BULK,64)==HAL_OK);
    auto *channel=reinterpret_cast<USB_OTG_HostChannelTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_CHANNEL_BASE+USB_OTG_HOST_CHANNEL_SIZE);
    CHECK((channel->HCCHAR&USB_OTG_HCCHAR_MPSIZ)==64);
    CHECK(((channel->HCCHAR&USB_OTG_HCCHAR_DAD)>>22)==5);CHECK((channel->HCCHAR&USB_OTG_HCCHAR_EPDIR)!=0);
}
TEST_CASE_FIXTURE(Usb,"USB host DMA transfer rounds input packet size without replacing the buffer address") {
    USB_OTG_HCTypeDef c{};c.ch_num=1;c.ep_num=2;c.ep_is_in=1;c.dev_addr=5;c.speed=USB_OTG_SPEED_FULL;
    c.ep_type=EP_TYPE_BULK;c.max_packet=64;c.xfer_len=80;uint8_t buffer[128]{};c.xfer_buff=buffer;
    REQUIRE(USB_HC_Init(h.Instance,1,0x82,5,c.speed,c.ep_type,c.max_packet)==HAL_OK);
    REQUIRE(USB_HC_StartXfer(h.Instance,&c,1)==HAL_OK);
    auto *channel=reinterpret_cast<USB_OTG_HostChannelTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_CHANNEL_BASE+USB_OTG_HOST_CHANNEL_SIZE);
    CHECK((channel->HCTSIZ&USB_OTG_HCTSIZ_XFRSIZ)==128);CHECK(channel->HCDMA==reinterpret_cast<uint32_t>(buffer));
}
TEST_CASE_FIXTURE(Usb,"USB host clock selection requests a full-speed frame interval") {
    REQUIRE(USB_InitFSLSPClkSel(h.Instance,HCFG_48_MHZ)==HAL_OK);
    auto *host=reinterpret_cast<USB_OTG_HostTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_BASE);
    CHECK(host->HFIR==48000);
}
}
TEST_SUITE("P12NativeUSBControlLifecycle") {
TEST_CASE_FIXTURE(Usb,"PCD start and stop gate global interrupts and soft connection requests") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);REQUIRE(HAL_PCD_Start(&h)==HAL_OK);
    CHECK((h.Instance->GAHBCFG&USB_OTG_GAHBCFG_GINT)!=0);CHECK((device()->DCTL&USB_OTG_DCTL_SDIS)==0);
    REQUIRE(HAL_PCD_Stop(&h)==HAL_OK);CHECK((h.Instance->GAHBCFG&USB_OTG_GAHBCFG_GINT)==0);
    CHECK((device()->DCTL&USB_OTG_DCTL_SDIS)!=0);REQUIRE(HAL_PCD_DeInit(&h)==HAL_OK);
    CHECK(HAL_PCD_GetState(&h)==HAL_PCD_STATE_RESET);
}
TEST_CASE_FIXTURE(Usb,"PCD receive completion reports only its selected endpoint and actual received count") {
    REQUIRE(HAL_PCD_Init(&h)==HAL_OK);busReset();REQUIRE(HAL_PCD_EP_Open(&h,1,64,EP_TYPE_BULK)==HAL_OK);
    uint8_t data[64]{};REQUIRE(HAL_PCD_EP_Receive(&h,1,data,64)==HAL_OK);native::usbInput(0,{0x78563412});
    h.Instance->GRXSTSP=1|(4U<<4)|(STS_DATA_UPDT<<17);h.Instance->GINTSTS=USB_OTG_GINTSTS_RXFLVL;
    h.Instance->GINTMSK=h.Instance->GINTMSK|USB_OTG_GINTSTS_RXFLVL;HAL_PCD_IRQHandler(&h);native::flush();
    device()->DAINT=1U<<17;out(1)->DOEPINT=USB_OTG_DOEPINT_XFRC;
    h.Instance->GINTSTS=USB_OTG_GINTSTS_OEPINT;h.Instance->GINTMSK=h.Instance->GINTMSK|USB_OTG_GINTSTS_OEPINT;
    HAL_PCD_IRQHandler(&h);CHECK(outCalls==1);CHECK(inCalls==0);CHECK(lastEndpoint==1);
    CHECK(HAL_PCD_EP_GetRxCount(&h,1)==4);CHECK(data[0]==0x12);CHECK(data[3]==0x78);
}
TEST_CASE_FIXTURE(Usb,"USB device OUT interrupt readers apply endpoint and common interrupt masks") {
    device()->DAINT=(1U<<17)|(1U<<19);device()->DAINTMSK=1U<<19;
    CHECK(USB_ReadDevAllOutEpInterrupt(h.Instance)==8);
    out(3)->DOEPINT=USB_OTG_DOEPINT_XFRC|USB_OTG_DOEPINT_STUP;device()->DOEPMSK=USB_OTG_DOEPMSK_STUPM;
    CHECK(USB_ReadDevOutEPInterrupt(h.Instance,3)==USB_OTG_DOEPINT_STUP);
}
TEST_CASE_FIXTURE(Usb,"USB host initialization reserves independent receive and transmit FIFO regions") {
    h.Init.Host_channels=8;REQUIRE(USB_HostInit(h.Instance,h.Init)==HAL_OK);
    CHECK(h.Instance->GRXFSIZ==128);CHECK((h.Instance->DIEPTXF0_HNPTXFSIZ&65535)==128);
    CHECK((h.Instance->HPTXFSIZ&65535)==224);CHECK((h.Instance->GINTMSK&USB_OTG_GINTMSK_HCIM)!=0);
}
TEST_CASE_FIXTURE(Usb,"USB host VBUS control changes the port power request") {
    auto *port=reinterpret_cast<uint32_t*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_PORT_BASE);
    REQUIRE(USB_DriveVbus(h.Instance,1)==HAL_OK);CHECK((*port&USB_OTG_HPRT_PPWR)!=0);
    REQUIRE(USB_DriveVbus(h.Instance,0)==HAL_OK);CHECK((*port&USB_OTG_HPRT_PPWR)==0);
}
TEST_CASE_FIXTURE(Usb,"USB host frame and speed readers isolate their documented fields") {
    auto *host=reinterpret_cast<USB_OTG_HostTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_BASE);
    host->HFNUM=0x98761234;CHECK(USB_GetCurrentFrame(h.Instance)==0x1234);
    auto *port=reinterpret_cast<uint32_t*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_PORT_BASE);
    *port=2U<<17;CHECK(USB_GetHostSpeed(h.Instance)==2);
}
TEST_CASE_FIXTURE(Usb,"USB host channel reader returns aggregate status and masks individual channel sources") {
    auto *host=reinterpret_cast<USB_OTG_HostTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_BASE);
    auto *channel=reinterpret_cast<USB_OTG_HostChannelTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_CHANNEL_BASE+USB_OTG_HOST_CHANNEL_SIZE);
    host->HAINT=0x26;host->HAINTMSK=0x24;CHECK(USB_HC_ReadInterrupt(h.Instance)==0x26);
    channel->HCINT=USB_OTG_HCINT_XFRC|USB_OTG_HCINT_STALL;channel->HCINTMSK=USB_OTG_HCINTMSK_STALLM;
    CHECK(USB_ReadChInterrupts(h.Instance,1)==USB_OTG_HCINT_STALL);
}
TEST_CASE_FIXTURE(Usb,"USB host ping requests one packet and enables the selected channel") {
    auto *channel=reinterpret_cast<USB_OTG_HostChannelTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_CHANNEL_BASE+USB_OTG_HOST_CHANNEL_SIZE);
    REQUIRE(USB_DoPing(h.Instance,1)==HAL_OK);CHECK((channel->HCTSIZ&USB_OTG_HCTSIZ_DOPING)!=0);
    CHECK(((channel->HCTSIZ&USB_OTG_HCTSIZ_PKTCNT)>>19)==1);CHECK((channel->HCCHAR&USB_OTG_HCCHAR_CHENA)!=0);
}
TEST_CASE_FIXTURE(Usb,"USB host channel halt requests disable for the selected channel") {
    auto *channel=reinterpret_cast<USB_OTG_HostChannelTypeDef*>(USB_OTG_FS_PERIPH_BASE+USB_OTG_HOST_CHANNEL_BASE+USB_OTG_HOST_CHANNEL_SIZE);
    channel->HCCHAR=USB_OTG_HCCHAR_CHENA;REQUIRE(USB_HC_Halt(h.Instance,1)==HAL_OK);native::flush();
    CHECK((channel->HCCHAR&USB_OTG_HCCHAR_CHENA)==0);
}
}
