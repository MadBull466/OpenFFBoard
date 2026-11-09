#include "native.h"
#ifdef HAL_CAN_MODULE_ENABLED
namespace {
struct Can : native::Fixture {
    CAN_HandleTypeDef h{};
    Can() {
        h.Instance=CAN1; h.Init.Prescaler=8; h.Init.Mode=CAN_MODE_NORMAL;
        h.Init.SyncJumpWidth=CAN_SJW_1TQ; h.Init.TimeSeg1=CAN_BS1_6TQ; h.Init.TimeSeg2=CAN_BS2_1TQ;
        h.Init.AutoRetransmission=ENABLE;
        native::event([](uint32_t){
            CAN1->MSR=(CAN1->MSR&~(CAN_MSR_INAK|CAN_MSR_SLAK))|
                ((CAN1->MCR&CAN_MCR_INRQ)?CAN_MSR_INAK:0)|((CAN1->MCR&CAN_MCR_SLEEP)?CAN_MSR_SLAK:0);
        });
    }
};
}
TEST_SUITE("P12NativeCAN") {
TEST_CASE_FIXTURE(Can,"CAN initialization requests independently selected bitrate fields") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    CHECK((CAN1->BTR&CAN_BTR_BRP)==7); CHECK((CAN1->BTR&CAN_BTR_TS1)==CAN_BS1_6TQ);
    CHECK(h.State==HAL_CAN_STATE_READY);
}
TEST_CASE_FIXTURE(Can,"CAN start and stop wait for explicitly modeled initialization acknowledgement") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    CHECK(HAL_CAN_Start(&h)==HAL_OK); CHECK(h.State==HAL_CAN_STATE_LISTENING);
    CHECK(HAL_CAN_Stop(&h)==HAL_OK); CHECK(h.State==HAL_CAN_STATE_READY);
}
TEST_CASE_FIXTURE(Can,"CAN initialization times out without the acknowledgement event") {
    native::event({}); CHECK(HAL_CAN_Init(&h)!=HAL_OK); CHECK(h.State==HAL_CAN_STATE_ERROR);
}
TEST_CASE_FIXTURE(Can,"CAN standard transmit mailbox preserves identifier length and payload byte order") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK); CAN1->TSR=CAN_TSR_TME0;
    CAN_TxHeaderTypeDef header{}; header.StdId=0x321; header.IDE=CAN_ID_STD; header.RTR=CAN_RTR_DATA; header.DLC=3;
    uint8_t bytes[8]={0x81,0x7f,0xaa}; uint32_t mailbox=99;
    REQUIRE(HAL_CAN_AddTxMessage(&h,&header,bytes,&mailbox)==HAL_OK);
    CHECK(mailbox==CAN_TX_MAILBOX0); CHECK((CAN1->sTxMailBox[0].TIR>>21)==0x321);
    CHECK((CAN1->sTxMailBox[0].TDTR&15)==3); CHECK(CAN1->sTxMailBox[0].TDLR==0x00aa7f81);
    CHECK((CAN1->sTxMailBox[0].TIR&CAN_TI0R_TXRQ)!=0);
}
TEST_CASE_FIXTURE(Can,"CAN extended remote transmit uses the extended identifier field") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK); CAN1->TSR=CAN_TSR_TME0;
    CAN_TxHeaderTypeDef header{}; header.ExtId=0x1234567; header.IDE=CAN_ID_EXT; header.RTR=CAN_RTR_REMOTE; header.DLC=0;
    uint8_t bytes[8]{}; uint32_t mailbox=0;
    REQUIRE(HAL_CAN_AddTxMessage(&h,&header,bytes,&mailbox)==HAL_OK);
    CHECK((CAN1->sTxMailBox[0].TIR>>3)==0x1234567);
    CHECK((CAN1->sTxMailBox[0].TIR&(CAN_TI0R_IDE|CAN_TI0R_RTR))==(CAN_TI0R_IDE|CAN_TI0R_RTR));
}
TEST_CASE_FIXTURE(Can,"CAN receive extracts a standard identifier filter index and payload") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    CAN1->RF0R=1; CAN1->sFIFOMailBox[0].RIR=0x245U<<21;
    CAN1->sFIFOMailBox[0].RDTR=3|(7<<8); CAN1->sFIFOMailBox[0].RDLR=0x00c3b2a1;
    CAN_RxHeaderTypeDef header{}; uint8_t bytes[8]{};
    REQUIRE(HAL_CAN_GetRxMessage(&h,CAN_RX_FIFO0,&header,bytes)==HAL_OK);
    CHECK(header.StdId==0x245); CHECK(header.DLC==3); CHECK(header.FilterMatchIndex==7);
    CHECK(bytes[0]==0xa1); CHECK(bytes[1]==0xb2); CHECK(bytes[2]==0xc3);
}
TEST_CASE_FIXTURE(Can,"CAN empty receive FIFO rejects a request without fabricating payload") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK); CAN_RxHeaderTypeDef header{}; uint8_t bytes[8]={0x5a};
    CHECK(HAL_CAN_GetRxMessage(&h,CAN_RX_FIFO0,&header,bytes)==HAL_ERROR); CHECK(bytes[0]==0x5a);
}
TEST_CASE_FIXTURE(Can,"CAN filter configuration preserves independent identifier and mask words") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    CAN_FilterTypeDef f{}; f.FilterBank=3; f.FilterMode=CAN_FILTERMODE_IDMASK;
    f.FilterScale=CAN_FILTERSCALE_32BIT; f.FilterActivation=ENABLE; f.FilterFIFOAssignment=CAN_FILTER_FIFO0;
    f.FilterIdHigh=0x2460; f.FilterMaskIdHigh=0xffe0; f.SlaveStartFilterBank=14;
    REQUIRE(HAL_CAN_ConfigFilter(&h,&f)==HAL_OK);
    CHECK(CAN1->sFilterRegister[3].FR1==0x24600000); CHECK(CAN1->sFilterRegister[3].FR2==0xffe00000);
    CHECK((CAN1->FA1R&(1U<<3))!=0);
}
TEST_CASE_FIXTURE(Can,"CAN notification activation modifies only requested interrupt enables") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    CHECK(HAL_CAN_ActivateNotification(&h,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR)==HAL_OK);
    CHECK((CAN1->IER&CAN_IT_RX_FIFO0_MSG_PENDING)!=0);
    CHECK(HAL_CAN_DeactivateNotification(&h,CAN_IT_ERROR)==HAL_OK);
    CHECK((CAN1->IER&CAN_IT_RX_FIFO0_MSG_PENDING)!=0); CHECK((CAN1->IER&CAN_IT_ERROR)==0);
}
TEST_CASE_FIXTURE(Can,"CAN abort targets only the requested transmit mailbox") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    CHECK(HAL_CAN_AbortTxRequest(&h,CAN_TX_MAILBOX1)==HAL_OK);
    CHECK((CAN1->TSR&CAN_TSR_ABRQ1)!=0); CHECK((CAN1->TSR&CAN_TSR_ABRQ0)==0);
}
}
#endif
#ifdef HAL_CAN_MODULE_ENABLED
namespace {unsigned canTxDone[3],canAborted[3],canErrors,canPending[2];}
extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef*){++canTxDone[0];}
extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef*){++canTxDone[1];}
extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef*){++canTxDone[2];}
extern "C" void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef*){++canAborted[0];}
extern "C" void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef*){++canAborted[1];}
extern "C" void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef*){++canAborted[2];}
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef*){++canErrors;}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*){++canPending[0];}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef*){++canPending[1];}
TEST_SUITE("P12NativeCANEvents") {
TEST_CASE_FIXTURE(Can,"CAN transmit completion dispatches each mailbox callback and clears its request flag") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);REQUIRE(HAL_CAN_ActivateNotification(&h,CAN_IT_TX_MAILBOX_EMPTY)==HAL_OK);
    for(unsigned box=0;box<3;++box){
        canTxDone[box]=0;native::flush();CAN1->TSR=(CAN_TSR_RQCP0|CAN_TSR_TXOK0)<<(box*8);
        HAL_CAN_IRQHandler(&h);native::flush();CHECK(canTxDone[box]==1);CHECK(CAN1->TSR==0);
    }
}
TEST_CASE_FIXTURE(Can,"CAN aborted transmission dispatches each abort callback without reporting success") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);REQUIRE(HAL_CAN_ActivateNotification(&h,CAN_IT_TX_MAILBOX_EMPTY)==HAL_OK);
    for(unsigned box=0;box<3;++box){
        canAborted[box]=canTxDone[box]=0;native::flush();CAN1->TSR=CAN_TSR_RQCP0<<(box*8);
        HAL_CAN_IRQHandler(&h);native::flush();CHECK(canAborted[box]==1);CHECK(canTxDone[box]==0);
    }
}
TEST_CASE_FIXTURE(Can,"CAN arbitration-loss interrupt records the specific mailbox error") {
    canErrors=0;REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    REQUIRE(HAL_CAN_ActivateNotification(&h,CAN_IT_TX_MAILBOX_EMPTY)==HAL_OK);
    CAN1->TSR=CAN_TSR_RQCP1|CAN_TSR_ALST1;HAL_CAN_IRQHandler(&h);
    CHECK(canErrors==1);CHECK((HAL_CAN_GetError(&h)&HAL_CAN_ERROR_TX_ALST1)!=0);
    CHECK(HAL_CAN_ResetError(&h)==HAL_OK);CHECK(HAL_CAN_GetError(&h)==HAL_CAN_ERROR_NONE);
}
TEST_CASE_FIXTURE(Can,"CAN pending receive interrupt dispatches only its selected FIFO callback") {
    canPending[0]=canPending[1]=0;REQUIRE(HAL_CAN_Init(&h)==HAL_OK);
    REQUIRE(HAL_CAN_ActivateNotification(&h,CAN_IT_RX_FIFO1_MSG_PENDING)==HAL_OK);
    CAN1->RF0R=1;CAN1->RF1R=2;HAL_CAN_IRQHandler(&h);CHECK(canPending[0]==0);CHECK(canPending[1]==1);
    CHECK(HAL_CAN_GetRxFifoFillLevel(&h,CAN_RX_FIFO0)==1);CHECK(HAL_CAN_GetRxFifoFillLevel(&h,CAN_RX_FIFO1)==2);
}
TEST_CASE_FIXTURE(Can,"CAN mailbox availability counts independent empty-mailbox inputs") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);CAN1->TSR=CAN_TSR_TME0|CAN_TSR_TME2;
    CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h)==2);CHECK(HAL_CAN_IsTxMessagePending(&h,CAN_TX_MAILBOX1)==1);
    CHECK(HAL_CAN_IsTxMessagePending(&h,CAN_TX_MAILBOX0)==0);
}
TEST_CASE_FIXTURE(Can,"CAN wakeup reports timeout while the supplied sleep acknowledgement remains asserted") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);REQUIRE(HAL_CAN_RequestSleep(&h)==HAL_OK);
    HAL_GetTick();CHECK(HAL_CAN_IsSleepActive(&h)==1);REQUIRE(HAL_CAN_WakeUp(&h)==HAL_ERROR);
    CHECK((HAL_CAN_GetError(&h)&HAL_CAN_ERROR_TIMEOUT)!=0);CHECK(HAL_CAN_IsSleepActive(&h)==1);
}
}
#endif

#ifdef HAL_CAN_MODULE_ENABLED
TEST_CASE_FIXTURE(Can,"CAN wakeup clears a pending request when sleep is no longer acknowledged") {
    REQUIRE(HAL_CAN_Init(&h)==HAL_OK);REQUIRE(HAL_CAN_RequestSleep(&h)==HAL_OK);native::flush();
    CAN1->MSR=CAN1->MSR&~CAN_MSR_SLAK;
    REQUIRE(HAL_CAN_WakeUp(&h)==HAL_OK);CHECK((CAN1->MCR&CAN_MCR_SLEEP)==0);
    CHECK(HAL_CAN_IsSleepActive(&h)==0);
}
#endif
