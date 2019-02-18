
#include "../can.h"

#define MY_CAN_ID       CAN_ID_4

uint8_t __attribute__((coherent, aligned(16))) MessageFifoArea[2 * 4 * 16];

void can_set_rx_id(int32_t rx_id) {
	can_write_rx_accept_filter(0, rx_id, 1, 15, 0);
	can_write_rx_accept_mask(0, 0x1FFFFFFF, 1, 1);
}

uint16_t* BOOT can_get_packet() {
	
	CAN_CHANNEL_EVENT ChannelEvent = PLIB_CAN_ChannelEventGet( MY_CAN_ID , CAN_CHANNEL1 );

    if( ChannelEvent & CAN_RX_CHANNEL_NOT_EMPTY )
    {
		CAN_RX_MSG_BUFFER * receivedMsg = PLIB_CAN_ReceivedMessageGet(MY_CAN_ID, CAN_CHANNEL1);
		if (receivedMsg) {
			PLIB_CAN_ChannelUpdate(MY_CAN_ID, CAN_CHANNEL1);
		}
		return (uint16_t*)receivedMsg;
	} else {
		return 0;
	}
}

uint16_t* can_get_free_tx_buffer() {
	CAN_TX_MSG_BUFFER * msgBuffer = PLIB_CAN_TransmitBufferGet(MY_CAN_ID, CAN_CHANNEL0);
	return (uint16_t*)msgBuffer;
}

void can_send_tx_buffer(uint16_t* buf) {
	PLIB_CAN_ChannelUpdate(MY_CAN_ID, CAN_CHANNEL0);
	PLIB_CAN_TransmitChannelFlush(MY_CAN_ID, CAN_CHANNEL0);
}

void can_write_rx_accept_filter(int n, int32_t identifier, uint16_t exide, uint16_t bufPnt,uint16_t maskSel) {}
void can_disable_rx_accept_filter(int n) {}
void can_write_rx_accept_mask(int m, int32_t identifier, uint16_t mide, uint16_t exide) {}

void can_init_pins(void) {
	PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
	PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_PPS_REGISTERS);
	TRISEbits.TRISE15 = 1;
	TRISAbits.TRISA8 = 0;
	ANSELEbits.ANSE15 = 0;
	PLIB_PORTS_RemapInput(PORTS_ID_0, INPUT_FUNC_C4RX, INPUT_PIN_RPE15);
	PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_C4TX, OUTPUT_PIN_RPA8);
	// C4RXRbits.C4RXR = 0x08; // 8 => RPE15
	// RPA8Rbits.RPA8R = 0x0c; // C4TX
	
	// PLIB_DEVCON_SystemLock(DEVCON_ID_0);
	// PLIB_DEVCON_DeviceRegistersLock(DEVCON_ID_0, DEVCON_PPS_REGISTERS);
}


void can_set_mode(int mode) {
	// C1CONbits.REQOP=mode;
	// while(C1CONbits.OPMOD!=mode);
	
	PLIB_CAN_OperationModeSelect(MY_CAN_ID, mode);
	// if (CAN_CONFIGURATION_MODE == mode)
	while(PLIB_CAN_OperationModeGet(MY_CAN_ID) != mode);
}


static void can_init_clk() {
	// CAN Baud Rate Configuration
	
    #define PRESCALE        5
    #define SYNCJUMPWIDTH   0
    #define PROPAGATION     2+4
    #define SEGMENT1        3+3
    #define SEGMENT2        3+2
    #define CAN_CLOCK       500000ul
    #define SYS_CLOCK       120000000ul

    uint32_t baudRate;
    bool result;

	PLIB_CAN_PhaseSegment2LengthFreelyProgrammableEnable(MY_CAN_ID);

    // //Set the Baud rate to 1000 kbps
    PLIB_CAN_PropagationTimeSegmentSet(MY_CAN_ID, 7-1);
    PLIB_CAN_PhaseSegment1LengthSet(MY_CAN_ID, 4-1);
    PLIB_CAN_PhaseSegment2LengthSet(MY_CAN_ID, 3-1);
    PLIB_CAN_SyncJumpWidthSet(MY_CAN_ID, 2-1);
    // PLIB_CAN_BaudRatePrescaleSet(MY_CAN_ID, 4-1); // set to 1 higher then ECAN tool
    PLIB_CAN_BaudRatePrescaleSet(MY_CAN_ID, 8-1); // set to 1 higher then ECAN tool
}

void can_init(int can_id, int use_eid) {
	can_set_default_id(can_id, 1, 0);
	can_init_pins();
	
	PLIB_CAN_Enable(MY_CAN_ID);
	while(C4CONbits.ON != 1);

	// config mode
	PLIB_CAN_OperationModeSelect(MY_CAN_ID, CAN_CONFIGURATION_MODE);
    while(PLIB_CAN_OperationModeGet(MY_CAN_ID) != CAN_CONFIGURATION_MODE);

	// init CAN clock
	can_init_clk();
	
	// setup channels
    PLIB_CAN_MemoryBufferAssign(MY_CAN_ID, MessageFifoArea);
    PLIB_CAN_ChannelForTransmitSet(MY_CAN_ID, CAN_CHANNEL0, 2, CAN_TX_RTR_DISABLED, CAN_LOW_MEDIUM_PRIORITY);
    PLIB_CAN_ChannelForReceiveSet(MY_CAN_ID, CAN_CHANNEL1, 2, CAN_RX_FULL_RECEIVE);
	
	// setup filter
	// PLIB_CAN_FilterConfigure(MY_CAN_ID, CAN_FILTER0, 0x80000000 | can_id, CAN_EID);
	PLIB_CAN_FilterConfigure(MY_CAN_ID, CAN_FILTER0, can_id, CAN_EID);
	// PLIB_CAN_FilterMaskConfigure(MY_CAN_ID, CAN_FILTER_MASK0, 0x1FFFFFFF, CAN_EID, CAN_FILTER_MASK_IDE_TYPE);
	PLIB_CAN_FilterMaskConfigure(MY_CAN_ID, CAN_FILTER_MASK0, 0x1FFFFFFF, CAN_EID, CAN_FILTER_MASK_ANY_TYPE);
	PLIB_CAN_FilterToChannelLink(MY_CAN_ID, CAN_FILTER0, CAN_FILTER_MASK0, CAN_CHANNEL1);
	PLIB_CAN_FilterEnable(MY_CAN_ID, CAN_FILTER0);
	
	// back to normal mode
    PLIB_CAN_OperationModeSelect(MY_CAN_ID, CAN_NORMAL_MODE);
    while(PLIB_CAN_OperationModeGet(MY_CAN_ID) != CAN_NORMAL_MODE);
}

void can_pkt_set_id(uint16_t* pkt, int32_t txIdentifier, uint16_t eid, uint16_t remoteTransmit) {
	CAN_TX_MSG_BUFFER* pkt2 = (CAN_TX_MSG_BUFFER*)pkt;
	if (eid) {
		pkt2->msgSID.sid = txIdentifier >> 18;
		pkt2->msgEID.eid = txIdentifier & 0x3ffff;
	} else {
		pkt2->msgSID.sid = txIdentifier & 0x7ff;
	}
	pkt2->msgEID.ide = eid;
	pkt2->msgEID.remote_request = remoteTransmit;
}


// get data pointer and length (or set length if dlen != 0)
uint8_t* can_pkt_get_data(uint16_t* pkt, int8_t* dlen) {
	CAN_TX_MSG_BUFFER* pkt2 = (CAN_TX_MSG_BUFFER*)pkt;
	if (dlen) {
		if(*dlen != 0) {
			pkt2->msgEID.data_length_code = *dlen;
		} else {
			*dlen = pkt2->msgEID.data_length_code;
		}
	}
	return (uint8_t*)&pkt[4];
}

void _C1Interrupt(void)
{    
}

