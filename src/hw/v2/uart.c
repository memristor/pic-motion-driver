#include "../uart.h"
#include "sys/attribs.h"


#define MY_USART USART_ID_3
static Packet* volatile tx_pkt_sending = 0;

void uart_putch(uint8_t c) {
	while(PLIB_USART_TransmitterBufferIsFull(MY_USART));
	PLIB_USART_TransmitterByteSend(MY_USART, c);
}

int8_t uart_getch2() {
	// if(PLIB_USART_ReceiverOverrunHasOccurred(MY_USART)) {
		// PLIB_USART_ReceiverOverrunErrorClear(MY_USART);
	// }
	while(!PLIB_USART_ReceiverDataIsAvailable(MY_USART));
	return PLIB_USART_ReceiverByteReceive(MY_USART);
}

void __ISR(_UART3_FAULT_VECTOR, IPL2SOFT) FaultInterrupt(void) {
	PLIB_USART_ReceiverOverrunErrorClear(MY_USART);
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_3_ERROR);
}

void __ISR(_UART3_RX_VECTOR, IPL2SOFT) RX3Interrupt(void) {
	int isError = PLIB_USART_ReceiverFramingErrorHasOccurred(MY_USART) |
				   PLIB_USART_ReceiverParityErrorHasOccurred(MY_USART) |
				   PLIB_USART_ReceiverOverrunHasOccurred(MY_USART);
	
	if(!isError && PLIB_USART_ReceiverDataIsAvailable(MY_USART)) {
		if(PLIB_USART_ReceiverDataIsAvailable(MY_USART)) {
			int8_t byte = PLIB_USART_ReceiverByteReceive(MY_USART);
			uart_push_char(byte);
		}
	}
	
	if(PLIB_USART_ReceiverOverrunHasOccurred(MY_USART)) {
		PLIB_USART_ReceiverOverrunErrorClear(MY_USART);
	}
	
	
	// clear interrupt flag
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_3_RECEIVE);
}


void __ISR(_UART3_TX_VECTOR, IPL2SOFT) TX3Interrupt(void) {
	
	// uart_putch('y');
	
	if(tx_pkt_sending != 0) {
		if(!PLIB_USART_TransmitterBufferIsFull(MY_USART)) {
			int8_t data = ((uint8_t*)tx_pkt_sending)[tx_pkt_sending->cursor];
			tx_pkt_sending->cursor++;
			PLIB_USART_TransmitterByteSend(MY_USART, data);
		}
		if(tx_pkt_sending->cursor >= tx_pkt_sending->size + PACKET_HEADER) {
			packet_free_packet(tx_pkt_sending);
			tx_pkt_sending = 0;
			uart_start_sending_packet(packet_sending_queue_pop());
		}
	}
	
	
	// clear interrupt flag
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_3_TRANSMIT);
}

void uart_init_pins(void) {
	// PLIB_PORTS_RemapInput(PORTS_ID_0, INPUT_FUNC_C4RX, INPUT_PIN_RPE15);
	// PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_C4TX, OUTPUT_PIN_RPA8);
	PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
	PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_PPS_REGISTERS);
	
	// TRISC |= (1<<6);
    // TRISC &= ~(1<<7);
	TRISC |= (1<<7);
    TRISC &= ~(1<<6);
    
    // U1RXRbits.U1RXR = 5;
    U3RXRbits.U3RXR = 5;
    // RPC7Rbits.RPC7R = 1;
    RPC6Rbits.RPC6R = 1;
}

void uart_init(long baud) {
	uart_init_pins();
	PLIB_USART_Disable(MY_USART);
	PLIB_USART_InitializeModeGeneral(MY_USART,
            false,  /*Auto baud*/
            false,  /*LoopBack mode*/
            false,  /*Auto wakeup on start*/
            false,  /*IRDA mode*/
            false);  /*Stop In Idle mode*/
    // PLIB_USART_LineControlModeSelect(MY_USART, DRV_USART_LINE_CONTROL_8NONE1);
    PLIB_USART_InitializeOperation(MY_USART,
            USART_RECEIVE_FIFO_ONE_CHAR,
            USART_TRANSMIT_FIFO_IDLE,
            USART_ENABLE_TX_RX_USED);
            
	PLIB_USART_OperationModeSelect(MY_USART, USART_ENABLE_TX_RX_USED);
	PLIB_USART_BaudSetAndEnable(MY_USART, 120000000ul, baud);
	// PLIB_USART_BaudSetAndEnable(MY_USART, 60000000ul, baud);
	// PLIB_USART_Enable(MY_USART);
	// PLIB_USART_BaudRateHighEnable(MY_USART);
	
	PLIB_USART_TransmitterEnable (MY_USART);
	PLIB_USART_ReceiverEnable (MY_USART);
	
	PLIB_USART_ReceiverInterruptModeSelect (MY_USART, USART_RECEIVE_FIFO_ONE_CHAR);
	PLIB_USART_TransmitterInterruptModeSelect (MY_USART, USART_TRANSMIT_FIFO_EMPTY);
	
	// SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_ERROR);
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_3_RECEIVE);
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_3_TRANSMIT);
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_3_ERROR);
	
	PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_3_RECEIVE);
	PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_3_TRANSMIT);
	PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_3_ERROR);
	
	PLIB_INT_VectorPrioritySet(INT_ID_0, _UART3_RX_VECTOR, INT_PRIORITY_LEVEL1);
	PLIB_INT_VectorSubPrioritySet(INT_ID_0, _UART3_RX_VECTOR, INT_SUBPRIORITY_LEVEL0);
	
	PLIB_INT_VectorPrioritySet(INT_ID_0, _UART3_TX_VECTOR, INT_PRIORITY_LEVEL1);
	PLIB_INT_VectorSubPrioritySet(INT_ID_0, _UART3_TX_VECTOR, INT_SUBPRIORITY_LEVEL0);
	PLIB_INT_Enable(INT_ID_0);
	
	// while(1) {}
	/*
	while(1)
	{
		int32_t cnt=0;
		for(cnt=0; cnt < 25000; cnt++);
		// int8_t b = uart_getch2();
		// int8_t b = uart_getch();
		// uart_putch(b);
	}
	*/
	
}

// ------------[ UART synchronous write ]---------------



// HW dependend implementation
void uart_start_sending_packet(Packet* p) {
	if((p == 0) || (tx_pkt_sending != 0)) return;
	
	tx_pkt_sending = p;
	tx_pkt_sending->status = sending;
	tx_pkt_sending->cursor = 0;
	
	// send byte by byte
	if(!PLIB_USART_TransmitterBufferIsFull(MY_USART)) {
		int8_t byte = ((uint8_t*)tx_pkt_sending)[tx_pkt_sending->cursor++];
		PLIB_USART_TransmitterByteSend(MY_USART, byte);
	}
}
