#include "../uart.h"

// __attribute__((__interrupt__, no_auto_psv))
void INTERRUPT _U1RXInterrupt(void)
{
	static unsigned char status;
	
	status = U1STAbits.FERR | U1STAbits.PERR | U1STAbits.OERR;
	char uart_rx_data = U1RXREG;

	if(status == 0)
	{
		if(++uart_rx_index1 == RX_BUF_LEN) {
			uart_rx_index1 = 0;
		}
		uart_rx_buf[uart_rx_index1] = uart_rx_data;
		uart_rx_counter++;
	}

	U1STAbits.OERR = 0;
	IFS0bits.U1RXIF = 0;
}


// HW dependend implementation
void uart_init(long baud) {
	U1BRG = (double)FCY / (16 * baud) - 1;
	U1MODEbits.STSEL = 0;   // 1 Stop bit
	U1MODEbits.PDSEL = 0;   // No Parity, 8 data bits
	U1MODEbits.BRGH = 0;    // Low-Speed mode

	U1STAbits.URXISEL = 0; // rx interrupt for each received byte
	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;

	IEC0bits.U1RXIE = 1;
	IEC0bits.U1TXIE = 1;
	
	IPC2bits.U1RXIP = 6;	// Priority must be less than 7, otherwise it can't be temporarily disabled
	IPC3bits.U1TXIP = 6;	// Priority must be less than 7, otherwise it can't be temporarily disabled

	uart_rx_index1 = uart_rx_index2 = uart_rx_counter = 0;

	U1MODEbits.UARTEN = 1;  // Enable UART
	U1STAbits.UTXEN = 1;	
}

static Packet* volatile tx_pkt_sending = 0;
void uart_start_sending_packet(Packet* p) {
	if((p == 0) || (tx_pkt_sending != 0)) return;
	tx_pkt_sending = p;
	tx_pkt_sending->status = sending;
	tx_pkt_sending->cursor = 0;
	if(U1STAbits.UTXBF == 0) {
		U1TXREG = ((uint8_t*)tx_pkt_sending)[tx_pkt_sending->cursor];
		tx_pkt_sending->cursor++;
	}
}

void __attribute__((__interrupt__,no_auto_psv)) _U1TXInterrupt(void) {
	IFS0bits.U1TXIF = 0;
	if(tx_pkt_sending != 0) {
		if(U1STAbits.UTXBF == 0) {
			U1TXREG = ((uint8_t*)tx_pkt_sending)[tx_pkt_sending->cursor];
			tx_pkt_sending->cursor++;
		}
		if(tx_pkt_sending->cursor >= tx_pkt_sending->size + PACKET_HEADER) {
			packet_free_packet(tx_pkt_sending);
			tx_pkt_sending = 0;
			uart_start_sending_packet(packet_sending_queue_pop());
		}
	}
}

void uart_putch(unsigned char c) {
	while(U1STAbits.UTXBF);
	U1TXREG = c;
}

void uart_init_pins(void) {
	RPINR18bits.U1RXR = 0;		//UART1 RX -> RP0- pin 4
	RPOR0bits.RP1R = 3;			//UART1 TX -> RP1- pin 5
}
