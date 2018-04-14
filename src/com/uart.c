#include "uart.h"
#include <p33FJ128MC802.h>
#include <libpic30.h>

// #define DISI(n) do { asm volatile ("disi #%0" : : "i"(n)); } while (0)

static volatile uint8_t rx_buf[RX_BUF_LEN];
static volatile uint8_t rx_index1 = 0; // new data is put at this offset
static volatile uint8_t rx_index2 = 0; // data is read from this offset
static volatile uint8_t rxData;
static volatile uint8_t rxCounter = 0;

void uart_init(long baud)
{
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

	rx_index1 = rx_index2 = rxCounter = 0;

	U1MODEbits.UARTEN = 1;  // Enable UART
	U1STAbits.UTXEN = 1;	
}

inline char uart_busy(void)
{  
	return !U1STAbits.TRMT;
}

void uart_close(void)
{  
	U1MODEbits.UARTEN = 0;

	IEC0bits.U1RXIE = 0;
	IEC0bits.U1TXIE = 0;

	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;
}

// ------------[ UART synchronous write ]---------------

void uart_putch(unsigned char c)
{
	while(U1STAbits.UTXBF);
	U1TXREG = c;
}

void uart_putint16(unsigned int s) {
	while(U1STAbits.UTXBF);
	
	U1TXREG = s >> 8;
	
	while(U1STAbits.UTXBF);
	
	U1TXREG = s;
}

void uart_init_pins(void) {
	RPINR18bits.U1RXR = 0;		//UART1 RX -> RP0- pin 4
	RPOR0bits.RP1R = 3;			//UART1 TX -> RP1- pin 5
}

void uart_putstr(const char* s) {
	while(*s) {
		uart_putch(*s);
		s++;
	}
}

// ------------[ UART synchronous read ]--------------
/*
 * Don't use from interrupt
 */
unsigned char uart_getch(void)
{
	while(rxCounter == 0);
	SRbits.IPL = 7;
	rxCounter--;
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
	SRbits.IPL = 0;
	return rx_buf[rx_index2];
}

/*
 * Don't use from interrupt
 */
unsigned int uart_getint16(void) {
	unsigned int ret;
	
	while(rxCounter < 2);
	
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
	
	ret = rx_buf[rx_index2] << 8;
	
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
		
	ret |= rx_buf[rx_index2];
	
	SRbits.IPL = 7;
	rxCounter-=2;
	SRbits.IPL = 0;
	return ret;
}

// ------------[ UART peek ]-------------

unsigned char uart_get_last_byte(void)
{
	return rx_buf[rx_index1];
}

unsigned char uart_check_rx(void)
{
	return rxCounter;
}

/*
 * Don't use from interrupt
 */
void uart_flush(void) {
	SRbits.IPL = 7;
	rxCounter = 0;
	rx_index2 = rx_index1;
	SRbits.IPL = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
	static unsigned char status;
	
	status = U1STAbits.FERR | U1STAbits.PERR | U1STAbits.OERR;
	rxData = U1RXREG;

	if(status == 0)
	{
		if(++rx_index1 == RX_BUF_LEN) {
			rx_index1 = 0;
		}
		rx_buf[rx_index1] = rxData;
		rxCounter++;
	}

	U1STAbits.OERR = 0;
	
	IFS0bits.U1RXIF = 0;
}

// ------------[ RX PACKET ]--------------

static uint8_t rx_get(void) {
	if(++rx_index2 >= RX_BUF_LEN) {
		rx_index2 = 0;
	}
	return rx_buf[rx_index2];
}

#define PACKET_SYNC 0x3c
static Packet rx_pkt;
static uint8_t rx_pkt_wait_for_data = 0;


/*
 * Don't use from interrupt
 */
Packet* uart_try_read_packet(void) {
	uint8_t read = 0;
	uint8_t pass = 0;
	if(rx_pkt_wait_for_data == 0) {
		while(rxCounter - read >= PACKET_HEADER) {
			read++;
			
			// if found packet_sync
			if(rx_get() == PACKET_SYNC ) {
				rx_pkt.crc = rx_get();
				rx_pkt.type = rx_get();
				rx_pkt.size = rx_get();
				read += 3;

				// check header checksum
				if( (rx_pkt.crc >> 4) == ((rx_pkt.type+rx_pkt.size) & 0xf) ) {
					rx_pkt_wait_for_data = 1;
					break;
				}
				
				if(rx_pkt_wait_for_data == 0) {
					// int can = packet_can_enabled();
					// if(can == 1) packet_enable_can(0);
					start_packet('F');
						put_byte(rx_pkt.type);
					end_packet();
					// if(can == 1) packet_enable_can(1);
				}
			}			
		}
	}
	
	if(rx_pkt_wait_for_data == 1 && rxCounter-read >= rx_pkt.size) {
		// check content checksum
		uint8_t c = 0;
		
		// checksum calculate for all data
		uint8_t i;
		for(i=0; i < rx_pkt.size; i++) {
			rx_pkt.data[i] = rx_get();
			c += rx_pkt.data[i];
		}
		
		// if checksum pass
		if( (rx_pkt.crc & 0xf) == (c & 0xf) ) {
			pass = 1;
		}
		
		read += rx_pkt.size;
		rx_pkt_wait_for_data = 0;
	}
	
	interrupt_lock
	if(rxCounter >= read) {
		rxCounter -= read;
	} else {
		rxCounter = 0;
	}
	interrupt_unlock
	
	
	if(pass) {
		rx_pkt.cursor = 0;
		
		int can = packet_can_enabled();
		if(can == 1) packet_enable_can(0);
		start_packet('A');
			put_byte(rx_pkt.type);
			put_byte(packet_get_free_packets());
		end_packet();
		if(can == 1) packet_enable_can(1);
		
			
		return &rx_pkt;
	}
	return 0;
}



// --------------[ TX packet ]----------------- 



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

void __attribute__((__interrupt__,no_auto_psv)) _U1TXInterrupt(void)
{
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




