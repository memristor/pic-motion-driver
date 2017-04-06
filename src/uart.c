#include "uart.h"
#include <p33FJ128MC802.h>
#include <libpic30.h>

static volatile uint8_t rx_buf[RX_BUF_LEN];
static volatile uint8_t rx_index1 = 0; // new data is put at this offset
static volatile uint8_t rx_index2 = 0; // data is read from this offset
static volatile uint8_t rxData;
static volatile uint8_t rxCounter = 0;

static void init_tx_packets(void);

void uart_init(long baud)
{
	U1BRG = (double)FCY / (16 * baud) - 1;
	U1MODEbits.STSEL = 0;   // 1 Stop bit
	U1MODEbits.PDSEL = 0;   // No Parity, 8 data bits
	U1MODEbits.BRGH = 0;    // Low-Speed mode

	U1STAbits.URXISEL = 0; // rx interrupt for each received byte
	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;

	// _U1RXIE = 1;    		// Enable UART1 Rx interrupt
	
	IEC0bits.U1RXIE = 1;
	IEC0bits.U1TXIE = 1;
	
	IPC2bits.U1RXIP = 6;	// Priority must be less than 7, otherwise it can't be temporarily disabled
	IPC3bits.U1TXIP = 6;	// Priority must be less than 7, otherwise it can't be temporarily disabled
	
	IPC0bits.T1IP = 1; 		// T1 interrupt priority set to minimum

	rx_index1 = rx_index2 = rxCounter = 0;

	U1MODEbits.UARTEN = 1;  // Enable UART
	U1STAbits.UTXEN = 1;
	
	init_tx_packets();

	__delay_ms(100);
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

void putch(unsigned char c)
{
	while(U1STAbits.UTXBF);
	U1TXREG = c;
}

void putint16(unsigned int s) {
	while(U1STAbits.UTXBF);
	
	U1TXREG = s >> 8;
	
	while(U1STAbits.UTXBF);
	
	U1TXREG = s;
}

void putstr(const char* s) {
	while(*s) {
		putch(*s);
		s++;
	}
}

// ------------[ UART synchronous read ]--------------

unsigned char getch(void)
{
	while(rxCounter == 0);
	SRbits.IPL = 7;
	rxCounter--;
	if(++rx_index2 == RX_BUF_LEN)
		rx_index2 = 0;
	SRbits.IPL = 0;
	return rx_buf[rx_index2];
}

unsigned int getint16(void) {
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


void flush(void) {
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
		if(++rx_index1 == RX_BUF_LEN)
			rx_index1 = 0;
		rx_buf[rx_index1] = rxData;
		rxCounter++;
	}

	U1STAbits.OERR = 0;
	
	IFS0bits.U1RXIF = 0;
}



// end

// ------------- UART packet protocol ------------

/*
	SCTLxxxxxxxx

	S - 1 Byte sync (0x3c)
	C - 1 Byte checksum ( upper nibble - header checksum, lower nibble payload checksum )
	T - 1 Byte type
	L - 1 Byte payload length
	x - L Bytes data
*/


#define PACKET_SYNC 0x3c



// ------------[ RX PACKET ]--------------

/*
	@param
		length - pointer to byte where length will be written if packet is found
	@return
		0 - no packet found
		other - pointer to data of length set by this function (in this data)
*/

static uint8_t rx_get(void) {
	if(++rx_index2 >= RX_BUF_LEN) {
		rx_index2 = 0;
	}
	return rx_buf[rx_index2];
}

static Packet rx_pkt;
static uint8_t rx_pkt_wait_for_data = 0;

Packet* try_read_packet(void) {
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
	
	SRbits.IPL = 7;
	if(rxCounter >= read)
		rxCounter -= read;
	else
		rxCounter = 0;
	SRbits.IPL = 0;
	
	
	if(pass) {
		rx_pkt.cursor = 0;
		start_packet('A');
			put_byte(rx_pkt.type);
		end_packet();
		return &rx_pkt;
	}
	return 0;
}

uint8_t get_byte(void) {
	if(rx_pkt.cursor < rx_pkt.size)
		return rx_pkt.data[rx_pkt.cursor++];
	else
		return 0;
}

uint16_t get_word(void) {
	if(rx_pkt.cursor+1 < rx_pkt.size) {
		uint16_t r = ((uint16_t)rx_pkt.data[rx_pkt.cursor] << 8) | rx_pkt.data[rx_pkt.cursor+1];
		rx_pkt.cursor += 2;
		return r;
	} else {
		return 0;
	}
}

// --------------[ TX packet ]----------------- 
#define NUM_TX_PACKETS 6

enum PacketStatus {
	free_to_use,
	writing_packet,
	ready_to_send,
	sending
};

static volatile Packet* tx_pkt_sending = 0;

static int8_t tx_pkt_stack_num = -1;
static Packet* tx_pkt_stack[NUM_TX_PACKETS];

static Packet tx_pkt[NUM_TX_PACKETS];
static Packet* tx_writing_pkt = 0;
static volatile uint8_t tx_num_free_packets = NUM_TX_PACKETS;

static void init_tx_packets(void) {
	int i;
	for(i=0; i < NUM_TX_PACKETS; i++) {
		tx_pkt[i].status = free_to_use;
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
	if(tx_pkt_sending) {
		U1TXREG = ((uint8_t*)tx_pkt_sending)[tx_pkt_sending->cursor++];
		if(tx_pkt_sending->cursor >= tx_pkt_sending->size + PACKET_HEADER) {
			tx_pkt_sending->status = free_to_use;
			tx_num_free_packets++;
			tx_pkt_sending = 0;
		}
	}
	
	if(!tx_pkt_sending) {
		int i;
		for(i=0; i < NUM_TX_PACKETS; i++) {
			if(tx_pkt[i].status == ready_to_send) {
				tx_pkt[i].status = sending;
				tx_pkt_sending = &tx_pkt[i];
				// packet is at least PACKET_HEADER size which is bigger than 1, so no need for checking
				// whether packet is fully sent
				U1TXREG = ((uint8_t*)tx_pkt_sending)[tx_pkt_sending->cursor++];
				break;
			}
		}
	}
	
	IFS0bits.U1TXIF = 0;
}

inline char can_send_packet(void) {
	return tx_num_free_packets > 1; // 1 packet is reserved for ACK message
}

void start_packet(uint8_t type) {
	tx_pkt_stack_num++;
	if(tx_pkt_stack_num < NUM_TX_PACKETS && tx_num_free_packets > 0) {
		
		SRbits.IPL = 7;
		tx_num_free_packets--;
		SRbits.IPL = 0;
		
		tx_writing_pkt = 0;
		int i;
		for(i=0; i < NUM_TX_PACKETS; i++) {
			if(tx_pkt[i].status == free_to_use) {
				tx_writing_pkt = &tx_pkt[i];
				break;
			}
		}
		
		tx_pkt_stack[tx_pkt_stack_num] = tx_writing_pkt;
		if(tx_writing_pkt == 0) return;
		
		tx_writing_pkt->status = writing_packet;
		tx_writing_pkt->type = type;
		tx_writing_pkt->size = 0;
	}
}

void put_byte(int8_t b) {
	if(!tx_writing_pkt) return;
	tx_writing_pkt->data[tx_writing_pkt->size++] = b;
}

void put_word(int16_t w) {
	if(!tx_writing_pkt) return;
	
	tx_writing_pkt->data[tx_writing_pkt->size] = w >> 8;
	tx_writing_pkt->data[tx_writing_pkt->size+1] = w;
	tx_writing_pkt->size += 2;
}

void end_packet(void) {
	
	if(tx_writing_pkt) {
		tx_writing_pkt->sync = PACKET_SYNC;
		int crc = 0;
		int i;
		for(i=0; i < tx_writing_pkt->size; i++) {
			crc += tx_writing_pkt->data[i];
		}
		
		tx_writing_pkt->crc = ((tx_writing_pkt->size + tx_writing_pkt->type) << 4) | (crc & 0xf);
		
		tx_writing_pkt->cursor = 0;
		tx_writing_pkt->status = ready_to_send;
		if(!tx_pkt_sending) {
			tx_pkt_sending = tx_writing_pkt;
			U1TXREG = ((uint8_t*)tx_pkt_sending)[tx_pkt_sending->cursor++];
			tx_writing_pkt->status = sending;
		}
	}
	
	if(tx_pkt_stack_num >= 0)
		tx_pkt_stack_num--;
		
	if(tx_pkt_stack_num >= 0 && tx_pkt_stack_num < NUM_TX_PACKETS) {
		tx_writing_pkt = tx_pkt_stack[tx_pkt_stack_num];
	} else {
		tx_writing_pkt = 0;
	}
	
}


