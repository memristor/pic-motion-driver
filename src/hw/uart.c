#include "uart.h"

volatile uint8_t uart_rx_buf[RX_BUF_LEN];
volatile uint8_t uart_rx_index1 = 0; // new data is put at this offset
volatile uint8_t uart_rx_index2 = 0; // data is read from this offset
volatile uint8_t uart_rx_data;
volatile uint8_t uart_rx_counter = 0;


// ------------[ RX PACKET ]--------------

static uint8_t rx_get(void) {
	if(++uart_rx_index2 >= RX_BUF_LEN) {
		uart_rx_index2 = 0;
	}
	uart_rx_counter--;
	return uart_rx_buf[uart_rx_index2];
}

#define PACKET_SYNC 0x3c
static Packet rx_pkt;
static uint8_t rx_pkt_wait_for_data = 0;

// ------------[ UART synchronous write ]---------------


void uart_putstr(const uint8_t* s) {
	while(*s) {
		uart_putch(*s);
		s++;
	}
}

// ------------[ UART synchronous read ]--------------
/*
 * Don't use from interrupt
 */
uint8_t uart_getch(void) {
	while(uart_rx_counter == 0);
	interrupt_lock
	uart_rx_counter--;
	if(++uart_rx_index2 == RX_BUF_LEN) {
		uart_rx_index2 = 0;
	}
	interrupt_unlock
	return uart_rx_buf[uart_rx_index2];
}

/*
 * Don't use from interrupt
 */
uint16_t uart_getint16(void) {
	unsigned int ret;
	while(uart_rx_counter < 2);
	
	if(++uart_rx_index2 == RX_BUF_LEN) {
		uart_rx_index2 = 0;
	}
	
	ret = uart_rx_buf[uart_rx_index2] << 8;
	if(++uart_rx_index2 == RX_BUF_LEN) {
		uart_rx_index2 = 0;
	}

	ret |= uart_rx_buf[uart_rx_index2];
	
	interrupt_lock
	uart_rx_counter-=2;
	interrupt_unlock
	return ret;
}

void uart_putint16(uint16_t s) {
	uart_putch(s >> 8);
	uart_putch(s & 0xff);
}

// ------------[ UART peek ]-------------

uint8_t uart_get_last_byte(void) {
	return uart_rx_buf[uart_rx_index1];
}

uint8_t uart_check_rx(void) {
	return uart_rx_counter;
}


void uart_push_char(char ch) {
	if(++uart_rx_index1 == RX_BUF_LEN) {
		uart_rx_index1 = 0;
	}
	uart_rx_buf[uart_rx_index1] = ch;
	uart_rx_counter++;
}

/*
 * Don't use from interrupt
 */
void uart_flush(void) {
	interrupt_lock
	uart_rx_counter = 0;
	uart_rx_index2 = uart_rx_index1;
	interrupt_unlock
}
/*
 * Don't use from interrupt
 */
Packet* uart_try_read_packet(void) {
	uint8_t read = 0;
	uint8_t pass = 0;
	if(rx_pkt_wait_for_data == 0) {
		while(uart_rx_counter - read >= PACKET_HEADER) {
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
					// Fail checksum
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
	
	if(rx_pkt_wait_for_data == 1 && uart_rx_counter - read >= rx_pkt.size) {
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
	
	// interrupt_lock
	// if(uart_rx_counter >= read) {
		// uart_rx_counter -= read;
	// } else {
		// uart_rx_counter = 0;
	// }
	// interrupt_unlock
	
	
	if(pass) {
		rx_pkt.cursor = 0;
		// ack only on uart
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
