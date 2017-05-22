#include <p33FJ128MC802.h>
#include "packet.h"
#include "uart.h"
#include "can.h"

static char uart_enabled=1;
static char can_enabled=0;

void packet_enable_uart(char tf) {
	uart_enabled = tf;
}
void packet_enable_can(char tf) {
	can_enabled = tf;
}
int packet_can_enabled() {
	return can_enabled;
}
int packet_uart_enabled() {
	return uart_enabled;
}

static Packet rx_pkt_tmp;
static Packet *rx_pkt = 0;

Packet* try_read_packet(void) {
	Packet* p = 0;
	rx_pkt = 0;
	if(uart_enabled == 1) {
		p = uart_try_read_packet();
		rx_pkt = p;
	}
	
	if(!p && can_enabled == 1) {
		uint16_t* pkt = can_get_packet();
		
		if(pkt) {
			p = &rx_pkt_tmp;
			uint8_t len = pkt[2] & 0xf;
			p->type = pkt[3] & 0xff;
			p->size = len;
			len--;
			uint8_t* d = (uint8_t*)&pkt[3] + 1;
			uint8_t i;
			for(i=0; i < len; i++) {
				p->data[i] = *d++;
			}
			rx_pkt = p;
		}
	}
	rx_pkt->cursor = 0;
	return rx_pkt;
}

#define NUM_TX_PACKETS 10
static Packet tx_pkt[NUM_TX_PACKETS];

static Packet* tx_pkt_stack[NUM_TX_PACKETS];
static Packet* tx_writing_pkt = 0;
static int8_t tx_pkt_stack_num = -1;
static volatile uint8_t tx_num_free_packets = NUM_TX_PACKETS;

void packet_init(void) {
	int i;
	for(i=0; i < NUM_TX_PACKETS; i++) {
		tx_pkt[i].status = free_to_use;
	}
}

Packet* packet_sending_queue_pop(void) {
	int i;
	for(i=0; i < NUM_TX_PACKETS; i++) {
		if(tx_pkt[i].status == ready_to_send) {
			tx_pkt[i].status = sending;
			return &tx_pkt[i];
		}
	}
	return 0;
}

void packet_free_packet(Packet* p) {
	p->status = free_to_use;
	tx_num_free_packets++;
}

char packet_can_send_packet(void) {
	// 1 packet is reserved for ACK message
	return tx_num_free_packets > 1;
}

#define PACKET_SYNC 0x3c

void start_packet(uint8_t type) {
	tx_pkt_stack_num++;
	if(tx_pkt_stack_num < NUM_TX_PACKETS && tx_num_free_packets > 0) {
		tx_writing_pkt = 0;
		
		// find free packet
		int i;
		for(i=0; i < NUM_TX_PACKETS; i++) {
			if(tx_pkt[i].status == free_to_use) {
				if(SRbits.IPL == 0) {
					SRbits.IPL = 7;
				}
				tx_writing_pkt = &tx_pkt[i];
				tx_writing_pkt->status = writing_packet;
				tx_num_free_packets--;
				SRbits.IPL = 0;
				break;
			}
		}
		
		tx_pkt_stack[tx_pkt_stack_num] = tx_writing_pkt;
		if(tx_writing_pkt == 0) return;
		
		tx_writing_pkt->type = type;
		tx_writing_pkt->size = 0;
	}
}

void put_byte(int8_t b) {
	if(tx_writing_pkt == 0) return;
	tx_writing_pkt->data[tx_writing_pkt->size++] = b;
}

void put_word(int16_t w) {
	if(tx_writing_pkt == 0) return;
	
	tx_writing_pkt->data[tx_writing_pkt->size] = w >> 8;
	tx_writing_pkt->data[tx_writing_pkt->size+1] = w;
	tx_writing_pkt->size += 2;
}



void end_packet(void) {
	
	if(tx_writing_pkt != 0) {
		tx_writing_pkt->cursor = 0;
		tx_writing_pkt->status = ready_to_send;
		
		
		if(uart_enabled == 1) {
			tx_writing_pkt->sync = PACKET_SYNC;
			int i, crc=0;
			for(i=0; i < tx_writing_pkt->size; i++) {
				crc += tx_writing_pkt->data[i];
			}
			tx_writing_pkt->crc = ((tx_writing_pkt->size + tx_writing_pkt->type) << 4) | (crc & 0xf);
			uart_start_sending_packet(tx_writing_pkt);
		}
		

		if(can_enabled == 1) {
			unsigned int* buf = can_get_free_tx_buffer();
			if(buf) {
				uint8_t* d = (uint8_t*)&buf[3];
				int i;
				can_write_tx_default_id(buf);
				uint8_t len = tx_writing_pkt->size+1;
				if(len == 0) len = 1;
				else if(len > 8) len = 8;
				buf[2] = (buf[2] & 0xfff0) | (len & 0xf);
				*d++ = tx_writing_pkt->type;
				len = len-1;
				for(i=0; i < len; i++) {
					d[i] = tx_writing_pkt->data[i];
				}
				can_send_tx_buffer(buf);
			}
			
			if(uart_enabled == 0) {
				if(SRbits.IPL == 0) {
					SRbits.IPL = 7;
				}
				packet_free_packet(tx_writing_pkt);
				SRbits.IPL = 0;
			}
		}
	}
	
	if(tx_pkt_stack_num >= 0) {
		tx_pkt_stack_num--;
	}
	
	if(tx_pkt_stack_num >= 0 && tx_pkt_stack_num < NUM_TX_PACKETS) {
		tx_writing_pkt = tx_pkt_stack[tx_pkt_stack_num];
	} else {
		tx_writing_pkt = 0;
	}
}


uint8_t get_byte(void) {
	if(rx_pkt == 0) return 0;
	if(rx_pkt->cursor < rx_pkt->size) {
		return rx_pkt->data[rx_pkt->cursor++];
	} else {
		return 0;
	}
}

uint16_t get_word(void) {
	if(rx_pkt == 0) return 0;
	if(rx_pkt->cursor+1 < rx_pkt->size) {
		uint16_t r = ((uint16_t)rx_pkt->data[rx_pkt->cursor] << 8) | rx_pkt->data[rx_pkt->cursor+1];
		rx_pkt->cursor += 2;
		return r;
	} else {
		return 0;
	}
}
