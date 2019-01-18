#include "packet.h"
#include "uart.h"
#include "can.h"
#include "../bootloader.h"
#include "../util/math.h"
static char uart_enabled=1;
static char can_enabled=1;

static char uart_is_used = 0;
static char can_is_used = 1;

void packet_enable_uart(char tf) {
	uart_enabled = tf;
	uart_is_used = tf;
}
void packet_enable_can(char tf) {
	can_enabled = tf;
	can_is_used = tf;
}
int packet_can_enabled() {
	return can_enabled;
}
int packet_uart_enabled() {
	return uart_enabled;
}

static Packet rx_pkt_tmp;
static Packet *rx_pkt = 0;

void BOOT can_to_packet(uint16_t* pkt, Packet* p) {
	if(!pkt || !p) return;
	uint8_t len = pkt[2] & 0xf;
	p->type = pkt[3] & 0xff;
	len--;
	p->size = len;
	uint8_t* d = (uint8_t*)&pkt[3] + 1;
	uint8_t i;
	for(i=0; i < len; i++) {
		p->data[i] = *d++;
	}
}

// CMD_CAN_EXTEND Size msg..., CMD_EXTEND msg...
Packet can_extend_pkt;
#define CMD_CAN_EXTEND_START '\n'
#define CMD_CAN_EXTEND '\r'

Packet* try_read_packet(void) {
	Packet* p = 0;
	rx_pkt = 0;
	
	if(uart_enabled == 1) {
		p = uart_try_read_packet();
		if(p) {
			uart_is_used = 1;
		}
		rx_pkt = p;
	}
	
	if(!p && can_enabled == 1) {
		uint16_t* pkt = can_get_packet();
		if(pkt) {
			p = &rx_pkt_tmp;
			
			can_is_used = 1;
			can_to_packet(pkt, p);
			
			rx_pkt = p;
			rx_pkt->cursor = 0;
			
			// handle can extending
			if(p->type == CMD_CAN_EXTEND || p->type == CMD_CAN_EXTEND_START) {
				int i;
				if(p->type == CMD_CAN_EXTEND_START) {
					can_extend_pkt.cursor = 0;
				}
				// \rTL + L data count
				if(can_extend_pkt.cursor == 0 && p->size > 2) {
					can_extend_pkt.type = p->data[0];
					can_extend_pkt.size = p->data[1];
					
					for(i=0; i < p->size-2; i++) {
						can_extend_pkt.data[i] = p->data[i+2];
					}
					can_extend_pkt.cursor += p->size - 2;
				} else {
					for(i=0; i < p->size; i++) {
						can_extend_pkt.data[i+can_extend_pkt.cursor] = p->data[i];
					}
					can_extend_pkt.cursor += p->size;
				}
				if(can_extend_pkt.cursor >= can_extend_pkt.size) {
					can_extend_pkt.cursor = 0;
					rx_pkt = &can_extend_pkt;
				} else {
					rx_pkt = 0;
				}
			}
		}
	}
	
	if(rx_pkt) {
		rx_pkt->cursor = 0;
	}
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
	can_extend_pkt.cursor = 0;
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
	p->cursor = 0;
	tx_num_free_packets++;
}

char packet_can_send_packet(void) {
	// 1 packet is reserved for ACK message
	return tx_num_free_packets > 1;
}

uint8_t packet_get_free_packets(void) {
	return tx_num_free_packets;
}

#define PACKET_SYNC 0x3c



void start_packet(uint8_t type) {
	tx_pkt_stack_num++;
	if(tx_pkt_stack_num < NUM_TX_PACKETS && tx_num_free_packets > 0) {
		tx_writing_pkt = 0;
		
		// find free packet
		int i;
		interrupt_lock
		for(i=0; i < NUM_TX_PACKETS; i++) {
			if(tx_pkt[i].status == free_to_use) {
				tx_writing_pkt = &tx_pkt[i];
				tx_writing_pkt->status = writing_packet;
				tx_num_free_packets--;
				break;
			}
		}
		interrupt_unlock
		
		tx_pkt_stack[tx_pkt_stack_num] = tx_writing_pkt;
		if(tx_writing_pkt == 0) return;
		
		tx_writing_pkt->type = type;
		tx_writing_pkt->size = 0;
		tx_writing_pkt->cursor = 0;
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

void put_long(int32_t l) {
	if(tx_writing_pkt == 0) return;
	
	tx_writing_pkt->data[tx_writing_pkt->size] = l >> 24;
	tx_writing_pkt->data[tx_writing_pkt->size+1] = l >> 16;
	tx_writing_pkt->data[tx_writing_pkt->size+2] = l >> 8;
	tx_writing_pkt->data[tx_writing_pkt->size+3] = l;
	tx_writing_pkt->size += 4;
}



void end_packet(void) {
	
	if(tx_writing_pkt != 0) {
		struct Packet* pkt = tx_writing_pkt;
		pkt->cursor = 0;
		
		if(uart_enabled == 1  && uart_is_used == 1) {
		// if(uart_enabled) {
			pkt->sync = PACKET_SYNC;
			int i, crc=0;
			for(i=0; i < pkt->size; i++) {
				crc += pkt->data[i];
			}
			pkt->crc = ((pkt->size + pkt->type) << 4) | (crc & 0xf);
			interrupt_lock
			pkt->status = ready_to_send;
			pkt->cursor = 0;
			uart_start_sending_packet(pkt);
			interrupt_unlock
		}
		
		
		if(can_enabled == 1 && can_is_used == 1) {
			uint16_t* buf = can_get_free_tx_buffer();
			if(buf) {
				uint8_t* d = (uint8_t*)&buf[3];
				int i;
				can_write_tx_default_id(buf);
				uint8_t len = pkt->size+1;
				if(len == 0) len = 1;
				else if(len > 8) len = 8;
				len = clip(1,8, len);
				buf[2] = (buf[2] & 0xfff0) | (len & 0xf);
				*d++ = pkt->type;
				len = len-1;
				for(i=0; i < len; i++) {
					d[i] = pkt->data[i];
				}
				can_send_tx_buffer(buf);
			}
		}
		
		if(pkt->status == writing_packet) {
			interrupt_lock
			packet_free_packet(pkt);
			interrupt_unlock
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

uint32_t get_long(void) {
	if(rx_pkt == 0) return 0;
	if(rx_pkt->cursor+3 < rx_pkt->size) {
		uint32_t r = ((uint32_t)rx_pkt->data[rx_pkt->cursor] << 24) |
					 ((uint32_t)rx_pkt->data[rx_pkt->cursor+1] << 16) |
					 ((uint32_t)rx_pkt->data[rx_pkt->cursor+2] << 8) |
					 (((uint32_t)rx_pkt->data[rx_pkt->cursor+3]) & 0xff);
		rx_pkt->cursor += 4;
		return r;
	} else {
		return 0;
	}
}
