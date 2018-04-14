#ifndef PACKET_H
#define PACKET_H
#include <stdint.h>

#ifdef SIM
#include <stdio.h>
#define dbg(...) printf(__VA_ARGS__)
#else
#define dbg(...)
#endif

#define MAX_PKT_SIZE 64
#define PACKET_HEADER 4

enum PacketStatus {
	free_to_use,
	writing_packet,
	ready_to_send,
	in_queue,
	sending
};

// ------------- UART packet protocol ------------

/*
	SCTLxxxxxxxx

	S - 1 Byte sync (0x3c)
	C - 1 Byte checksum ( upper nibble - header checksum, lower nibble payload checksum )
	T - 1 Byte type
	L - 1 Byte payload length
	x - L Bytes data
*/

struct Packet {
	uint8_t sync;
	uint8_t crc;
	uint8_t type;
	uint8_t size;
	uint8_t data[MAX_PKT_SIZE-PACKET_HEADER];
	uint8_t cursor;
	enum PacketStatus status;
} __attribute__((packed));

typedef struct Packet Packet;
int packet_uart_enabled();
int packet_can_enabled();

void packet_enable_uart(char tf);
void packet_enable_can(char tf);
void packet_init(void);
void packet_free_packet(Packet* p);
uint8_t packet_get_free_packets(void);
Packet* packet_sending_queue_pop(void);

void can_to_packet(uint16_t* pkt, Packet* p);

// reading packet
Packet* try_read_packet(void);

// after succeeding try_read_packet
uint8_t get_byte(void);
uint16_t get_word(void);
uint32_t get_long(void);

char packet_can_send_packet(void);
void put_byte(int8_t b);
void put_word(int16_t w);
void put_long(int32_t l);
#define put_byte_word(b,w) put_byte(b); put_word(w);

void start_packet(uint8_t type);
void end_packet(void);


#endif
