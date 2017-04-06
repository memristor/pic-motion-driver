#ifndef UART_H
#define	UART_H

#include <stdint.h>

#define RX_BUF_LEN  100
#define RX_BUF_MASK  0x0f

#ifndef FCY
#define FCY 29491200ULL
#endif

void uart_init(long);
char uart_busy(void);
void uart_close(void);
unsigned char uart_check_rx(void);
unsigned char uart_get_last_byte(void);

unsigned char getch(void);
unsigned int getint16(void);

void putch(unsigned char c);
void putint16(unsigned int c);
void putstr(const char* s);


// -------- uart packet protocol ----------

#define MAX_PKT_SIZE 32
#define PACKET_HEADER 4

struct Packet {
	uint8_t sync;
	uint8_t crc;
	uint8_t type;
	uint8_t size;
	uint8_t data[MAX_PKT_SIZE-PACKET_HEADER];
	uint8_t cursor;
	uint8_t status;
};

typedef struct Packet Packet;

// reading packet
Packet* try_read_packet(void);
// after succeeding try_read_packet
uint8_t get_byte(void);
uint16_t get_word(void);



// writing packet
char can_send_packet(void);
void start_packet(uint8_t type);
void put_byte(int8_t b);
void put_word(int16_t w);
#define put_byte_word(b,w) put_byte(b); put_word(w);
void end_packet(void);

#endif	/* UART_H */
