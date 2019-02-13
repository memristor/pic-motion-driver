#ifndef UART_H
#define	UART_H

#include <stdint.h>
#include "packet.h"

#define RX_BUF_LEN  100
#define RX_BUF_MASK  0x0f

#include "../config.h"

void uart_init(long);
void uart_close(void);
unsigned char uart_check_rx(void);
unsigned char uart_get_last_byte(void);

unsigned char uart_getch(void);
unsigned int uart_getint16(void);

void uart_putch(unsigned char c);
void uart_putint16(unsigned int c);
void uart_putstr(const char* s);


// -------- uart packet protocol ----------

void uart_start_sending_packet(Packet* p);

// reading packet
Packet* uart_try_read_packet(void);
// after succeeding try_read_packet
uint8_t uart_get_byte(void);
uint16_t uart_get_word(void);


// writing packet
char uart_can_send_packet(void);
void uart_start_packet(uint8_t type);
void uart_put_byte(int8_t b);
void uart_put_word(int16_t w);
#define uart_put_byte_word(b,w) uart_put_byte(b); uart_put_word(w);
void uart_end_packet(void);
void uart_init_pins(void);

#endif	/* UART_H */
