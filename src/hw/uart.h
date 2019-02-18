#ifndef UART_H
#define	UART_H

#include <stdint.h>
#include "../packet.h"
#include "../config.h"

#define RX_BUF_LEN  100
#define RX_BUF_MASK  0x0f


void uart_init(long);
uint8_t uart_check_rx(void);
uint8_t uart_get_last_byte(void);

uint8_t uart_getch(void);
uint16_t uart_getint16(void);

void uart_putch(uint8_t c);
void uart_putint16(uint16_t c);
void uart_putstr(const uint8_t* s);


// -------- uart packet protocol ----------

void uart_start_sending_packet(Packet* p);

// reading packet
Packet* uart_try_read_packet(void);
// after succeeding try_read_packet
uint8_t uart_get_byte(void);
uint16_t uart_get_word(void);


// writing packet
int8_t uart_can_send_packet(void);
void uart_start_packet(uint8_t type);
void uart_put_byte(int8_t b);
void uart_put_word(int16_t w);
#define uart_put_byte_word(b,w) uart_put_byte(b); uart_put_word(w);
void uart_end_packet(void);
void uart_init_pins(void);


extern volatile uint8_t uart_rx_buf[RX_BUF_LEN];
extern volatile uint8_t uart_rx_index1; // new data is put at this offset
extern volatile uint8_t uart_rx_index2; // data is read from this offset
extern volatile uint8_t uart_rx_counter;

#endif	/* UART_H */
