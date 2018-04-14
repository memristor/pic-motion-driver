#include "uart.h"

void uart_init(long baud)
{	
}

inline char uart_busy(void)
{  
}

void uart_close(void)
{  
}

void uart_putch(unsigned char c)
{
}

void uart_putint16(unsigned int s) {
}

void uart_init_pins(void) {
}

void uart_putstr(const char* s) {
}

unsigned char uart_getch(void)
{
	return 0;
}

unsigned int uart_getint16(void) {
	return 0;
}


unsigned char uart_get_last_byte(void)
{
	return 0;
}

unsigned char uart_check_rx(void)
{
	return 0;
}

void uart_flush(void) {
}

Packet* uart_try_read_packet(void) {
	return 0;
}

// --------------[ TX packet ]----------------- 

void uart_start_sending_packet(Packet* p) {
}



