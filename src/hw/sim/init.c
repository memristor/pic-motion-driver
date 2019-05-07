#include "../timer.h"
#include "../init.h"
#include <stdio.h>
static void port_init(void) {}
static void init_chip_clock(void) {}
void external_interrupt_init_pins() {}
void external_interrupt_init() {}
static void init_pins(void) {}

void hw_init(void) {
	
	init_chip_clock();
	init_pins();
	external_interrupt_init();
	//INTCON1bits.NSTDIS = 1; // disable recursive interrupts?
	port_init();
	
	#define CAN_ID 600
	#define CAN_USE_EXTENDED_ID 1
	#define UART_BAUD 57600
	
	can_init(CAN_ID, CAN_USE_EXTENDED_ID);
	uart_init(UART_BAUD);
	
	timer_init();
	encoder_init();
}
