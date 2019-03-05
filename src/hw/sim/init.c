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
	
	packet_init();
	printf("pkt initialized\n");
	
	config_init();
	
	regulator_init();
	printf("regulator initialized\n");
	// must come after regulator_init !
	config_load_defaults();
	
	timer_init();
	encoder_init();
	printf("encoder initialized\n");
	
	reset_driver();
	printf("driver reset\n");
	motor_init();
	set_speed(0x32);
	
	printf("initialized\n");
}
