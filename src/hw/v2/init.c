#include "../../config.h"
#include "../timer.h"
#include "../motor.h"
#include "../encoder.h"

static void port_init(void) {
	ANSELA = 0;
	ANSELB = 0;
	ANSELC = 0;
}

static void init_chip_clock(void) {
	
	PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
	PLIB_OSC_FRCDivisorSelect(OSC_ID_0, OSC_FRC_DIV_1);
	int i;
	for(i=0; i <= 5; i++) {
		PLIB_OSC_PBClockDivisorSet(OSC_ID_0, i, 1);
		PLIB_OSC_PBOutputClockEnable(OSC_ID_0, i);
	}
}


void external_interrupt_init_pins() {
}

void external_interrupt_init() {
}

static void init_pins(void) {
	encoder_init_pins();
}


void hw_init(void) {
	init_chip_clock();
	port_init();
	init_pins();
	// external_interrupt_init();
	PLIB_INT_MultiVectorSelect(INT_ID_0);
	encoder_init();
	timer_init();
	
	#define CAN_ID 600
	#define CAN_USE_EXTENDED_ID 1
	#define UART_BAUD 57600
	
	
	can_init(CAN_ID, CAN_USE_EXTENDED_ID);
	uart_init(UART_BAUD);
	
	// motor_init();
}
