#include "../../config.h"
#include "../../packet.h"
#include "../../regulator.h"

#include "../timer.h"
#include "../can.h"
#include "../uart.h"
#include "../motor.h"
#include "../encoder.h"


static void port_init(void) {
	AD1PCFGL = 0xFFFF;// all PORT Digital
	
	TRISAbits.TRISA4=0;
	TRISBbits.TRISB8=0;
	TRISBbits.TRISB9=0;
	TRISBbits.TRISB10=1;
	TRISBbits.TRISB11=0;
	TRISBbits.TRISB12=0;
	TRISBbits.TRISB13=1;
	TRISBbits.TRISB14=0;
	TRISBbits.TRISB15=0;

	LATAbits.LATA4 = 0;

	LATBbits.LATB8 = 0;
	LATBbits.LATB9 = 0;
	// LATBbits.LATB10 = 0;
	LATBbits.LATB11 = 1;
	LATBbits.LATB12 = 1;
	// LATBbits.LATB13 = 0;
	LATBbits.LATB14 = 0;
	LATBbits.LATB15 = 0;
}

static void init_chip_clock(void) {
	/* Configure Oscillator to operate the device at 30Mhz
	   Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	   Fosc= 7.37*(32)/(2*2)=58.96Mhz for Fosc, Fcy = 29.48Mhz */

	// OSCCONbits.CLKLOCK = 0;
	
	#ifdef USE_FRCPLL
	// Configure PLL prescaler, PLL postscaler, PLL divisor
	// PLLFBDbits.PLLDIV = 30;   // M = PLLFBD + 2
	PLLFBD = 30;
	CLKDIVbits.PLLPOST=0;   // N1 = 2
	CLKDIVbits.PLLPRE=0;    // N2 = 2
	
	#else // USE_CRYSTAL
	
	PLLFBD = 30;
	CLKDIVbits.PLLPOST=0;   // N1 = 2
	// CLKDIVbits.PLLPRE=0;    // N2 = 2
	CLKDIVbits.PLLPRE=2;    // N2 = 4
	
	
	// OSCCONbits.NOSC = FNOSC_PRIPLL;
	
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);
	while (OSCCONbits.COSC != 0b011);
	#endif
	// OSCCONbits.OSWEN = 1;
	// while(OSCCONbits.OSWEN != 0);
	// OSCCONbits.CLKLOCK = 1;
	while(!OSCCONbits.LOCK); // wait for PLL to lock
}


static void external_interrupt_init_pins() {
	RPINR1bits.INT2R = 10;
	RPINR0bits.INT1R = 13;
}

static void external_interrupt_init() {
	_INT1IE = 1;
	_INT2IE = 1;
}

static void init_pins(void) {
	__builtin_write_OSCCONL(OSCCON & 0xDF);
	
	can_init_pins();
	uart_init_pins();
	encoder_init_pins();
	external_interrupt_init_pins();

	__builtin_write_OSCCONL(OSCCON | (1<<6));
}

void hw_init(void) {
	init_chip_clock();
	init_pins();
	external_interrupt_init();
	//INTCON1bits.NSTDIS = 1; // disable recursive interrupts?
	port_init();
	
	#define CAN_ID 600
	#define CAN_USE_EXTENDED_ID 1
	#define UART_BAUD 57600

	timer_init();
	encoder_init();
	can_init(CAN_ID, CAN_USE_EXTENDED_ID);
	uart_init(UART_BAUD);
}
