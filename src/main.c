/*
 * Code for eurobot; max speed limited to 180
 *
 * Comment: X forward; Y right;
 */
#define FCY 29491200ULL

#include "regulator.h"
#include "uart.h"
#include "motor.h"
#include "encoder.h"
#include "timer.h"
#include "config.h"
#include <libpic30.h>
#include <stdint.h>
#include <p33FJ128MC802.h>

#pragma config FWDTEN = OFF, \
			   FNOSC = FRCPLL

// static uint32_t load_uint32_bigendian(uint8_t* s) {
	// return (((uint32_t)s[0]) << 24) | (((uint32_t)s[1]) << 16) | (((uint32_t)s[2]) << 8) | ((uint32_t)s[3]);
// }


void port_init(void)
{

	TRISAbits.TRISA4=0;

	TRISBbits.TRISB8=0;
	TRISBbits.TRISB9=0;
	TRISBbits.TRISB10=0;
	TRISBbits.TRISB11=0;
	TRISBbits.TRISB12=0;
	TRISBbits.TRISB13=0;
	TRISBbits.TRISB14=0;
	TRISBbits.TRISB15=0;

	LATAbits.LATA4 = 0;

	LATBbits.LATB8 = 0;
	LATBbits.LATB9 = 0;
	LATBbits.LATB10 = 0;
	LATBbits.LATB11 = 1;
	LATBbits.LATB12 = 1;
	LATBbits.LATB13 = 0;
	LATBbits.LATB14 = 0;
	LATBbits.LATB15 = 0;
}

int main(void)
{
	int tmpX, tmpY, tmp, tmpO;
	calculate_K(92.936704f,330.7f);
	char command, v, direction, tmpSU;

	/* Configure Oscillator to operate the device at 30Mhz
	   Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	   Fosc= 7.37*(32)/(2*2)=58.96Mhz for Fosc, Fcy = 29.48Mhz */

	/* Configure PLL prescaler, PLL postscaler, PLL divisor */

	PLLFBDbits.PLLDIV = 30;   /* M = PLLFBD + 2 */
	CLKDIVbits.PLLPOST=0;   /* N1 = 2 */
	CLKDIVbits.PLLPRE=0;    /* N2 = 2 */

	while(!OSCCONbits.LOCK); // wait for PLL to lock

	AD1PCFGL = 0xFFFF;// all PORT Digital

	
	__builtin_write_OSCCONL(OSCCON & 0xDF);
	
	RPINR18bits.U1RXR = 0;		//UART1 RX -> RP0- pin 4
	RPOR0bits.RP1R = 3;			//UART1 TX -> RP1- pin 5
	RPINR14bits.QEA1R = 2;		//QEI1A -> RP2
	RPINR14bits.QEB1R = 3;		//QEI1B -> RP3

	RPINR16bits.QEA2R = 4;		//QEI2A -> RP4
	RPINR16bits.QEB2R = 7;		//QEI2B -> RP7

	__builtin_write_OSCCONL(OSCCON | (1<<6));

	//INTCON1bits.NSTDIS = 1; // disable recursive interrupts?

	config_init();
	port_init();
	uart_init(57600);
	timer_init();
	encoder_init();
	motor_init();
	
	regulator_init();
	config_load_defaults();
	
	reset_driver();

	set_speed(0x32);

	while(1)
	{
		Packet* pkt = 0;
		if(!(pkt=try_read_packet())) continue;
		command = pkt->type;
		reset_stuck();
		switch(command)
		{
			// set position and orientation
			case CMD_SET_POSITION_AND_ORIENTATION:
				// x [mm], y [mm], orientation
				tmpX = get_word();
				tmpY = get_word();
				tmpO = get_word();
				set_position(tmpX, tmpY, tmpO);
				break;
				
			case CMD_SET_CONFIG:
				config_load(pkt->size, pkt->data);
				break;
			
			case CMD_GET_CONFIG: {
				int key = get_byte();
				uint32_t val = config_get_as_uint32(key);
				start_packet(CMD_GET_CONFIG);
					put_word(val >> 16);
					put_word(val);
				end_packet();
				break;
			}
				// read status and position
			case CMD_SEND_STATUS:
				send_status_and_position();
				break;

				// set speed; Vmax(0-255)
			case CMD_SET_SPEED:
				set_speed(get_byte());
				break;

			case CMD_SET_ROTATION_SPEED: {
				uint8_t max_speed = get_byte(); 
				uint8_t max_accel = get_byte(); 
				set_rotation_speed(max_speed, max_accel);
				break;
			}
				
				// move forward [mm]
			case CMD_FORWARD:
				tmp = get_word();
				v = get_byte();

				motor_init();
				forward(tmp, v);

				break;

				// relative angle [degrees]
			case CMD_RELATIVE_ROTATE:
				tmp = get_word();

				motor_init();
				turn(tmp);
				break;
				
				// absolute angle [degrees]
			case CMD_ABSOLUTE_ROTATE:
				tmp = get_word();

				motor_init();
				rotate_absolute_angle(tmp);

				break;

				// rotate to and then move to point (Xc, Yc, v, direction) [mm]
			case CMD_TURN_AND_GO:
				
				tmpX = get_word();
				tmpY = get_word();
				v = get_byte();
				direction = get_byte(); // + means forward, - means backward

				motor_init();
				turn_and_go(tmpX, tmpY, v, direction); //(x, y, end_speed, direction)
				break;
				     
			case CMD_CURVE:
				tmpX = get_word();
				tmpY = get_word();
				
				tmpO = get_word();
				tmpSU = get_word();
				direction = get_byte();

				motor_init();
				arc(tmpX, tmpY, tmpO, tmpSU, direction);

				break;
			
				// x [mm], y [mm], direction {-1 - backwards, 0 - pick closest, 1 - forward}
			case CMD_MOVE_TO: {
				tmpX = get_word();
				tmpY = get_word();
				int direction = get_byte();
				int radius = 0x7fff;
				if(pkt->size >= 6) {
					radius = get_word();
				}
				
				motor_init();
				move_to(tmpX, tmpY, direction, radius);
				break;
			}
				// stop
			case CMD_HARD_STOP:
				stop();
				break;

				// stop and kill PWM
			case CMD_SOFT_STOP:
				motor_turn_off();
				break;
				
				// reset position, status and speed
			case CMD_RESET_DRIVER:
				reset_driver();
				break;

			default:
				force_status(STATUS_ERROR);
				break;
		}
		
		report_status();
	}

	return 0;
}
