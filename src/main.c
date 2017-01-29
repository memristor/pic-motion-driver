/*
 * Kod na eurobot-u; Ogranicena max brzina na 180;
 *
 * Comment: X napred pravo; Y bocno (na desno); orjentacija pozitivan direction u odnosu na X (clockwise)
 */
#define FCY 29491200ULL

#include "regulacija.h"
#include "uart.h"
#include "pwm.h"
#include "init.h"
#include <libpic30.h>
#include <p33FJ128MC802.h>

#pragma config FWDTEN = OFF, \
			   FNOSC = FRCPLL


int main(void)
{
	int tmpX, tmpY, tmp, tmpO;

	char komanda, v, direction, tmpSU;

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
	RPINR18bits.U1RXR = 0;		//UART1 RX na RP0- pin 4
	RPOR0bits.RP1R = 3;			//UART1 TX na RP1- pin 5
	RPINR14bits.QEA1R = 2;		//QEI1A na RP2
	RPINR14bits.QEB1R = 3;		//QEI1B na RP3

	RPINR16bits.QEA2R = 4;		//QEI2A na RP4
	RPINR16bits.QEB2R = 7;		//QEI2B na RP7

	__builtin_write_OSCCONL(OSCCON | (1<<6));

	//INTCON1bits.NSTDIS = 1; // zabranjeni ugnjezdeni prekidi

	PortInit();

	UART_Init(57600);
	TimerInit();
	QEIinit();
	
	CloseMCPWM();
	PWMinit();
	
	resetDriver();

	setSpeed(0x32); //poc brz je 10

	while(1)
	{
		// komanda = getch();
		uint8_t len;
		
		if(!try_read_packet((uint8_t*)&komanda, &len)) continue;
		
		switch(komanda)
		{
			// zadavanje trenutne pozicije
			case 'I':
				
				// tmpX = getint16();
				// tmpY = getint16();
				// tmpO = getint16();
				tmpX = get_word();
				tmpY = get_word();
				tmpO = get_word();
				setPosition(tmpX, tmpY, tmpO);

				break;
				
			case 'd':
				// debug_level(getch());
				debug_level(get_byte());
				break;
				
				// read status and position
			case 'P':
				sendStatusAndPosition();
				break;


				// set speed; Vmax(0-255)
			case 'V':
				// tmp = getch();
				tmp = get_byte();
				setSpeed(tmp);
				break;

			case 'r':
				// setRotationSpeed(getch(), getch());
				setRotationSpeed(get_byte(), get_byte());
				break;
				
				
				// move forward [mm]
			case 'D':
				// tmp = getint16();
				// v = getch();
				tmp = get_word();
				v = get_byte();

				PWMinit();
				forward(tmp, v);

				break;

				//relativni ugao [stepen]
			case 'T':
				// tmp = getint16();
				tmp = get_word();

				PWMinit();
				turn(tmp);
				break;
				
				//apsolutni ugao [stepen]
			case 'A':
				// tmp = getint16();
				tmp = get_word();

				PWMinit();
				rotate_absolute_angle(tmp);

				break;

				// rotate to and then move to point (Xc, Yc) [mm]
			case 'G':
				// tmpX = getint16();
				// tmpY = getint16();
				// v = getch();
				// direction = getch(); // + means forward, - means backward
				tmpX = get_word();
				tmpY = get_word();
				v = get_byte();
				direction = get_byte(); // + means forward, - means backward

				PWMinit();
				turn_and_go(tmpX, tmpY, v, direction); //(x, y, end speed, direction)
				break;
				     
			case 'Q':
				// tmpX = getint16();
				// tmpY = getint16();
				
				// tmpO = getch();
				// tmpSU = getch();
				// direction = getch();
				tmpX = get_word();
				tmpY = get_word();
				
				tmpO = get_word();
				tmpSU = get_word();
				direction = get_byte();

				PWMinit();
				luk(tmpX, tmpY, tmpO, tmpSU, direction);

				break;
			
			case 'N': {
				
				// tmpX = getint16();
				// tmpY = getint16();
				// int direction = getch();
				tmpX = get_word();
				tmpY = get_word();
				int direction = get_byte();
				
				PWMinit();
				move_to(tmpX, tmpY, direction);
				
				break;
			}
				//ukopaj se u mestu
			case 'S':
				stop();

				break;

				//stani i ugasi PWM; budi mlitav
			case 's':
				stop();
				CloseMCPWM();

				break;
				
				//nicemu ne sluzi, resetuje vrednosti svih
			case 'R':
				resetDriver();

				break;

			case 'z':
				iskljuciZaglavljivanje();
				break;

			case 'Z':
				ukljuciZaglavljivanje();

				break;
			default:
				forceStatus(STATUS_ERROR);
				break;
		}
	}

	return 0;
}
