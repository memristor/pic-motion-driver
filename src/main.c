/*
 * Kod na eurobot-u; Ogranicena max brzina na 180;
 *
 * Comment: X napred pravo; Y bocno (na desno); orjentacija pozitivan smer u odnosu na X (clockwise)
 */
#define FCY 29491200ULL

#include "regulacija.h"
#include "uart.h"
#include "pwm.h"
#include "init.h"
#include <libpic30.h>
#include <p33FJ128MC802.h>


// _FWDT (FWDTEN_OFF);
// _FOSCSEL(FNOSC_FRCPLL);			// Internal FRC oscillator with PLL


#pragma config FWDTEN = OFF, \
			   FNOSC = FRCPLL


int main(void)
{
	int tmpX, tmpY, tmp, tmpO;

	char komanda, v, smer, tmpSU;

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
	// LATBbits.LATB8 = 1;
	// LATBbits.LATB9 = 0;
	// LATBbits.LATB15 = 0;
	// LATBbits.LATB14 = 1;
	// while(1) {
		
	// }
	UART_Init(57600);
	TimerInit();
	QEIinit();
	
	//Ovo je bilo zakomentarisano
	CloseMCPWM();
	PWMinit();
	
	resetDriver();

	//setSpeed(0x80); //0x80 = 128 ; 0x64 = 100 0x32 = 50 0x0A = 10
	setSpeed(0x32); //poc brz je 10 /*Milos: podesavanje pocetne brzine*/

	//setSpeedAccel(K2;	//K2 je za 1m/s /bilo je 2
	//ima li efekta podesavati setSpeedAccel nakon setSpeed?

	//CloseMCPWM();//da ne drzi poziciju kada se upali, jer se motorcina cuje, skripi

	// while(1)
	// {gotoXY(0,0,100,1);}
	while(1)
	{
		// if(getStatus() == STATUS_MOVING)
			// komanda = UART_GetLastByte();
		// else
		komanda = getch();

		switch(komanda)
		{
			// zadavanje trenutne pozicije
			case 'I':
				// tmpX = (getch() << 8) | getch();
				// tmpY = (getch() << 8) | getch();
				// tmpO = (getch() << 8) | getch();
				tmpX = getint16();
				tmpY = getint16();
				tmpO = getint16();

				setPosition(tmpX, tmpY, tmpO);

				break;
				
			case 'd':
				debug_level(getch());
				break;
				
				// citanje pozicije i statusa
			case 'P':
				sendStatusAndPosition();
				break;


				//zadavanje max. brzine (default K2/2); VminMax(0-255)
			case 'V':
				tmp = getch();
				setSpeed(tmp);
				break;

			case 'r':
				setRotationSpeed(getch(), getch());
				break;
				//kretanje pravo [mm]
				// 300mm napred; HEX: 44012c00
			case 'D':
				// tmp = getch() << 8;
				// tmp |= getch();
				tmp = getint16();
				v = getch();

				PWMinit();
				kretanje_pravo(tmp, v);//(distanca, krajnja brzina); krajnja brzina uvek 0
				/*Milos: Moze li se iskoristiti da je v != 0???*/

				break;

				//relativni ugao [stepen]
			case 'T':
				// tmp = getch() << 8;
				// tmp |= getch();
				tmp = getint16();

				PWMinit();
				okret(tmp);

				// putch('T');//dodao
				// putch(tmp >> 8); //dodao
				// putch(tmp);


				break;
				
				//apsolutni ugao [stepen]
			case 'A':
				// tmp = getch() << 8;
				// tmp |= getch();
				tmp = getint16();

				PWMinit();
				rotiraj_robota_apsolutni_ugao(tmp);

				break;

				//idi u tacku (Xc, Yc) [mm]
			case 'G':
				// tmpX = getch() << 8;
				// tmpX |= getch();
				tmpX = getint16();
				// tmpY = getch() << 8;
				// tmpY |= getch();
				tmpY = getint16();
				v = getch();
				smer = getch(); //guzica VS glava;bilo koji + broj jedan smer; -broj drugi smer

				PWMinit();
				gotoXY(tmpX, tmpY, v, smer); //(x koordinata, y koordinata, v=krajnja brzina, smer kretanja)

				break;

				     
			case 'Q':
				// tmpX = getch() << 8;
				// tmpX |= getch();
				tmpX = getint16();
				// tmpY = getch() << 8;
				// tmpY |= getch();
				tmpY = getint16();
				
				
				//                tmpO = getch() << 8;
				//                tmpO |= getch();      //ovako je bilo, mislim da nije ok                
				tmpO = getch();         //jer je Fi INT, a ne long
				tmpSU = getch();        //ovim se odredjuje smer ugla
				smer = getch();

				PWMinit();
				luk(tmpX, tmpY, tmpO, tmpSU, smer);

				break;
			
			case 'N': {
				
				tmpX = getint16();
				tmpY = getint16();
				int direction = getch();
				PWMinit();
				move_to(tmpX, tmpY, direction);
				
				/*
				// tmpX = getch() << 8;
				// tmpX |= getch();
				tmpX = getint16();
				// tmpY = getch() << 8;
				// tmpY |= getch();
				tmpY = getint16();
				v = getch();
				smer = getch(); //guzica VS glava;bilo koji + broj jedan smer; -broj drugi smer

				PWMinit();
				gotoXY(tmpX, tmpY, v, smer); //(x koordinata, y koordinata, v=krajnja brzina, smer kretanja)
				*/
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
