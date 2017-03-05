#include <p33FJ128MC802.h>
void QEIinit(void)
{
	QEI1CONbits.POSRES=0;       //index impuls ne resetuje brojac
	QEI1CONbits.TQCS=1;         //brojac broji impulse sa QEA ulaza
	QEI1CONbits.UPDN_SRC=1;     //za to vreme QEB odredjuje smer brojanja
	QEI1CONbits.QEIM=6;         //Quadrature Encoder Interface enabled (x4 mode) with index pulse reset of position counter
	QEI1CONbits.TQCKPS=0;

	MAX1CNT=0000;
	POS1CNT=0;

	QEI2CONbits.POSRES=0;       //index impuls ne resetuje brojac
	QEI2CONbits.TQCS=1;         //brojac broji impulse sa QEA ulaza
	QEI2CONbits.UPDN_SRC=1;     //za to vreme QEB odredjuje smer brojanja
	QEI2CONbits.QEIM=6;         //Quadrature Encoder Interface enabled (x4 mode) with index pulse reset of position counter
	QEI2CONbits.TQCKPS=0;

	MAX2CNT=0000;
	POS2CNT=0;
}
