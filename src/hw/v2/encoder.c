#include "../encoder.h"
void encoder_init(void)
{
	// O1 - QEI3
	QEI3CON = 0;
	QEI3CONbits.QEIEN = 1;
	
	// O2 - QEI1
	QEI1CON = 0;
	QEI1CONbits.QEIEN = 1;
	
	// M1 - QEI6
	QEI6CON = 0;
	QEI6CONbits.QEIEN = 1;
	
	// M2 - QEI5
	QEI5CON = 0;
	QEI5CONbits.QEIEN = 1;
}

void encoder_init_pins(void) {
	PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
	PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_PPS_REGISTERS);
	
	// O1
	TRISBbits.TRISB7 = 1;
	TRISBbits.TRISB8 = 1;
	
	// O2
	TRISAbits.TRISA0 = 1;
	TRISAbits.TRISA1 = 1;
	
	// M1
	TRISCbits.TRISC10 = 1;
	TRISCbits.TRISC13 = 1;
	
	// M2
	TRISAbits.TRISA11 = 1;
	TRISAbits.TRISA12 = 1;
	
	// O1 - QE3
	QEA3Rbits.QEA3R = 4; 	// B7
	QEB3Rbits.QEB3R = 4; 	// B8
	
	// O2 - QE1
	QEA1Rbits.QEA1R = 0;	// A0
	QEB1Rbits.QEB1R = 0; 	// A1
	
	// M1 - QE6
	QEA6Rbits.QEA6R = 9;	// C10
	QEB6Rbits.QEB6R = 9; 	// C13
	
	// M2 - QE5
	QEA5Rbits.QEA5R = 8;	// A11
	QEB5Rbits.QEB5R = 8; 	// A12
}

int encoder_odometry_left_get_velocity(void) {
	int r = -(int)VEL3CNT;
	return r;
}

int encoder_odometry_right_get_velocity(void) {
	int r = +(int)VEL1CNT;
	return r;
}

int encoder_motor_left_get_velocity(void){
	return VEL6CNT;
}

int encoder_motor_right_get_velocity(void){
	return VEL5CNT;
}

int encoder_odometry_left_get_count(void) {
	int r = +(int)POS3CNT;
	return r;
}

int encoder_odometry_right_get_count(void) {
	int r = +(int)POS1CNT;
	return r;
}

int encoder_motor_left_get_count(void){
	return POS6CNT;
}

int encoder_motor_right_get_count(void){
	return POS5CNT;
}
