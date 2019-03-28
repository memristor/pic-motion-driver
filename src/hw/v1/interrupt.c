#include "../interrupt.h"
#include "hw.h"

 void INTERRUPT _INT1Interrupt(void) {
	///*
	//#ifndef SIM
	//IFS1bits.INT1IF = 0;
	 //RUN_EACH_NTH_CYCLES(uint16_t, 200, {
		
		//start_packet('X');
		//end_packet();
	 //})
	//#endif
	//*/
	if (funcs[0]) {
		funcs[0]();
	}
 }

 void INTERRUPT _INT2Interrupt(void) {
	 start_packet('x');
	 end_packet();
	if (funcs[1]) {
		funcs[1]();
	}
	///*
	//#ifndef SIM
	//IFS1bits.INT2IF = 0;
	 //RUN_EACH_NTH_CYCLES(uint16_t, 200, {
		
		//start_packet('x');
		//end_packet();
	 //})
	//#endif
	//*/
 }
