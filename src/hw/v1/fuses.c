// #include "bootloader.h"
#ifdef USE_FRCPLL
	#pragma config FWDTEN = OFF, FNOSC = FRCPLL
#else
	// #pragma config FWDTEN = OFF, FNOSC = PRIPLL, IESO=ON, OSCIOFNC=OFF, FCKSM=CSDCMD, POSCMD=HS
	#pragma config FWDTEN = OFF, FNOSC = FRC, IESO=ON, OSCIOFNC=OFF, FCKSM=CSECMD, POSCMD=HS
#endif
