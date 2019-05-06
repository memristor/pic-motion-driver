#include "../interrupt.h"
#include "hw.h"

void INTERRUPT _INT1Interrupt(void) {
	if (funcs[0]) {
		funcs[0]();
	}
}

void INTERRUPT _INT2Interrupt(void) {
	if (funcs[1]) {
		funcs[1]();
	}
}
