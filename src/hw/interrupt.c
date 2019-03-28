#include "interrupt.h"

Func funcs[2] = {0,0};

void on_interrupt(int n, Func func) {
	funcs[n] = func;
}
