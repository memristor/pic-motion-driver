#include <xc.h>
#define interrupt_lock INTSTATbits.SRIPL = 7;
#define interrupt_unlock INTSTATbits.SRIPL = 0;
#define INTERRUPT
#define DMA_SPACE
#define __delay_ms(t)
#include "peripheral.h"
#define BOOT 
typedef long double ldouble;
#define printf(...)
