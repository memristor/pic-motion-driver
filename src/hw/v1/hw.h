#define FCY 29491200ULL
#include <p33FJ128MC802.h>
#include <libpic30.h>
#include "../interrupt.h"
#define interrupt_lock if(!is_interrupt) {SRbits.IPL = 7;}
#define interrupt_unlock if(!is_interrupt) {SRbits.IPL = 0;}
// #define INTERRUPT __attribute__((interrupt(auto_psv)))
#define INTERRUPT __attribute__((interrupt(auto_psv)))
#define DMA_SPACE __attribute__((space(dma)))
#define printf(...)
typedef long double ldouble;
