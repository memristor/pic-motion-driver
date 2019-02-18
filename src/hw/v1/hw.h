#define FCY 29491200ULL
#include <p33FJ128MC802.h>
#include <libpic30.h>
#define interrupt_lock SRbits.IPL = 7;
#define interrupt_unlock SRbits.IPL = 0;
// #define INTERRUPT __attribute__((interrupt(auto_psv)))
#define INTERRUPT __attribute__((interrupt(auto_psv)))
#define DMA_SPACE __attribute__((space(dma)))

typedef long double ldouble;
#define dbg(...)
