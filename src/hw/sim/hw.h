#define interrupt_lock
#define interrupt_unlock
#define INTERRUPT
#define __delay_ms(t) usleep(t*1000)

#include <unistd.h>
#include <stdio.h>
typedef double ldouble;
#define dbg(...) printf(__VA_ARGS__)
// #define __delay_ms(x) usleep(x*1000)
