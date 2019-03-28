#ifndef INTERRUPT_HPP
#define INTERRUPT_HPP
typedef void (*Func)();
extern Func funcs[2];
void on_interrupt(int n, Func func);

#endif
