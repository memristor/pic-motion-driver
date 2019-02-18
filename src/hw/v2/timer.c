#include "../timer.h"
#include "sys/attribs.h"


// void __ISR(_CORE_TIMER_VECTOR, IPL2SOFT) timer_interrupt() {
void __ISR(_TIMER_1_VECTOR, IPL2SOFT) timer_interrupt() {
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_1);
	regulator_interrupt();
}

void timer_init(void)
{
	#define MY_TMR TMR_ID_1
	PLIB_TMR_Stop(MY_TMR);
	PLIB_TMR_Counter16BitClear(MY_TMR);
	PLIB_TMR_ClockSourceSelect ( MY_TMR, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK );
	PLIB_TMR_PrescaleSelect(MY_TMR, TMR_PRESCALE_VALUE_64);
	PLIB_TMR_Mode16BitEnable(MY_TMR);
	PLIB_TMR_Period16BitSet(MY_TMR, 1875); // 1kHz
	PLIB_TMR_Start(MY_TMR);
	
	PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_1);
	PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_1);
	PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_T1, INT_PRIORITY_LEVEL1);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0); 
    PLIB_INT_Enable(INT_ID_0);
}
