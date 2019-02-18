// PIC32MK1024MCF064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PWMLOCK = OFF            // PWM IOxCON lock (PWM IOxCON register writes accesses are not locked or protected)
#pragma config FUSBIDIO2 = ON           // USB2 USBID Selection (USBID pin is controlled by the USB2 module)
#pragma config FVBUSIO2 = ON            // USB2 VBUSON Selection bit (VBUSON pin is controlled by the USB2 module)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration bit (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO1 = ON           // USB1 USBID Selection (USBID pin is controlled by the USB1 module)
#pragma config FVBUSIO1 = ON            // USB2 VBUSON Selection bit (VBUSON pin is controlled by the USB1 module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_13_26_MHZ// System PLL Input Range (13-26 MHz Input)
#pragma config FPLLICLK = PLL_POSC      // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT = MUL_30        // System PLL Multiplier (PLL Multiply by 30)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (4x Divider)
#pragma config VBATBOREN = ON           // VBAT BOR Enable (Enable ZPBOR during VBAT Mode)
#pragma config DSBOREN = ON             // Deep Sleep BOR Enable (Enable ZPBOR during Deep Sleep Mode)
#pragma config DSWDTPS = DSPS32         // Deep Sleep Watchdog Timer Postscaler (1:2^36)
#pragma config DSWDTOSC = LPRC          // Deep Sleep WDT Reference Clock Selection (Select LPRC as DSWDT Reference clock)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer Enable (Disable DSWDT during Deep Sleep Mode)
#pragma config FDSEN = OFF              // Deep Sleep Enable (Disable DSEN bit in DSCON)
#pragma config BORSEL = HIGH            // Brown-out trip voltage (BOR trip voltage 2.1v (Non-OPAMP deviced operation))
#pragma config UPLLEN = OFF             // USB PLL Enable (USB PLL Disabled)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disable Secondary Oscillator)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_LEVEL_3  // Primary Oscillator Gain Control bits (Gain Level 3 (highest))
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot Enable (Normal EJTAG functionality)

// DEVCP
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#include <xc.h>
 
//#include <plib.h> 

#define SYS_FREQ		(120000000)

//void DelayMs(unsigned int msec);

void main(){
    TRISB=0x00000000;
    TRISA=0x00000000;
    long unsigned cnt = 0;
    while(1){
        cnt = 0;
        LATA = 0x0000FFFF;
        LATB = 0x0000FFFF;
        while(cnt!=250000)
            cnt++;
        cnt = 0;
        LATA = 0x00000000;
        LATB = 0x00000000;
        while(cnt!=250000)
            cnt++;
    }
}

/******************************************************************************
*	DelayMs()
*
*	This functions provides a software millisecond delay
******************************************************************************/
/*void DelayMs(unsigned int msec)
{
	unsigned int tWait, tStart;
		
    tWait=(SYS_FREQ/2000)*msec;
    tStart=ReadCoreTimer();
    while((ReadCoreTimer()-tStart)<tWait);		// wait for the time to pass

}*/
