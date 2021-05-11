/*******************************************************************************
 *  FileName:
 *      timing.c
 *  Description: 
 *      File defines a simple millisecond timer with interrupt for use with
 *      real time requirements
 ******************************************************************************/


// Remove plib warnings
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _SUPPRESS_PLIB_WARNING
#include "TIMING_MS.h"



#define	GetSystemClock() 			(80000000ul)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))

#define TIMER_ON_BIT    0x8000
#define TICK_MILLIS     10000U // 10 Mhz bus -> 10,000 count/millisecond

volatile unsigned long long milli_reg = 0;

void InitTiming(void) // Setup the system timing buses
{
    INTCONbits.MVEC = 1; //Setup multi-vectored mode
    T2CON = 0x00000000; //Clear the settings, prescaler is 1:1
    T4CON = 0x00000000;
    TMR2 = 0x00000000; //set timer2 to zero
    TMR3 = 0x00000000; //set timer3 to zero
    TMR4 = 0x00000000; //set timer4 to zero
    TMR5 = 0x00000000; //set timer5 to zero
    IFS0 = 0x00000000; // Clear all interrupt flags
    IEC0CLR = _IEC0_T4IE_MASK; //disable timer 4 interrupt
    IEC0CLR = _IEC0_T5IE_MASK; //disable timer 5 interrupt
    PR4 = 0x3880;
    PR5 = 0x0001;
    IPC5SET = 0x0000001F; // Set priority to 7, subpriority to 3, highest possible priority
    T4CONbits.T32 = 1; //turn on T32 mode for T45
    T2CONbits.T32 = 1; //turn on T32 mode for T23 as well
    //LED set up
    TRISCbits.TRISC1 = 0; 
    LATCbits.LATC1 = 1;
    IEC0bits.T5IE = 1; // Enable TMR32 interrupt
    __asm__("ei"); //globally enable all interrupts
    T2CONSET = 0x8000; // Start the timer 2
    T4CONSET = 0x8000; // Start the timer 4
}
void toggleLED()
{
    if(LATCbits.LATC1 == 1){
        LATCbits.LATC1 = 0;
    }
    else{
        LATCbits.LATC1 = 1;
    }
}
    
void delay(uint32_t millis)    // Busy wait for specified number of Milliseconds
{
     millis += milli_reg;    
     while(millis > milli_reg);   // Busy wait
}
inline const uint32_t millis()
{
    return milli_reg;
}// Return the number of milliseconds since start

void __ISR(_TIMER_5_VECTOR, IPL7SOFT) myTimer5Handler(void) 
{
    milli_reg++;
    IFS0CLR = _IFS0_T5IF_MASK;
    if (milli_reg % 1000 == 0){
        toggleLED();
    }
}