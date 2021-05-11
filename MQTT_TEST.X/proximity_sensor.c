/* This file contains an initialization function for the proximity sensor module
  and an ISR to respond to a comparator event
 */

// Remove plib warnings
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _SUPPRESS_PLIB_WARNING

#include "proximity_sensor.h"
#include <plib.h>

void analogCompInit()
{
    asm("di");
    CVRCON |= 8; //setting CVR to 8, 1.65 v threshold
    CVRCONbits.CVRR = 0;
    CVRCONbits.ON = 1;
    CMP1Close(); //turn it off while working on it
    CMP1ConfigInt(CMP_INT_DISABLE | CMP_INT_PRIOR_7 | CMP_INT_SUB_PRI_1);
    CMP1Open(CMP_ENABLE | CMP_OUTPUT_DISABLE | CMP_OUTPUT_INVERT |
    CMP_EVENT_HIGH_TO_LOW | CMP_POS_INPUT_CVREF | CMP1_NEG_INPUT_C1IN_POS );
    IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag
    asm("ei");
}

void __ISR(_COMPARATOR_1_VECTOR, IPL7SOFT)Cmp1_IntHandler(void)
{
    panicFlag = 1;
    IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag
}