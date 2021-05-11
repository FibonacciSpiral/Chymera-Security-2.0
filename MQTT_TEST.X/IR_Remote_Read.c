/* This file contains an initialization function for the input capture module
  and a function to read the input signal
 */

// Remove plib warnings
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _SUPPRESS_PLIB_WARNING

#include "IR_Remote_Read.h"
#include <math.h>

#define tolerance 0.085; //85 microseconds
int startBitFound = 0;
float startBitUpperBound = 2.4 + tolerance;
float startBitLowerBound = 2.4 - tolerance;
float highBitUpperBound = 1.8 + tolerance;
float highBitLowerBound = 1.8 - tolerance;
float lowBitUpperBound = 1.2 + tolerance;
float lowBitLowerBound = 1.2 - tolerance;
float stopBitUpperBound = 3.0 + tolerance;
float stopBitLowerBound = 3.0 - tolerance;
int bitCount = 0;
uint16_t myDataBuffer = 0;

//Remote receiver input is pin 49 on pic32

void initInputCapture()
{
    asm("di");
    DisableIntIC4;
    OpenCapture4( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_CAP_32BIT | IC_FEDGE_FALL | IC_ON);
    ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_6 | IC_INT_SUB_PRIOR_3);
    mIC4ReadCapture(); //read dummy interrupt that gets triggered from enabling IC module
    mIC4ClearIntFlag();
    EnableIntIC4;
    asm("ei");
}

void __ISR(_INPUT_CAPTURE_4_VECTOR, IPL6SOFT) myInputCapture4Handler(void)
{
    if( mIC4CaptureReady() )
    {
        captureTime = mIC4ReadCapture();
        flag = 0;
        if(PORTDbits.RD11 == 0) //then this is the first edge
            firstEdge = captureTime;
        else 
        {
            lastEdge = captureTime; //this is the last edge of the pulse
            period = (lastEdge - firstEdge); //This isn't exactly a period. Its the number of ticks in the pulse
            pulse_width = (float)period;
            pulse_width = (pulse_width / 80000000) * 1000;
            // Now we have a pulse measurement
            // Is the measurement a start bit?
            if (startBitFound)
            {
                if (bitCount > 14)
                {
                    //invalid data was grabbed. oops we need to abort
                    startBitFound = 0;
                    myDataBuffer = 0;
                    bitCount = 0;
                }
                else if ((pulse_width < lowBitUpperBound) && (pulse_width > lowBitLowerBound))
                {
                    //then its a zero
                    bitCount++;
                }
                else if ((pulse_width < highBitUpperBound) && (pulse_width > highBitLowerBound))
                {
                    //then its a one
                    myDataBuffer = myDataBuffer + (1 << bitCount);
                    bitCount++;
                }
                else if ((pulse_width < stopBitUpperBound) && (pulse_width > stopBitLowerBound))
                {
                    //then its a stop bit
                    uint16_t codeFound = myDataBuffer >> 2;
                    int codeVerified = 0;
                    int i;
                    for(i = 0; i < 64; i++)
                    {
                        if (codeFound == rollingCode[codeCount+i] )
                        {
                            codeVerified = 1;
                            codeCount = codeCount + i;
                            codeCount++;
                            break;
                        }
                    }
                    if (codeVerified)
                    {
                        uint8_t command = myDataBuffer & 0x0003;
                        if(command == 1)
                        {
                            //arm command
                            CMP1ConfigInt(CMP_INT_ENABLE | CMP_INT_PRIOR_7 | CMP_INT_SUB_PRI_1); // Enable CMP1
                            IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag
                            mINT1ClearIntFlag(); //clear int1 flag
                            LATEbits.LATE4 = 1; //turn on proximity sensor
                            IEC0bits.INT1IE = 1; // enable INT1
                            arm_flag = 1;
                            data_on = 0;
                            LATEbits.LATE2 = 1; //Turn on arm signal
                            LATEbits.LATE6 = 0; //Turn off dataOn signal
                        }
                        else if (command == 2)
                        {
                            //disarm command
                            IEC0bits.INT1IE = 0; // disable INT1
                            CMP1ConfigInt(CMP_INT_DISABLE | CMP_INT_PRIOR_7 | CMP_INT_SUB_PRI_1); // disable CMP1
                            IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag
                            LATEbits.LATE4 = 0; //set proximity enable to off
                            arm_flag = 0;
                            data_on = 0;
                            LATEbits.LATE2 = 0; //Turn off arm signal
                            LATEbits.LATE6 = 0; //Turn off dataOn signal
                            mINT1ClearIntFlag(); //clear int1 flag
                            IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag
                        }
                        else if (command == 3)
                        {
                            //data on command
                            enteringDataMode = 1;
                            IEC0bits.INT1IE = 0; // disable INT1
                            LATEbits.LATE4 = 0; //set proximity enable to off
                            CMP1ConfigInt(CMP_INT_DISABLE | CMP_INT_PRIOR_7 | CMP_INT_SUB_PRI_1); // disable CMP1
                            IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag
                            data_on = 1;
                            arm_flag = 0; // you wouldn't want both of these on at the same time
                            LATEbits.LATE6 = 1; //Turn on dataOn signal
                            LATEbits.LATE2 = 0; //Turn off arm signal
                            mINT1ClearIntFlag(); //clear int1 flag
                            IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag 
                        }
                        else
                        {
                            //garbage command?
                        }
                    }
                    startBitFound = 0;
                    myDataBuffer = 0;
                    bitCount = 0;
                }
                else
                {
                    //what the heck is this?
                }
            }
            else
            {
                //check to see if we have a start bit
                if ((pulse_width < startBitUpperBound) && (pulse_width > startBitLowerBound))
                {
                    //then a start bit was found

                    startBitFound = 1;
                    myDataBuffer = 0; //clear it out
                    bitCount = 0;
                }
                else
                {
                    startBitFound = 0;
                }
            }
        }
    }
    mIC4ClearIntFlag();
}
