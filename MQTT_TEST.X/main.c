// PIC32MX795F512L Configuration Bit Settings
// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FSRSSEL = PRIORITY_7     // SRS Select (SRS Priority 7)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config FCANIO = ON              // CAN I/O Pin Select (Default CAN I/O)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
//#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)

// DEVCFG1
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)


#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF              // Watchdog Timer Enable (WDT Disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#pragma config FPBDIV = DIV_1
#pragma config FNOSC = 3
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_20
#pragma config FPLLODIV = DIV_1
#pragma config FCKSM = 0x0
#pragma config POSCMOD = HS

// Remove plib warnings
#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

volatile int front;
volatile int end;
volatile int gpsFront;
volatile int gpsEnd;

#include <xc.h>
#include <sys/attribs.h>
#include <plib.h>
#include "TIMING_MS.h"
#include "UART_LIB.h"
#include "GSM_MQTT.h"
#include "IR_Remote_Read.h"
#include <time.h>
#include <math.h>

volatile char newBuffer[UART_BUFFER_LENGTH];
volatile char gpsBuffer[UART_BUFFER_LENGTH];

#define maxNumberTriggers 1

uint32_t captureTime = 0;
uint32_t firstEdge = 0;
uint32_t lastEdge = 0;
uint32_t period = 0;
float pulse_width = 0;
int flag = 0;
volatile int codeCount = 0;
volatile int arm_flag = 0;
volatile int data_on = 0;
uint8_t GSM_Response = 0;
int call_flag = 0;
volatile int panicFlag = 0;
volatile int triggerCount = 0;
const int alarmLength = 10;
float GPS_SEND_FREQ_SEC = 1.2;
int enteringDataMode = 0;
uint8_t myDeviceName[2] = {'\0'};

void __ISR(_EXTERNAL_1_VECTOR, IPL6SOFT) MyINT1Handler (void) //To do: raise priority to 7 with lowest subpriority
{
    triggerCount++;
    if (triggerCount > maxNumberTriggers)
    {
        panicFlag = 1;
        triggerCount = 0;
    }
    
    mINT1ClearIntFlag();
}

void portSetup()
{
    //LED set up
    TRISCbits.TRISC1 = 0;  //timer led
    LATCbits.LATC1 = 0;
    
    TRISEbits.TRISE0 = 0; //alarm signal
    LATEbits.LATE0 = 0; //set it to off
    
    TRISEbits.TRISE2 = 0; //Arm signal
    LATEbits.LATE2 = 0; //set it to off
    
    TRISEbits.TRISE4 = 0; //Proximity sensor enable, active low
    LATEbits.LATE4 = 0; //set it to off
    
    TRISEbits.TRISE6 = 0; // dataOn signal
    LATEbits.LATE6 = 0; //Off to begin with
}

void setupExternalInterrupt()
{
    __asm__("di");
    IEC0bits.INT1IE = 0; // disable INT1
    INTCONbits.INT1EP = 0; // clear the bit for falling edge trigger
    IFS0bits.INT1IF = 0; // clear the interrupt flag
    IPC1SET = 0x1B000000; // Setup priority for 6
    IPC1CLR = 0x20000000; // and set sub-priority to 3 
    // don't enable it yet, that is done in the remote control routine
    __asm__("ei");
}

int main(){
    long GPS_SEND_FREQ_MS = (long) ceil(GPS_SEND_FREQ_SEC*1000.0);
    portSetup();
    setupExternalInterrupt();
    analogCompInit();
    begin();
    int newdata  = 0;
    unsigned long long currentTime = millis();
    unsigned long long nextSendTime = millis() + GPS_SEND_FREQ_MS;
    unsigned long long currentTimeForCall = 0;
    unsigned long long nextTimeForCall = 0;
    int allowedToSendGps = 1;
    float avgLat = 0;
    float avgLon = 0;
    int index = 0; //This is the index for the lat and lon averaging.
    int tcpWasOn = 0; //boolean to tell if tcp was on when making a phone call
    int pingWasOn = 0; //boolean to tell if pingFlag was on
    int success_DL = 0;
    int success_Connect = 0;
    int done_with_call = 0; //says when its time to reconnect to mqtt if applicable
    int phoneCallMade = 0;
    int atCount = 0;
    int waitForHangup = 0;
    int waitForHangupTimeout = 0;
    int waitForHangupCurrentTime = 0;
    int alarmOn = 0; //bool that says if the alarm is on
    int panicCurrentTime = 0;
    int panicTimeTilDone = 0;
    int once = 0;
    int stopAlarm = 0;
    long timeOutDataMode = 0;
    long currentTimeData = 0;
    myDeviceName[0] = 1;
    while(1)
    {
        while(arm_flag == 1)
        {
            if((call_flag==0) )
            {
                processing(); //don't try connecting to mqtt while making phonecall
            }
            if (once == 0)
            {
                delay(250); //wait a bit for voltage to stabilize
                pingCount = 0; //set this to 0 to initialize
                once = 1;
                srand(millis());
                GPS_SEND_FREQ_MS = (int) ceil(GPS_SEND_FREQ_SEC*1000.0);
            }
            if((call_flag==0) )
            {
                processing(); //don't try connecting to mqtt while disarmed
            }
            if(call_flag)
            {
                if ((TCP_Flag == 1) && !done_with_call)
                {
                    TCP_Flag = 0; //turn it off temporarily that way we can use serial event
                    tcpWasOn = 1;
                }

                if ((pingFlag == 1) && !done_with_call)
                {
                    pingFlag = 0; //turn it off temporarily that way we can use serial event
                    pingWasOn = 1;
                }

                if(tcpWasOn && pingWasOn && !success_DL)
                { //we need to turn off direct link
                    currentTimeForCall = millis();
                    if (currentTimeForCall >= nextTimeForCall)
                    {
                        if(((_sendAT("+++", 1000) == 1) || (atCount >= 5)))
                        {
                            success_DL = 1;
                            atCount = 0;
                        }
                        else //turn off direct link
                        {
                            ResetNewBuffer();
                            delay(500);
                            atCount++;
                            nextTimeForCall = millis() + 500;
                        }  
                    }
                }
                else
                {
                    success_DL = 1; //we can say we are successful because direct link was never on to begin with
                }
                if(success_DL && !done_with_call)
                {
                    ResetNewBuffer();
                    //make phone call
                    if (!phoneCallMade)
                    {
                        WriteString("ATD5412952229;\r\n");
                        phoneCallMade = 1;
                    }
                    currentTimeForCall = millis();
                    if (currentTimeForCall >= nextTimeForCall)
                    {
                        if( atCount < 8)
                        {
                            WriteString("AT\r\n");
                            nextTimeForCall = millis() + 500;
                            atCount++;
                            ResetNewBuffer();
                            if (atCount == 8)
                            {
                                waitForHangupTimeout = millis() + 18500;
                            }
                        }
                        else
                        {
                            if ( (waitForHangup != 1) && (waitForHangupCurrentTime <= waitForHangupTimeout) )
                            {
                                if (end != 0)
                                {
                                    delay(100); //wait a bit for serial to come in
                                    waitForHangup = _sendAT("", 1000); // dont send any command, we just need to check for serial
                                }
                                waitForHangupCurrentTime = millis();
                            }
                            else
                            {
                                if (waitForHangupCurrentTime >= waitForHangupTimeout)
                                {
                                    //timeout occurred, we should hang up the call
                                    WriteString("AT+CHUP\r\n");
                                    delay(100);
                                    ResetNewBuffer();
                                }
                                done_with_call = 1;
                                atCount = 0;
                                nextTimeForCall = 0;
                            }
                        }
                    }               
                }
                if (done_with_call && tcpWasOn && pingWasOn && !success_Connect) //then connect to mqtt once more
                {
                    currentTimeForCall = millis();
                    if (currentTimeForCall >= nextTimeForCall)
                    {
                        if(!success_Connect && (atCount < 5))
                        {
                            ResetNewBuffer();
                            success_Connect = sendATreply("AT+USODL=0\r\n", "CONNECT", 1500);
                            atCount++;
                            nextTimeForCall = millis() + 500;
                        }
                    }
                    if (atCount == 5)
                    {
                        //connect did not go through
                        TCP_Flag = 0;
                        MQTT_Flag = 0;
                        pingFlag = 0;
                    }
                    else
                    {
                        TCP_Flag = tcpWasOn;
                        pingFlag = pingWasOn;
                    }
                    if (success_Connect) //we reconnected to mqtt and phone call is done
                    {
                        tcpWasOn = 0;
                        pingWasOn = 0;
                        done_with_call = 0;
                        success_DL = 0;
                        success_Connect = 0;
                        atCount = 0;
                        call_flag = 0;
                        phoneCallMade = 0;
                        waitForHangup = 0;
                        waitForHangupTimeout = 0;
                        waitForHangupCurrentTime = 0;
                    }
                }
                else if (done_with_call && tcpWasOn==0 && pingWasOn==0 && !success_Connect)
                {
                    TCP_Flag = tcpWasOn;
                    pingFlag = pingWasOn;
                    tcpWasOn = 0;
                    pingWasOn = 0;
                    done_with_call = 0;
                    success_DL = 0;
                    success_Connect = 0;
                    atCount = 0;
                    call_flag = 0;
                    phoneCallMade = 0;
                    waitForHangup = 0;
                    waitForHangupTimeout = 0;
                    waitForHangupCurrentTime = 0;
                }
            }
            if((call_flag==0) )
            {
                processing(); //don't try connecting to mqtt while disarmed
            }
            if (available())
            {
                if (allowedToSendGps == 0)
                {
                    if(currentTime >= nextSendTime)
                    {
                        allowedToSendGps = 1;
                        //turn on the GPS unit
                    }
                    else
                    {
                        currentTime = millis();
                    }
                    ClearGpsBuffer();
                }
                else //allowedToSendGps=1
                {
                    if (index < 3)
                    {
                        if(gpsEnd != 0)
                        {
                            delay(140); //wait for serial to come in
                            while (gpsEnd != 0) //serial is available
                            {
                                if(encode(gpsBuffer[gpsFront]))
                                    newdata = 1;
                                gpsFront++;
                                gpsEnd--;
                            }
                            ClearGpsBuffer();
                        }

                        if(newdata && (index < 3))
                        {
                            float flat, flon;
                            unsigned long age;
                            f_get_position(&flat, &flon, &age);
                            avgLat += flat;
                            avgLon += flon;
                            index++;
                        }
                    }

                    if(index >= 3)
                    {
                        avgLat /= 3;
                        avgLon /= 3;
                        char latitude[13] = {'\0'};
                        char longitude[13] ={'\0'};
                        char message[45] = {'\0'};
                        print_float(avgLat, 13, latitude);
                        print_float(avgLon, 13, longitude);
                        strcat(message, "lat: ");
                        strcat(message, latitude);
                        strcat(message, ", lon: ");
                        strcat(message, longitude);
                        if(available())
                        {
                            publish(0, 0, 0, _generateMessageID(), "GiselleLFreude/feeds/gps-coordinates-feed", message);
                        }
                        newdata = 0;
                        index = 0;
                        allowedToSendGps = 0;
                        avgLat = 0;
                        avgLon = 0;
                        nextSendTime = millis() + GPS_SEND_FREQ_MS;
                        //turn off the gps unit
                    }
                } // if allowedToSendGps 
            } 
            if((call_flag==0) )
            {
                processing(); //don't try connecting to mqtt while disarmed
            }

            if(panicFlag == 1)
            {
                //we have an alarm situation
                if (alarmOn == 0)
                {
                    alarmOn = 1;
                    LATEbits.LATE0 = 1; //turn the alarm on
                    panicTimeTilDone = millis() + alarmLength*1000;
                    call_flag = 1; //make that phone call
                }
                else
                {
                    //alarm is going
                    panicCurrentTime = millis();
                    if (panicCurrentTime >= panicTimeTilDone)
                    {
                        //break out of the alarm sequence
                        stopAlarm = 1;
                    }
                }
                if(arm_flag == 0)
                {
                    // turn it off!
                    stopAlarm = 0;
                }
                if (stopAlarm == 1)
                {
                    //breakout sequence
                    alarmOn = 0;
                    LATEbits.LATE0 = 0; //turn the alarm off
                    stopAlarm = 0;
                    panicFlag = 0;
                }
            }
            if((call_flag==0) )
            {
                processing(); //don't try connecting to mqtt while disarmed
            }
        }
        while ((arm_flag == 0) && (data_on == 0)) //if disarmed and data is not on
        {
            if (once == 1)
            {
                once = 0;
                delay(250);
                pingCount = 0; //set this to 0 to initialize
                mINT1ClearIntFlag(); //clear int1 flag
                IFS1CLR = 0x00000008; // Clear the CMP1 interrupt flag
            }
            if(alarmOn || panicFlag)
            {
                //breakout sequence
                alarmOn = 0;
                LATEbits.LATE0 = 0; //turn the alarm off
                stopAlarm = 0;
                panicFlag = 0;
            }
            while (call_flag) //if we are disarmed, then i don't want the device to execute the call sequence
            {
                if(success_Connect)
                {
                    disconnect();
                    WriteString("+++"); //disconnect the direct link
                    delay(100);
                    TCP_Flag = 0;
                    pingFlag = 0;
                    tcpWasOn = 0;
                    pingWasOn = 0;
                    done_with_call = 0;
                    success_DL = 0;
                    success_Connect = 0;
                    atCount = 0;
                    call_flag = 0;
                    phoneCallMade = 0;
                    waitForHangup = 0;
                    waitForHangupTimeout = 0;
                    waitForHangupCurrentTime = 0;
                    ResetNewBuffer();
                }
                else
                {
                    //In this scenario the call sequence has started
                    //we need to reconnect the direct link
                    currentTimeForCall = millis();
                    if (currentTimeForCall >= nextTimeForCall)
                    {
                        if(!success_Connect && (atCount < 5))
                        {
                            ResetNewBuffer();
                            success_Connect = sendATreply("AT+USODL=0\r\n", "CONNECT", 1500);
                            atCount++;
                            nextTimeForCall = millis() + 500;
                        }
                    }
                    if(atCount >= 5)
                    {
                        success_Connect = 1;
                    }
                }
            }
            if(call_flag == 0 && (TCP_Flag || MQTT_Flag || pingFlag)) //in this scenario we have disarmed, the panic sequence was never initiated and we are still connected
            {
                //disconnect gracefully
                disconnect();
                WriteString("+++"); //disconnect the direct link
                delay(100);
                ResetNewBuffer();
                TCP_Flag = 0;
                MQTT_Flag = 0;
                pingFlag = 0;
            }
            if(end!=0)
            {
                ResetNewBuffer(); 
            }
            if(gpsEnd!=0)
            {
                ClearGpsBuffer();
            }       
        }
        while (data_on) //user wants to update settings
        {
            if (enteringDataMode == 1)
            {
                pingCount = 0; //set this to 0 to initialize
                timeOutDataMode = 15 * 60 * 1000 + millis();
                enteringDataMode = 0;
                srand(millis());
            }
            else
            {
                currentTimeData = millis();
            }
            
            if (currentTimeData > timeOutDataMode)
            {
                data_on = 0;
                arm_flag = 0; //this better be off, but just in case
                LATEbits.LATE6 = 0; //Turn off dataOn signal
            }
            if(alarmOn || panicFlag)
            {
                //breakout sequence
                alarmOn = 0;
                LATEbits.LATE0 = 0; //turn the alarm off
                stopAlarm = 0;
                panicFlag = 0;
            }
            if(call_flag) //handle a call and reconnect if that is going on
            {
                if ((TCP_Flag == 1) && !done_with_call)
                {
                    TCP_Flag = 0; //turn it off temporarily that way we can use serial event
                    tcpWasOn = 1;
                }

                if ((pingFlag == 1) && !done_with_call)
                {
                    pingFlag = 0; //turn it off temporarily that way we can use serial event
                    pingWasOn = 1;
                }

                if(tcpWasOn && pingWasOn && !success_DL)
                { //we need to turn off direct link
                    currentTimeForCall = millis();
                    if (currentTimeForCall >= nextTimeForCall)
                    {
                        if(((_sendAT("+++", 1000) == 1) || (atCount >= 5)))
                        {
                            success_DL = 1;
                            atCount = 0;
                        }
                        else //turn off direct link
                        {
                            ResetNewBuffer();
                            delay(500);
                            atCount++;
                            nextTimeForCall = millis() + 500;
                        }  
                    }
                }
                else
                {
                    success_DL = 1; //we can say we are successful because direct link was never on to begin with
                }
                if(success_DL && !done_with_call)
                {
                    ResetNewBuffer();
                    //make phone call
                    if (!phoneCallMade)
                    {
                        WriteString("ATD5412952229;\r\n");
                        phoneCallMade = 1;
                    }
                    currentTimeForCall = millis();
                    if (currentTimeForCall >= nextTimeForCall)
                    {
                        if( atCount < 8)
                        {
                            WriteString("AT\r\n");
                            nextTimeForCall = millis() + 500;
                            atCount++;
                            ResetNewBuffer();
                            if (atCount == 8)
                            {
                                waitForHangupTimeout = millis() + 18500;
                            }
                        }
                        else
                        {
                            if ( (waitForHangup != 1) && (waitForHangupCurrentTime <= waitForHangupTimeout) )
                            {
                                if (end != 0)
                                {
                                    delay(100); //wait a bit for serial to come in
                                    waitForHangup = _sendAT("", 1000); // dont send any command, we just need to check for serial
                                }
                                waitForHangupCurrentTime = millis();
                            }
                            else
                            {
                                if (waitForHangupCurrentTime >= waitForHangupTimeout)
                                {
                                    //timeout occurred, we should hang up the call
                                    WriteString("AT+CHUP\r\n");
                                    delay(100);
                                    ResetNewBuffer();
                                }
                                done_with_call = 1;
                                atCount = 0;
                                nextTimeForCall = 0;
                            }
                        }
                    }               
                }
                if (done_with_call && tcpWasOn && pingWasOn && !success_Connect) //then connect to mqtt once more
                {
                    currentTimeForCall = millis();
                    if (currentTimeForCall >= nextTimeForCall)
                    {
                        if(!success_Connect && (atCount < 5))
                        {
                            ResetNewBuffer();
                            success_Connect = sendATreply("AT+USODL=0\r\n", "CONNECT", 1500);
                            atCount++;
                            nextTimeForCall = millis() + 500;
                        }
                    }
                    if (atCount == 5)
                    {
                        //connect did not go through
                        TCP_Flag = 0;
                        MQTT_Flag = 0;
                        pingFlag = 0;
                    }
                    else
                    {
                        TCP_Flag = tcpWasOn;
                        pingFlag = pingWasOn;
                    }
                    if (success_Connect) //we reconnected to mqtt and phone call is done
                    {
                        tcpWasOn = 0;
                        pingWasOn = 0;
                        done_with_call = 0;
                        success_DL = 0;
                        success_Connect = 0;
                        atCount = 0;
                        call_flag = 0;
                        phoneCallMade = 0;
                        waitForHangup = 0;
                        waitForHangupTimeout = 0;
                        waitForHangupCurrentTime = 0;
                    }
                }
                else if (done_with_call && tcpWasOn==0 && pingWasOn==0 && !success_Connect)
                {
                    TCP_Flag = tcpWasOn;
                    pingFlag = pingWasOn;
                    tcpWasOn = 0;
                    pingWasOn = 0;
                    done_with_call = 0;
                    success_DL = 0;
                    success_Connect = 0;
                    atCount = 0;
                    call_flag = 0;
                    phoneCallMade = 0;
                    waitForHangup = 0;
                    waitForHangupTimeout = 0;
                    waitForHangupCurrentTime = 0;
                }
            }
            processing(); //we are going to want to connect to mqtt or continue a connection if we are still connected
        }
    }
    return 0;
}
