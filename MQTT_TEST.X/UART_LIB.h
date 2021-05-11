#ifndef _UART_LIB_H    /* Guard against multiple inclusion */
#define _UART_LIB_H 
#include "UART_LIB.h"
#include <xc.h>
#include <sys/attribs.h>
//#include "GSM_MQTT.h"
#include "TIMING_MS.h"

// Remove plib warnings
#define _SUPPRESS_PLIB_WARNING
#include <plib.h>
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#define	GetSystemClock()              (80000000ul)
#define	GetPeripheralClock()          (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()         (GetSystemClock())

#define DESIRED_BAUDRATE_GPS              (9600)      //The desired BaudRate

#define DESIRED_BAUDRATE_MQTT             (115200)      //The desired BaudRate

#define UART_BUFFER_LENGTH 300    //Maximum length allowed for UART data

extern volatile int front;

extern volatile int end;

extern volatile int gpsFront;

extern volatile int gpsEnd;

extern volatile char newBuffer[UART_BUFFER_LENGTH];

extern volatile char gpsBuffer[UART_BUFFER_LENGTH];

void UARTinit();
void WriteString(const char *string);
void PutCharacter(const char character);

#endif /* _UART_LIB_H  */