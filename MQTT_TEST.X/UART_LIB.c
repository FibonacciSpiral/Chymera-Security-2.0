// Handle all UART interface
#include "UART_LIB.h"

// TO DO!!!!!!!!!!! Change priorities of UART interrupts!!! GPS should be lower priority than MQTT interrupt and both should be lower priority than remote interrupt
void ResetNewBuffer();
void ClearGpsBuffer();

//Configures the UART interface for the cellular module and another UART interface for
//the GPS module. 
//Cellular module UART pins: Rx, TX -> (0, 1))
//GPS module UART pins: Rx, Tx -> (17, 16)

void UARTinit()
{
    ResetNewBuffer();
    ClearGpsBuffer();
    INTEnableSystemMultiVectoredInt();
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, GetSystemClock(), DESIRED_BAUDRATE_MQTT);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    // Configure UART1 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);
    
    //setup uart2
    INTEnableSystemMultiVectoredInt();
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, GetSystemClock(), DESIRED_BAUDRATE_GPS);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    // Configure UART2 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_3);
    
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);

    // configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

  
    //setup led(RA3) which is toggled at every RX interrupt
    mPORTAClearBits(BIT_3);
    mPORTASetPinsDigitalOut(BIT_3);

    // configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    // enable interrupts
    INTEnableInterrupts();

    // wait some time until the UART is done setting up connection
    int i = 100000;
    while(i)
    {
      asm volatile("nop");
      i--;
    }
}

// helper uart functions
void WriteString(const char *string)
{
  while (*string != '\0')
    {
      while(!UARTTransmitterIsReady(UART1));
      
      UARTSendDataByte(UART1, *string);
      string++;
    }
  while(!UARTTransmissionHasCompleted(UART1));
}

void PutCharacter(const char character)
{
  while (!UARTTransmitterIsReady(UART1));
  UARTSendDataByte(UART1, character);
  while (!UARTTransmissionHasCompleted(UART1));
}

void writeToGps(const char *string)
{
  while (*string != '\0')
    {
      while(!UARTTransmitterIsReady(UART2));
      
      UARTSendDataByte(UART2, *string);
      string++;
    }
  while(!UARTTransmissionHasCompleted(UART2));
}

void PutCharacterToGps(const char character)
{
  while (!UARTTransmitterIsReady(UART2));
  UARTSendDataByte(UART2, character);
  while (!UARTTransmissionHasCompleted(UART2));
}


void ResetNewBuffer()
{
    front = 0;
    end = 0;
    int i = 0;
    while(i < UART_BUFFER_LENGTH)
    {
        newBuffer[i]= '\0';
        i++;
    }
}

void ClearGpsBuffer()
{
    gpsFront = 0;
    gpsEnd = 0;
    int i = 0;
    while(i < UART_BUFFER_LENGTH)
    {
        gpsBuffer[i]= '\0';
        i++;
    }
}
 
 void __ISR(24, IPL2SOFT) IntUart1Handler(void)
 {
   // Is this an RX interrupt?
  if (INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
      // Clear the RX interrupt Flag
      INTClearFlag(INT_SOURCE_UART_RX(UART1));
      
      
      // Echo what we just received.
      //PutCharacter(character);
      
      //add received character to new (larger) buffer
      

      while(U1STAbits.URXDA==1) //if the buffer has stuff in it
      {
         //copy the stuff
         char character = UARTGetDataByte(UART1);
         if (end < UART_BUFFER_LENGTH)
         {
             newBuffer[end] = character;
             end++;
         }
      }
      
      if (U1STAbits.OERR == 1)//if an overflow occurs, clear out the fifo
      {                         //We already saved the data we need
           U1STAbits.OERR = 0;
      }

      // Toggle LED to indicate UART activity
      mPORTAToggleBits(BIT_7);

    }

  // We don't care about TX interrupt
  if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )
    {
      INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }  
 }
 
 void __ISR(32, IPL3SOFT) IntUart2Handler(void)
 {
       // Is this an RX interrupt?
  if (INTGetFlag(INT_SOURCE_UART_RX(UART2)))
    {
      // Clear the RX interrupt Flag
      INTClearFlag(INT_SOURCE_UART_RX(UART2));
      
      
      // Echo what we just received.
      //PutCharacter(character);
      
      //add received character to new (larger) buffer
      

      while(U2STAbits.URXDA==1) //if the buffer has stuff in it
      {
         //copy the stuff
         char character = UARTGetDataByte(UART2);
         if(gpsEnd < UART_BUFFER_LENGTH)
         {
            gpsBuffer[gpsEnd] = character;
            gpsEnd++;
         }
      }
      
      if (U2STAbits.OERR == 1)//if an overflow occurs, clear out the fifo
      {                         //We already saved the data we need
           U2STAbits.OERR = 0;
      }

      // Toggle LED to indicate UART activity
      mPORTAToggleBits(BIT_7);

    }

  // We don't care about TX interrupt
  if ( INTGetFlag(INT_SOURCE_UART_TX(UART2)) )
    {
      INTClearFlag(INT_SOURCE_UART_TX(UART2));
    }  
 }
