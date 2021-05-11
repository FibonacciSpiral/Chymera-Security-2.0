/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#pragma once
#include <xc.h>
#include <stdint.h>
#include <sys/attribs.h>
// Remove plib warnings
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _SUPPRESS_PLIB_WARNING
#include <plib.h>
#include "CodeDataBase.h"

//prototypes
void initInputCapture();
extern uint32_t captureTime;
extern uint32_t firstEdge;
extern uint32_t lastEdge;
extern uint32_t period;
extern float pulse_width;
extern int flag;
extern volatile int codeCount;
extern volatile int arm_flag;
extern volatile int data_on;
extern int enteringDataMode;

/* *****************************************************************************
 End of File
 */
