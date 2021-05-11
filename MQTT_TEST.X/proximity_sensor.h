#pragma once
#include <xc.h>
#include <stdint.h>
#include <sys/attribs.h>
// Remove plib warnings
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _SUPPRESS_PLIB_WARNING
#include <plib.h>

void analogCompInit();

extern volatile int panicFlag;
