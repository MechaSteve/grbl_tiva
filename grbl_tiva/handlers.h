/*
 * 	handlers.h
 * 	This file provides prototypes for all interrupt handlers required for grbl
 *
 * 	interrupt handlers are sorted from high to low priority
 *
 * 	each interrupt handler will determine the source of the interrupt event,
 * 	and then call the required On<event>( args) function elsewhere in the
 * 	application.
 *
 *
 *
 *
 *
 *
 */


#ifndef handlers_h
#define handlers_h

#include "grbl.h"
#include "gcode.h"

// Serial Communications Interrupt Handlers
//
// These interrupts are triggered by one of two events:
// - byte transmission complete
// - byte received
//
// TRIGGER: Byte Transmission Complete
// always clear the interrupt then
// the handler should call OnSerialTxComplete();
//
// TRIGGER: Byte received
// clear the event and then call OnSerialRxByte(uchar data);
void UART0IntHandler(void);

// GPIO Pin Change interrupts
// all GPIO ports trigger the same ISR
//
// TRIGGER: Limit switch changes state
// all limit switch interrupts are cleared
// all enabled limit switches are polled for state and packed into a word
// limitAlarm controls whether this calls any other function
// call OnLimitChange(ulLimitsState);
// value indicates raw pin state without any inverting applied
// unused switches are reported as 0
// bit00:	X Positive
// bit01:	X Negative
// bit02:	Y Positive
// bit03:	Y Negative
// bit04:	Z Positive
// bit05:	Z Negative
//
//
// TRIGGER: Probe switch
// interrupt is cleared
// call OnProbeEvent(void);
// state is raw pin value
//
// TRIGGER: Reset Switch
// interrupt is cleared
// call OnOperatorResetEvent(void);
//
// TRIGGER: Cycle Start Switch
// interrupt is cleared
// call OnOperatorCycleStartEvent(void);
//
// TRIGGER: Feed Hold Switch
// interrupt is cleared
// call OnOperatorFeedHoldEvent(void);
//
// TRIGGER: E-Stop or Safety Door
// interrupt is cleared
// alarm bit is set
// call OnSafetyEvent(void);
//
// ELSE: Interrupts not handled so far
// Clear all interrupts
//
void GPIOIntHandler(void);

// Step Pulse start interrupt
void Timer0IntHandler(void);

// Step Pulse reset interrupt
void Timer1IntHandler(void);

// Spindle Timing counter interrupt
void SpindleTimerHandler(void);




#endif
