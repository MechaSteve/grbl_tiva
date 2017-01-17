/*
 * handlers.c
 *
 *  Created on: Jan 15, 2017
 *      Author: steph
 */

#include "grbl.h"
#include "driverlib/uart.h"

// Serial Communications Interrupt Handlers
//
// These interrupts are triggered by one of two events:
// - byte transmission complete
// - byte received
//
// TRIGGER: Byte Transmission Complete
// always clear the interrupt then
// the handler should call OnSerialTxEmpty();
//
// TRIGGER: Byte received
// clear the event and then call OnSerialRx(uint8_t data);
void UART0IntHandler(void)
{
	unsigned long status = UARTIntStatus(UART0_BASE, true);

	  /////
	  /////Blink the Green LED to let us know we got to here
	  blinkLed();
	  /////
	  /////

	if (status & UART_INT_TX)
	{
		OnSerialTxEmpty();
		UARTIntClear(UART0_BASE, UART_INT_TX);
	}
	if (status & UART_INT_RX)
	{
		if (UARTCharsAvail(UART0_BASE))	OnSerialRx( (uint8_t)( UARTCharGet(UART0_BASE) ) );
		UARTIntClear(UART0_BASE, UART_INT_RX);
	}
}

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
// probeChangeDebounce is set to prevent triggering again
// call OnProbeChange(tboolean state);
// state is raw pin value
//
// TRIGGER: Feed Hold Switch
// interrupt is cleared
// call OnFeedHoldChange(tboolean state);
//
// TRIGGER: E-Stop or Safety Door
// interrupt is cleared
// alarm bit is set
// call OnSafetyChange(tboolean state);
//
void GPIOIntHandler(void)
{
	long limitStatus = (LIMIT_MASK & GPIOPinIntStatus(LIMIT_BASE, true));
	long probeStatus = (PROBE_PIN & GPIOPinIntStatus(PROBE_BASE, true));

	if(limitStatus != 0) //Limit Switch Triggered
	{
		GPIOPinIntClear(LIMIT_BASE, LIMIT_MASK); //We check all pins, so clear all the interrupts
		OnLimitEvent();
	}

	if(InputResetIntRead())
	{
		InputResetIntClear();
		OnOperatorResetEvent();
	}

	if(InputFeedHoldIntRead())
	{
		InputFeedHoldIntClear();
		OnOperatorFeedHoldEvent();
	}

	if(InputCycleStartIntRead())
	{
		InputCycleStartIntClear();
		OnOperatorCycleStartEvent();
	}


}



// Step Pulse start interrupt
void Timer0IntHandler(void)
{
	OnStepStart();

}

// Step Pulse reset interrupt
void Timer1IntHandler(void)
{
	OnStepReset();
}

