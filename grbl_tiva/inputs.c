/*
 * inputs.c
 *
 *  Created on: Jan 15, 2017
 *      Author: steph
 */

#include "grbl.h"


///
///  Operator Input pins
///

// Pin Initializations
void InputResetInit(void)
{
	SysCtlPeripheralEnable(OPERATOR_RESET_PORT);
	GPIODirModeSet(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN, GPIO_DIR_MODE_IN);
#ifdef DISABLE_RESET_PIN_PULL_UP
	// Weak Pull-Down on pins, pins 0 state until connected to Vdd
	GPIOPadConfigSet(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	//Convention is to execute reset on release edge (anti-hold down logic)
	GPIOIntTypeSet(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN, GPIO_FALLING_EDGE);
#else
	// Weak Pull-Up on pins, pins are 1 state until connected to Vss
	GPIOPadConfigSet(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	//Convention is to execute reset on release edge (anti-hold down logic)
	GPIOIntTypeSet(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN, GPIO_RISING_EDGE);
#endif
	//GPIOPinIntEnable(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN);
}

void InputFeedHoldInit(void)
{
	SysCtlPeripheralEnable(OPERATOR_FEED_HOLD_PORT);
	GPIODirModeSet(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN, GPIO_DIR_MODE_IN);
#ifdef DISABLE_FEED_HOLD_PIN_PULL_UP
	// Weak Pull-Down on pins, pins 0 state until connected to Vdd
	GPIOPadConfigSet(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOIntTypeSet(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN, GPIO_RISING_EDGE);
#else
	// Weak Pull-Up on pins, pins are 1 state until connected to Vss
	GPIOPadConfigSet(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN, GPIO_FALLING_EDGE);
#endif
	//GPIOPinIntEnable(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN);
}

void InputCycleStartInit(void)
{
    // NOT IMPLEMENTED
    //
    //
}

// Read pin states
bool InputResetRead(void)
{
	return GPIOPinRead(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN) != 0;
}

bool InputFeedHoldRead(void)
{
	return GPIOPinRead(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN) != 0;
}

bool InputCycleStartRead(void)
{
    // NOT IMPLEMENTED
    //
    //return GPIOPinRead(OPERATOR_CYCLE_START_BASE, OPERATOR_CYCLE_START_PIN) != 0;
    return false;
}



// Read pin interrupt states
bool InputResetIntRead(void)
{
	//return (GPIOPinIntStatus(OPERATOR_RESET_BASE, true) & OPERATOR_RESET_PIN) != 0;
    return false;
}

bool InputFeedHoldIntRead(void)
{
	//return (GPIOPinIntStatus(OPERATOR_FEED_HOLD_BASE, true) & OPERATOR_FEED_HOLD_PIN) != 0;
    return false;
}

bool InputCycleStartIntRead(void)
{
    // NOT IMPLEMENTED
    //
	//return (GPIOPinIntStatus(OPERATOR_CYCLE_START_BASE, true) & OPERATOR_CYCLE_START_PIN) != 0;
    return false;
}


// Clear pin interrupt states
void InputResetIntClear(void)
{
	//GPIOPinIntClear(OPERATOR_RESET_BASE, OPERATOR_RESET_PIN);
}

void InputFeedHoldIntClear(void)
{
	//GPIOPinIntClear(OPERATOR_FEED_HOLD_BASE, OPERATOR_FEED_HOLD_PIN);
}

void InputCycleStartIntClear(void)
{
    // NOT IMPLEMENTED
    //
    //
	//GPIOPinIntClear(OPERATOR_CYCLE_START_BASE, OPERATOR_CYCLE_START_PIN);
}







