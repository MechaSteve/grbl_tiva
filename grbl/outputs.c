/*
 * outputs.c
 *
 *  Created on: Jan 15, 2017
 *      Author: Stephen Culpepper
 */

#include "grbl.h"




///
///		Coolant output pins
///

// Flood Coolant
void OutputFloodInit(void)
{
	SysCtlPeripheralEnable(COOLANT_FLOOD_PORT);
	GPIOPadConfigSet(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN, GPIO_DIR_MODE_OUT);
}
#ifdef INVERT_COOLANT_COOLANT_FLOOD_PIN
void OutputFloodOn(void) { GPIOPinWrite(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN, 0); }
void OutputFloodOff(void) { GPIOPinWrite(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN, COOLANT_FLOOD_PIN); }
tBoolean OutputFloodRead(void) { return GPIOPinRead(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN) == 0; }
#else
void OutputFloodOn(void) { GPIOPinWrite(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN, COOLANT_FLOOD_PIN); }
void OutputFloodOff(void) { GPIOPinWrite(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN, 0); }
tBoolean OutputFloodRead(void) { return GPIOPinRead(COOLANT_FLOOD_BASE, COOLANT_FLOOD_PIN) != 0; }
#endif

// Mist Coolant
#ifdef ENABLE_M7
void OutputMistInit(void)
{
	SysCtlPeripheralEnable(COOLANT_MIST_PORT);
	GPIOPadConfigSet(COOLANT_MIST_BASE, COOLANT_MIST_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(COOLANT_MIST_BASE, COOLANT_MIST_PIN, GPIO_DIR_MODE_OUT);
}
#ifdef INVERT_COOLANT_COOLANT_MIST_PIN
void OutputMistOn(void) { GPIOPinWrite(COOLANT_MIST_BASE, COOLANT_MIST_PIN, 0); }
void OutputMistOff(void) { GPIOPinWrite(COOLANT_MIST_BASE, COOLANT_MIST_PIN, COOLANT_MIST_PIN); }
tBoolean OutputMistRead(void) { return GPIOPinRead(COOLANT_MIST_BASE, COOLANT_MIST_PIN) == 0; }
#else
void OutputMistOn(void) { GPIOPinWrite(COOLANT_MIST_BASE, COOLANT_MIST_PIN, COOLANT_MIST_PIN); }
void OutputMistOff(void) { GPIOPinWrite(COOLANT_MIST_BASE, COOLANT_MIST_PIN, 0);
tBoolean OutputMistRead(void) { return GPIOPinRead(COOLANT_MIST_BASE, COOLANT_MIST_PIN) != 0; }}
#endif //INVERT_COOLANT_COOLANT_MIST_PIN
#endif //ENABLE_M7

///
/// 	Spindle Output pins
///

// Spindle Enable
void OutputSpindleEnableInit(void)
{
	SysCtlPeripheralEnable(SPINDLE_ENABLE_PORT);
	GPIOPadConfigSet(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN, GPIO_DIR_MODE_OUT);
}
#ifdef INVERT_SPINDLE_ENABLE_PIN
void OutputSpindleEnableOn(void) { GPIOPinWrite(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN, 0); }
void OutputSpindleEnableOff(void) { GPIOPinWrite(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN, SPINDLE_ENABLE_PIN); }
tBoolean OutputSpindleEnableRead(void) { return GPIOPinRead(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN) == 0; }
#else
void OutputSpindleEnableOn(void) { GPIOPinWrite(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN, SPINDLE_ENABLE_PIN); }
void OutputSpindleEnableOff(void) { GPIOPinWrite(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN, 0); }
tBoolean OutputSpindleEnableRead(void) { return GPIOPinRead(SPINDLE_ENABLE_BASE, SPINDLE_ENABLE_PIN) != 0; }
#endif


#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
// Spindle Direction
void OutputSpindleDirectionInit(void)
{
	SysCtlPeripheralEnable(SPINDLE_DIRECTION_PORT);
	GPIOPadConfigSet(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN, GPIO_DIR_MODE_OUT);
}
#ifdef INVERT_SPINDLE_DIRECTION_PIN
void OutputSpindleDirectionCW(void) { GPIOPinWrite(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN, 0); }
void OutputSpindleDirectionCCW(void) { GPIOPinWrite(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN, SPINDLE_DIRECTION_PIN); }
tBoolean OutputSpindleDirectionIsCW(void) { return GPIOPinRead(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN) == 0; }
#else
void OutputSpindleDirectionCW(void) { GPIOPinWrite(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN, SPINDLE_DIRECTION_PIN); }
void OutputSpindleDirectionCCW(void) { GPIOPinWrite(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN, 0); }
tBoolean OutputSpindleDirectionIsCW(void) { return GPIOPinRead(SPINDLE_DIRECTION_BASE, SPINDLE_DIRECTION_PIN) != 0; }
#endif //INVERT_SPINDLE_DIRECTION_PIN
#endif //USE_SPINDLE_DIR_AS_ENABLE_PIN

///
///  Stepper output pins
///

// Stepper Enable and direction initializations
void AxisEnDirInit(void)
{
	//set pin direction and drive 2ma, push-pull

	//Master Enable
	SysCtlPeripheralEnable(STEPPERS_ENABLE_PORT);
	GPIOPadConfigSet(STEPPERS_ENABLE_BASE, STEPPERS_ENABLE_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(STEPPERS_ENABLE_BASE, STEPPERS_ENABLE_PIN, GPIO_DIR_MODE_OUT);

	//X Direction
	SysCtlPeripheralEnable(X_DIRECTION_PORT);
	GPIOPadConfigSet(X_DIRECTION_BASE, X_DIRECTION_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(X_DIRECTION_BASE, X_DIRECTION_PIN, GPIO_DIR_MODE_OUT);

	//Y Direction
	SysCtlPeripheralEnable(Y_DIRECTION_PORT);
	GPIOPadConfigSet(Y_DIRECTION_BASE, Y_DIRECTION_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(Y_DIRECTION_BASE, Y_DIRECTION_PIN, GPIO_DIR_MODE_OUT);

	//Z Direction
	SysCtlPeripheralEnable(Z_DIRECTION_PORT);
	GPIOPadConfigSet(Z_DIRECTION_BASE, Z_DIRECTION_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(Z_DIRECTION_BASE, Z_DIRECTION_PIN, GPIO_DIR_MODE_OUT);
}

// Stepper Enable
void AxisMasterEnable(void)
{
	//invert if needed
	if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) GPIOPinWrite(STEPPERS_ENABLE_BASE, STEPPERS_ENABLE_PIN, 0);
	else GPIOPinWrite(STEPPERS_ENABLE_BASE, STEPPERS_ENABLE_PIN, STEPPERS_ENABLE_PIN);
}

// Stepper Disable
void AxisMasterDisable(void)
{
	//invert if needed
	if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) GPIOPinWrite(STEPPERS_ENABLE_BASE, STEPPERS_ENABLE_PIN, STEPPERS_ENABLE_PIN);
	else GPIOPinWrite(STEPPERS_ENABLE_BASE, STEPPERS_ENABLE_PIN, 0);
}

// Stepper Direction Positive and Negative
void AxisDirectionSet(unsigned long ulDirMask)
{
	// X Axis Direction
	if(ulDirMask & X_AXIS_MASK) GPIOPinWrite(X_DIRECTION_BASE, X_DIRECTION_PIN, X_DIRECTION_PIN);
	else GPIOPinWrite(X_DIRECTION_BASE, X_DIRECTION_PIN, 0);

	// Y Axis Direction
	if(ulDirMask & Y_AXIS_MASK) GPIOPinWrite(Y_DIRECTION_BASE, Y_DIRECTION_PIN, Y_DIRECTION_PIN);
	else GPIOPinWrite(Y_DIRECTION_BASE, Y_DIRECTION_PIN, 0);

	// Z Axis Direction
	if(ulDirMask & Z_AXIS_MASK) GPIOPinWrite(Z_DIRECTION_BASE, Z_DIRECTION_PIN, Z_DIRECTION_PIN);
	else GPIOPinWrite(Z_DIRECTION_BASE, Z_DIRECTION_PIN, 0);
}

// Step output initializations
void AxisStepInit(void)
{

	//X Step
	SysCtlPeripheralEnable(X_STEP_PORT);
	GPIOPinTypeGPIOOutput(X_STEP_BASE, X_STEP_PIN);
	//GPIOPadConfigSet(X_STEP_BASE, X_STEP_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD_WPU);
	//GPIODirModeSet(X_STEP_BASE, X_STEP_PIN, GPIO_DIR_MODE_OUT);

	//Y Step
	SysCtlPeripheralEnable(Y_STEP_PORT);
	GPIOPadConfigSet(Y_STEP_BASE, Y_STEP_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD_WPU);
	GPIODirModeSet(Y_STEP_BASE, Y_STEP_PIN, GPIO_DIR_MODE_OUT);

	//Z Step
	SysCtlPeripheralEnable(Z_STEP_PORT);
	GPIOPadConfigSet(Z_STEP_BASE, Z_STEP_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD_WPU);
	GPIODirModeSet(Z_STEP_BASE, Z_STEP_PIN, GPIO_DIR_MODE_OUT);

}

// Write to Step Pin
void AxisStepSet(unsigned long ulStepMask)
{


	// X Axis Step
	if(ulStepMask & X_AXIS_MASK) GPIOPinWrite(X_STEP_BASE, X_STEP_PIN, X_STEP_PIN);
	else GPIOPinWrite(X_STEP_BASE, X_STEP_PIN, 0);

	// Y Axis Step
	if(ulStepMask & Y_AXIS_MASK) GPIOPinWrite(Y_STEP_BASE, Y_STEP_PIN, Y_STEP_PIN);
	else GPIOPinWrite(Y_STEP_BASE, Y_STEP_PIN, 0);

	// Z Axis Step
	if(ulStepMask & Z_AXIS_MASK) GPIOPinWrite(Z_STEP_BASE, Z_STEP_PIN, Z_STEP_PIN);
	else GPIOPinWrite(Z_STEP_BASE, Z_STEP_PIN, 0);
}
