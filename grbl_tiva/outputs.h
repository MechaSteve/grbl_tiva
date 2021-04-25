/*
 * outputs.h
 *
 *  Created on: Jan 15, 2017
 *      Author: Stephen Culpepper
 */

#ifndef OUTPUTS_H_
#define OUTPUTS_H_

///
///		Coolant output pins
///

// Flood Coolant
void OutputFloodInit(void);
void OutputFloodOn(void);
void OutputFloodOff(void);
bool OutputFloodRead(void);


#ifdef ENABLE_M7
// Mist Coolant
void OutputMistInit(void);
void OutputMistOn(void);
void OutputMistOff(void);
tBoolean OutputMistRead(void);
#endif

///
/// 	Spindle Output pins
///

// Spindle Enable
void OutputSpindleEnableInit(void);
void OutputSpindleEnableOn(void);
void OutputSpindleEnableOff(void);
bool OutputSpindleEnableRead(void);


#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
// Spindle Direction
void OutputSpindleDirectionInit(void);
void OutputSpindleDirectionCW(void);
void OutputSpindleDirectionCCW(void);
bool OutputSpindleDirectionIsCW(void);
#endif

///
///  Stepper output pins
///

// Stepper Enable and direction initializations
void AxisEnDirInit(void);

// Stepper Enable Set and Clear
void AxisMasterEnable(void);
void AxisMasterDisable(void);

// Stepper Direction Positive and Negative
void AxisDirectionSet(unsigned long ulDirMask);

// Step output initializations
void AxisStepInit(void);

// Write to Step Pin
void AxisStepSet(unsigned long ulStepMask);




#endif /* OUTPUTS_H_ */
