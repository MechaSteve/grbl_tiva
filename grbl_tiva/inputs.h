/*
 * inputs.h
 *
 *  Created on: Jan 15, 2017
 *      Author: steph
 */

#ifndef INPUTS_H_
#define INPUTS_H_

//
///
///  Operator Input pins
///
#define CONTROL_RESET_BIT

// Pin Initializations
void InputResetInit(void);
void InputFeedHoldInit(void);
void InputCycleStartInit(void);

// Interrupt get and clear used by the interrupt handler
bool InputResetIntRead(void);
bool InputFeedHoldIntRead(void);
bool InputCycleStartIntRead(void);
void InputResetIntClear(void);
void InputFeedHoldIntClear(void);
void InputCycleStartIntClear(void);

// Read pin states
bool InputResetRead(void);
bool InputFeedHoldRead(void);
bool InputCycleStartRead(void);



#endif /* INPUTS_H_ */
