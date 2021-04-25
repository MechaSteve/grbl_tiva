/*
 * blink.c
 *
 *  Created on: Jan 16, 2017
 *      Author: steph
 */

#include "grbl.h"

void BlinkLed(void)
{
	volatile unsigned long ulLoop;

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    //SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    //ulLoop = SYSCTL_RCGC2_R;

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    //GPIO_PORTF_DIR_R = 0x08;
    //GPIO_PORTF_DEN_R = 0x08;
    //
	// Turn on the LED.
	//
	//GPIO_PORTF_DATA_R |= 0x08;

	//
	// Delay for a bit.
	//
	for(ulLoop = 0; ulLoop < 20000; ulLoop++)
	{
	}

	//
	// Turn off the LED.
	//
	//GPIO_PORTF_DATA_R &= ~(0x08);

	//
	// Delay for a bit.
	//
	for(ulLoop = 0; ulLoop < 2000000; ulLoop++)
	{
	}
}

