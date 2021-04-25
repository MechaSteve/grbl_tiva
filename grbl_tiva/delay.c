/*
 * delay.c
 *
 *  Created on: Jan 16, 2017
 *      Author: steph
 */

#include 	"grbl.h"
#include	"driverlib/sysctl.h"




// Delays variable defined milliseconds. Compiler compatibility fix for _delay_ms(),
// which only accepts constants in future compiler releases.
void delay_ms(uint16_t ms)
{
  while ( ms-- ) { SysCtlDelay(SysCtlClockGet()/3000); }
}


// Delays variable defined microseconds. Compiler compatibility fix for _delay_us(),
// which only accepts constants in future compiler releases. Written to perform more
// efficiently with larger delays, as the counter adds parasitic time in each iteration.
void delay_us(uint32_t us)
{
	long loops_per_us = (SysCtlClockGet()/3000000) ;	//SysCtlDelay function takes 3 cpu cycles per loop
	long loops_per_10us = (SysCtlClockGet()/300000) ;
	long loops_per_100us = (SysCtlClockGet()/30000) ;
	long loops_per_1000us = (SysCtlClockGet()/3000) ;
	while (us) {
		if (us < 10) {
			//_delay_us(1);
			SysCtlDelay(loops_per_us);
			us--;
		} else if (us < 100) {
			//_delay_us(10);
			SysCtlDelay(loops_per_10us);
			us -= 10;
		} else if (us < 1000) {
			//_delay_us(100);
			SysCtlDelay(loops_per_100us);
			us -= 100;
		} else {
			//_delay_ms(1);
			SysCtlDelay(loops_per_1000us);
			us -= 1000;
		}
	}
}





