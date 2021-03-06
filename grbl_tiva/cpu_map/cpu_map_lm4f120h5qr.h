/*
  cpu_map_atmega328p.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Grbl officially supports the Arduino Uno, but the other supplied pin mappings are
   supplied by users, so your results may vary. This cpu_map file serves as a central
   pin mapping settings file for AVR 328p used on the Arduino Uno.  */
   
#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif


#define GRBL_PLATFORM "lm4f120h5qr"
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"

// IO Pins used on the CNC Breakout board
//
// PC4 - X Axis Pulse (WGPT0A)
// PC5 - Y Axis Pulse (WGPT0B)
// PC6 - Z Axis Pulse (WGPT1A)
// PC7 - A Axis Pulse (WGPT1B)
// PE0 - X Axis Direction
// PF0 - Y Axis Direction
// PA4 - Z Axis Direction
// PA3 - A Axis Direction
// PB2 - Master Enable
// PD0 - X Axis Limit Switches
// PD1 - Y Axis Limit Switches
// PD2 - Z Axis Limit Switches
// PD3 - A Axis Limit Switches
// PE2 - Emergency Stop 1 / Feed Hold / Safety Door
// PE3 - Emergency Stop 2 / Probe
// ( PB5 - Spindle PulseOut !not on CNC Breakout!)
// PE4 - Spindle Rx/ EN
// PE5 - Spindle Tx/ DIR
// PB0 - Tool Rx/ FLOOD COOLANT
// PB1 - Tool Tx/ MIST COOLANT
// PF4 - Reset (Left User Button, SW1)
// PF0 - Cycle Start (Right User Button, SW2)


/////////////////////////////////////
//Based On OSH Park Order for EK-TM4C123GXL
////////////////////////////////////
//J1
//1: 3v3
//2: PB5 :X-Axis Limit 1   (Future: Probe Input)
//3: PB0 :U1Tx: Bluetooth Serial TX
//4: PB1 :U1Rx: Bluetooth Serial Rx
//5: PE4 :U5Tx: Spindle Tx
//6: PE5 :U5Rx: Spindle Rx
//7: PB4 : NC   (Future: Spindle RTS)
//8: PA5 : NC   (Future: Spindle Run Output)
//9: PA6 : NC   (Future: Spindle Reset Output)
//10:PA7 : NC

//J2
//1: GND
//2: PB2: NC : (Future Move X Direction to here)
//3: PE0: X Direction : (Future Move Y Direction to here)
//4: PF0: Y Direction : User SW2 (right) : (Future No External Connection)
//5: HW Reset: NC
//6: PB7: NC : Connected to PD1 (J3.04)
//7: PB6: NC : Connected to PD0 (J3.03)
//8: PA4: Z Direction
//9: PA3: A Direction
//10:PA2: NC

//J3
//1: Vbus : 5V for stepper control
//2: GND
//3: PD0 : X-Axis Limit 2 : Connected to PB6 (J2.07) : (Future Shift all down 1, this becomes X-Axis Limit 1)
//4: PD1 : Y-Axis Limit 1 : Connected to PB7 (J2.06)
//5: PD2 : Y-Axis Limit 2
//6: PD3 : Z-Axis Limit 1
//7: PE1 : Z-Axis Limit 2
//8: PE2 : A-Axis Limit 1 / Home/0deg
//9: PE3 : A-Axis Limit 2 / Probe
//10:PF1 : NC

//J4
//1: PF2: Blue LED
//2: PF3: Green LED
//3: PB3: Master Enable (to all steppers and to safety header) (Future: only to steppers)
//4: PC4: WT0CCP0 : X-Axis Step
//5: PC5: WT0CCP1 : Y-Axis Step
//6: PC6: WT1CCP0 : Z-Axis Step
//7: PC7: WT1CCP1 : A-Axis Step
//8: PD6: Enable Return From Safety Port (Future: return of Safety Out + 1k ohm)
//9: PD7: NC : Future Enable Out (Pulse Testing)
//10:PF4: User SW1 (left)


// Define serial port pins and interrupt vectors.
//#define SERIAL_RX     USART_RX_vect
//#define SERIAL_UDRE   USART_UDRE_vect
//<!--  These are just here as a reminder  --!>
extern void SerialRxIntHandler(void);
extern void SerialTxIntHandler(void);

// Define step pulse output pins.
#define X_STEP_PORT 	SYSCTL_PERIPH_GPIOC
#define X_STEP_BASE     GPIO_PORTC_BASE
#define X_STEP_PIN     	GPIO_PIN_4
#define Y_STEP_PORT 	SYSCTL_PERIPH_GPIOC
#define Y_STEP_BASE     GPIO_PORTC_BASE
#define Y_STEP_PIN     	GPIO_PIN_5
#define Z_STEP_PORT 	SYSCTL_PERIPH_GPIOC
#define Z_STEP_BASE     GPIO_PORTC_BASE
#define Z_STEP_PIN     	GPIO_PIN_6

// Define step direction output pins
//<!--  No real good reason all must be on same port  --!>
//<!--  Stepper functions will need to be rewritten to write to each individual direction pin  --!>
#define X_DIRECTION_PORT 	SYSCTL_PERIPH_GPIOE
#define X_DIRECTION_BASE	GPIO_PORTE_BASE
#define X_DIRECTION_PIN		GPIO_PIN_0

#define Y_DIRECTION_PORT 	SYSCTL_PERIPH_GPIOF
#define Y_DIRECTION_BASE	GPIO_PORTF_BASE
#define Y_DIRECTION_PIN		GPIO_PIN_0

#define Z_DIRECTION_PORT 	SYSCTL_PERIPH_GPIOA
#define Z_DIRECTION_BASE	GPIO_PORTA_BASE
#define Z_DIRECTION_PIN		GPIO_PIN_4

// Define stepper driver enable/disable output pin.
//<!--  CNC Crawler is designed for enable pin (sinking disable)  --!>
#define STEPPERS_ENABLE_PORT 	SYSCTL_PERIPH_GPIOD
#define STEPPERS_ENABLE_BASE	GPIO_PORTD_BASE
#define STEPPERS_ENABLE_PIN		GPIO_PIN_6
// Needed for Axis Lock Function
#define X_STEP_MASK		GPIO_PIN_0
#define Y_STEP_MASK		GPIO_PIN_1
#define Z_STEP_MASK		GPIO_PIN_2
#define STEP_MASK		(X_STEP_MASK|Y_STEP_MASK|Z_STEP_MASK)

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
//#define LIMIT_DDR        DDRB
//#define LIMIT_PIN        PINB
//#define LIMIT_PORT       PORTB
//#define X_LIMIT_BIT      1  // Uno Digital Pin 9
//#define Y_LIMIT_BIT      2  // Uno Digital Pin 10
//#ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.
//  #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
//#else
//  #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
//#endif

//#define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
//#define LIMIT_INT_vect   PCINT0_vect
//#define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register
#define LIMIT_PORT 		SYSCTL_PERIPH_GPIOD
#define LIMIT_BASE		GPIO_PORTD_BASE
#define LIMIT_DIR		GPIO_PORTD_DIR_R
#define LIMIT_DEN		GPIO_PORTD_DEN_R
#define LIMIT_DATA		GPIO_PORTD_DATA_R
#define X_LIMIT_BIT      0  // PD0
#define Y_LIMIT_BIT      1  // PD1
#define Z_LIMIT_BIT      2  // PD2
#define X_LIMIT_PIN		GPIO_PIN_0
#define Y_LIMIT_PIN		GPIO_PIN_1
#define Z_LIMIT_PIN		GPIO_PIN_2
#define LIMIT_MASK      (X_LIMIT_PIN | Y_LIMIT_PIN | Z_LIMIT_PIN) // All limit bits
#define LIMIT_INT		INT_GPIOD

// Define spindle enable and spindle direction output pins.
//#define SPINDLE_ENABLE_DDR    DDRB
//#define SPINDLE_ENABLE_PORT   PORTB
// Z Limit pin and spindle PWM/enable pin swapped to access hardware PWM on Pin 11.
//#ifdef VARIABLE_SPINDLE
//  #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
//    // If enabled, spindle direction pin now used as spindle enable, while PWM remains on D11.
//    #define SPINDLE_ENABLE_BIT    5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
//  #else
//    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
//  #endif
//#else
//  #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
//#endif
//#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
//  #define SPINDLE_DIRECTION_DDR   DDRB
//  #define SPINDLE_DIRECTION_PORT  PORTB
//  #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
//#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT 	SYSCTL_PERIPH_GPIOE
#define SPINDLE_ENABLE_BASE		GPIO_PORTE_BASE
#define SPINDLE_ENABLE_PIN		GPIO_PIN_4

#define SPINDLE_DIRECTION_PORT 		SYSCTL_PERIPH_GPIOE
#define SPINDLE_DIRECTION_BASE		GPIO_PORTE_BASE
#define SPINDLE_DIRECTION_PIN		GPIO_PIN_5
  
// Define flood and mist coolant enable output pins.
// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
//#define COOLANT_FLOOD_DDR   DDRC
//#define COOLANT_FLOOD_PORT  PORTC
//#define COOLANT_FLOOD_BIT   3  // Uno Analog Pin 3
//#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
//  #define COOLANT_MIST_DDR   DDRC
//  #define COOLANT_MIST_PORT  PORTC
//  #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
//#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT 		SYSCTL_PERIPH_GPIOB
#define COOLANT_FLOOD_BASE		GPIO_PORTB_BASE
#define COOLANT_FLOOD_PIN		GPIO_PIN_0

#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
#define COOLANT_MIST_PORT 		SYSCTL_PERIPH_GPIOB
#define COOLANT_MIST_BASE		GPIO_PORTB_BASE
#define COOLANT_MIST_PIN		GPIO_PIN_1
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
//#define CONTROL_DDR       DDRC
//#define CONTROL_PIN       PINC
//#define CONTROL_PORT      PORTC
//#define RESET_BIT         0  // Uno Analog Pin 0
//#define FEED_HOLD_BIT     1  // Uno Analog Pin 1
//#define CYCLE_START_BIT   2  // Uno Analog Pin 2
//#define SAFETY_DOOR_BIT   1  // Uno Analog Pin 1 NOTE: Safety door is shared with feed hold. Enabled by config define.
//#define CONTROL_INT       PCIE1  // Pin change interrupt enable pin
//#define CONTROL_INT_vect  PCINT1_vect
//#define CONTROL_PCMSK     PCMSK1 // Pin change interrupt register
//#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
//#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.

// Define user-control controls (cycle start, reset, feed hold) input pins.
// Will need to rewrite usage of port wide masks for inverting and interrupts
#define OPERATOR_RESET_PORT 	SYSCTL_PERIPH_GPIOF
#define OPERATOR_RESET_BASE		GPIO_PORTF_BASE
#define OPERATOR_RESET_PIN		GPIO_PIN_4
#define OPERATOR_RESET_INT		INT_GPIOF

#define OPERATOR_CYCLE_START_PORT 	SYSCTL_PERIPH_GPIOF
#define OPERATOR_CYCLE_START_BASE	GPIO_PORTF_BASE
#define OPERATOR_CYCLE_START_PIN	GPIO_PIN_0
#define OPERATOR_CYCLE_START_INT	INT_GPIOF

#define OPERATOR_FEED_HOLD_PORT 	SYSCTL_PERIPH_GPIOE
#define OPERATOR_FEED_HOLD_BASE		GPIO_PORTE_BASE
#define OPERATOR_FEED_HOLD_PIN		GPIO_PIN_2
#define OPERATOR_FEED_HOLD_INT		INT_GPIOE

#define SAFETY_DOOR_PORT 	SYSCTL_PERIPH_GPIOE
#define SAFETY_DOOR_BASE	GPIO_PORTE_BASE
#define SAFETY_DOOR_PIN		GPIO_PIN_2
#define SAFETY_DOOR_INT		INT_GPIOE
  
// Define probe switch input pin.
//#define PROBE_DDR       DDRC
//#define PROBE_PIN       PINC
//#define PROBE_PORT      PORTC
//#define PROBE_BIT       5  // Uno Analog Pin 5
//#define PROBE_MASK      (1<<PROBE_BIT)
#define PROBE_PORT 	SYSCTL_PERIPH_GPIOE
#define PROBE_BASE	GPIO_PORTE_BASE
#define PROBE_PIN	GPIO_PIN_3
#define PROBE_INT	INT_GPIOE


//<!--  Variable Spindle will be implemented as separate MCU via UART5 TX/RX  --!>
// Start of PWM & Stepper Enabled Spindle
//#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
//  #define PWM_MAX_VALUE    255.0
//  #define TCCRA_REGISTER	 TCCR2A
//  #define TCCRB_REGISTER	 TCCR2B
//  #define OCR_REGISTER     OCR2A
//
//  #define COMB_BIT	     COM2A1
//  #define WAVE0_REGISTER	 WGM20
//  #define WAVE1_REGISTER	 WGM21
//  #define WAVE2_REGISTER	 WGM22
//  #define WAVE3_REGISTER	 WGM23
      
  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
//  #define SPINDLE_PWM_DDR	  DDRB
//  #define SPINDLE_PWM_PORT  PORTB
//  #define SPINDLE_PWM_BIT	  3    // Uno Digital Pin 11
//#endif // End of VARIABLE_SPINDLE
