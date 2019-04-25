/*
  cpu_map_tm4c123gxl.h - CPU and pin mapping configuration file
  Part of Grbl
  Copyright (c) 2019 Stephen J. Culpepper
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
   pin mapping settings file for the Tiva C EK-TM4C123GXL Launchpad.  */
   
#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif


#define GRBL_PLATFORM "tm4c123gxl"
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"


/////////////////////////////////////
//Based On OSH Park Order for EK-TM4C123GXL
////////////////////////////////////
//J1
//1: 3v3
//2: PB5 :Spindle Comm RTS
//3: PB0 : U1Rx : Bluetooth Serial TX
//4: PB1 : U1Tx : Bluetooth Serial Rx
//5: PE4 : U5Rx : Spindle RS-485 Driver Data Out
//6: PE5 : U5Tx : Spindle RS-485 Driver Data In
//7: PB4 : X Axis Positive Overtravel
//8: PA5 : Y Axis Positive Overtravel
//9: PA6 : Z Axis Positive Overtravel
//10:PA7 : A Axis Positive Overtravel / Home

//J2
//1: GND
//2: PB2 : X Axis Direction
//3: PE0 : Y Axis Direction
//4: PF0 : User SW2 (right) : Feed Hold
//5: RST : NC
//6: PB7 : NC : Connected to PD1 (J3.04)
//7: PB6 : NC : Connected to PD0 (J3.03)
//8: PA4 : Z Direction
//9: PA3 : Safety Monitor In
//10:PA2 : NC

//J3
//1: Vbus : 5V for stepper control
//2: GND :
//3: PD0 : NC : Connected to PB6 (J2.07)
//4: PD1 : NC : Connected to PB7 (J2.06)
//5: PD2 : Spindle Command Reset
//6: PD3 : Spindle Command Run
//7: PE1 : X Axis Negative Overtravel
//8: PE2 : Y Axis Negative Overtravel
//9: PE3 : Z Axis Negative Overtravel
//10:PF1 : A Axis Negative Overtravel / Probe

//J4
//1: PF2: Blue LED
//2: PF3: Green LED
//3: PB3: Master Enable (to all steppers)
//4: PC4: WT0CCP0 : X-Axis Step
//5: PC5: WT0CCP1 : Y-Axis Step
//6: PC6: WT1CCP0 : Z-Axis Step
//7: PC7: WT1CCP1 : A-Axis Step
//8: PD6: A Axis Direction
//9: PD7: Safety Monitor Out
//10:PF4: User SW1 (left)


// Define serial port pins and interrupt vectors.
//#define SERIAL_RX     USART_RX_vect
//#define SERIAL_UDRE   USART_UDRE_vect
//<!--  These are just here as a reminder  --!>
extern void SerialRxIntHandler(void);
extern void SerialTxIntHandler(void);

//for future implementation of autonomous steppers
//extern void XAxisIntHandler(void);
//extern void YAxisIntHandler(void);
//extern void ZAxisIntHandler(void);
//extern void AAxisIntHandler(void);

// Define step pulse output pins.
#define X_STEP_PORT 	SYSCTL_PERIPH_GPIOC
#define X_STEP_BASE     GPIO_PORTC_BASE
#define X_STEP_PIN     	GPIO_PIN_4
// for future implementation of autonomous steppers
//#define X_STEP_TIMER    SYSCTL_PERIPH_WTIMER0
//#define X_STEP_TIMER_BASE   WTIMER0_BASE
//#define X_STEP_SUB_TIMER    TIMER_A
//#define X_STEP_INT          INT_WTIMER0A
//#define X_STEP_INT_CFG      TIMER_TIMA_TIMEOUT
//#define Y_STEP_INT      INT_WTIMER0A_TM4C123
//#define X_STEP_TIMER_CFG    TIMER_CFG_A_PWM


#define Y_STEP_PORT 	SYSCTL_PERIPH_GPIOC
#define Y_STEP_BASE     GPIO_PORTC_BASE
#define Y_STEP_PIN     	GPIO_PIN_5
// for future implementation of autonomous steppers
//#define Y_STEP_TIMER    SYSCTL_PERIPH_WTIMER0
//#define Y_STEP_TIMER_BASE   WTIMER0_BASE
//#define Y_STEP_SUB_TIMER    TIMER_B
//#define Y_STEP_INT_CFG      TIMER_TIMB_TIMEOUT
//#define Y_STEP_INT      INT_WTIMER0B_TM4C123
//#define Y_STEP_TIMER_CFG    TIMER_CFG_B_PWM

#define Z_STEP_PORT 	SYSCTL_PERIPH_GPIOC
#define Z_STEP_BASE     GPIO_PORTC_BASE
#define Z_STEP_PIN     	GPIO_PIN_6
// for future implementation of autonomous steppers
//#define Z_STEP_TIMER    SYSCTL_PERIPH_WTIMER1
//#define Z_STEP_TIMER_BASE   WTIMER1_BASE
//#define Z_STEP_SUB_TIMER    TIMER_A
//#define Z_STEP_INT_CFG      TIMER_TIMA_TIMEOUT
//#define Y_STEP_INT      INT_WTIMER1A_TM4C123
//#define Z_STEP_TIMER_CFG    TIMER_CFG_A_PWM

// Define step direction output pins
//<!--  No real good reason all must be on same port  --!>
//<!--  Stepper functions will need to be rewritten to write to each individual direction pin  --!>
#define X_DIRECTION_PORT 	SYSCTL_PERIPH_GPIOB
#define X_DIRECTION_BASE	GPIO_PORTB_BASE
#define X_DIRECTION_PIN		GPIO_PIN_2

#define Y_DIRECTION_PORT 	SYSCTL_PERIPH_GPIOE
#define Y_DIRECTION_BASE	GPIO_PORTE_BASE
#define Y_DIRECTION_PIN		GPIO_PIN_0

#define Z_DIRECTION_PORT 	SYSCTL_PERIPH_GPIOA
#define Z_DIRECTION_BASE	GPIO_PORTA_BASE
#define Z_DIRECTION_PIN		GPIO_PIN_4

// Define stepper driver enable/disable output pin.
//<!--  CNC Crawler is designed for enable pin (sinking disable)  --!>
#define STEPPERS_ENABLE_PORT 	SYSCTL_PERIPH_GPIOB
#define STEPPERS_ENABLE_BASE	GPIO_PORTB_BASE
#define STEPPERS_ENABLE_PIN		GPIO_PIN_3

// Define homing/hard limit switch input pins. 
// Limit switches do not require a separate interrupt, they are polled at every axis step for homing
#define X_POSITIVE_LIMIT_PORT   SYSCTL_PERIPH_GPIOB
#define X_POSITIVE_LIMIT_BASE   GPIO_PORTB_BASE
#define X_POSITIVE_LIMIT_PIN    GPIO_PIN_4

#define X_NEGATIVE_LIMIT_PORT   SYSCTL_PERIPH_GPIOE
#define X_NEGATIVE_LIMIT_BASE   GPIO_PORTE_BASE
#define X_NEGATIVE_LIMIT_PIN    GPIO_PIN_1

#define Y_POSITIVE_LIMIT_PORT   SYSCTL_PERIPH_GPIOA
#define Y_POSITIVE_LIMIT_BASE   GPIO_PORTA_BASE
#define Y_POSITIVE_LIMIT_PIN    GPIO_PIN_5

#define Y_NEGATIVE_LIMIT_PORT   SYSCTL_PERIPH_GPIOE
#define Y_NEGATIVE_LIMIT_BASE   GPIO_PORTE_BASE
#define Y_NEGATIVE_LIMIT_PIN    GPIO_PIN_2

#define Z_POSITIVE_LIMIT_PORT   SYSCTL_PERIPH_GPIOA
#define Z_POSITIVE_LIMIT_BASE   GPIO_PORTA_BASE
#define Z_POSITIVE_LIMIT_PIN    GPIO_PIN_6

#define Z_NEGATIVE_LIMIT_PORT   SYSCTL_PERIPH_GPIOE
#define Z_NEGATIVE_LIMIT_BASE   GPIO_PORTE_BASE
#define Z_NEGATIVE_LIMIT_PIN    GPIO_PIN_3

// Define spindle reset and spindle run output pins.
#define SPINDLE_RESET_PORT 	SYSCTL_PERIPH_GPIOD
#define SPINDLE_RESET_BASE	GPIO_PORTD_BASE
#define SPINDLE_RESET_PIN	GPIO_PIN_2

#define SPINDLE_RUN_PORT 	SYSCTL_PERIPH_GPIOD
#define SPINDLE_RUN_BASE	GPIO_PORTD_BASE
#define SPINDLE_RUN_PIN		GPIO_PIN_3

// Define spindle COMM port and RTS output pin
#define SPINDLE_UART_PORT   SYSCTL_PERIPH_UART5
#define SPINDLE_UART_BASE   UART5_BASE

#define SPINDLE_RTS_PORT 	SYSCTL_PERIPH_GPIOB
#define SPINDLE_RTS_BASE	GPIO_PORTB_BASE
#define SPINDLE_RTS_PIN		GPIO_PIN_5

#define SPINDLE_RX_PORT 	SYSCTL_PERIPH_GPIOE
#define SPINDLE_RX_BASE	    GPIO_PORTE_BASE
#define SPINDLE_RX_PIN		GPIO_PIN_4
#define SPINDLE_RX_CONFIG	GPIO_PE4_U5RX

#define SPINDLE_TX_PORT 	SYSCTL_PERIPH_GPIOE
#define SPINDLE_TX_BASE	    GPIO_PORTE_BASE
#define SPINDLE_TX_PIN		GPIO_PIN_5
#define SPINDLE_TX_PIN		GPIO_PE5_U5TX

// Define user-control controls (cycle start, reset, feed hold) input pins.
// Will need to rewrite usage of port wide masks for inverting and interrupts
#define OPERATOR_RESET_PORT 	SYSCTL_PERIPH_GPIOF
#define OPERATOR_RESET_BASE		GPIO_PORTF_BASE
#define OPERATOR_RESET_PIN		GPIO_PIN_4

#define OPERATOR_FEED_HOLD_PORT 	SYSCTL_PERIPH_GPIOE
#define OPERATOR_FEED_HOLD_BASE		GPIO_PORTE_BASE
#define OPERATOR_FEED_HOLD_PIN		GPIO_PIN_2

#define SAFETY_OUT_PORT 	SYSCTL_PERIPH_GPIOD
#define SAFETY_OUT_BASE     GPIO_PORTD_BASE
#define SAFETY_OUT_PIN		GPIO_PIN_7

#define SAFETY_DOOR_PORT 	SYSCTL_PERIPH_GPIOA
#define SAFETY_DOOR_BASE	GPIO_PORTA_BASE
#define SAFETY_DOOR_PIN		GPIO_PIN_3
  
// Define probe switch input pin.
#define PROBE_PORT 	SYSCTL_PERIPH_GPIOF
#define PROBE_BASE	GPIO_PORTF_BASE
#define PROBE_PIN	GPIO_PIN_1

