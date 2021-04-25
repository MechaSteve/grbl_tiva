/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "grbl.h"
#include "driverlib/uart.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

//This array must remain as an 8 bit data type to store character data (maybe change to char?)
uint8_t serial_rx_buffer[RX_RING_BUFFER];
//TODO: Head and tail offsets can be native 32bit words
int serial_rx_buffer_head = 0;
volatile int serial_rx_buffer_tail = 0;

//This array must remain as an 8 bit data type to store character data (maybe change to char?)
uint8_t serial_tx_buffer[TX_RING_BUFFER];
//TODO: Head and tail offsets can be native 32bit words
int serial_tx_buffer_head = 0;
volatile int serial_tx_buffer_tail = 0;

bool serial_tx_running = false;


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}


void serial_init()
{

	//
	// Enable the UART peripheral.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Configure the UART for 115,200, 8-N-1 operation.
	// Do not use FIFO, maintain atmega functionality
	//
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTFIFODisable(UART0_BASE);

	//Test

	UARTCharPut(UART0_BASE, 'H');
	while(UARTBusy(UART0_BASE)) {;}
	UARTCharPut(UART0_BASE, 'i');
	while(UARTBusy(UART0_BASE)) {;}
	UARTCharPut(UART0_BASE, '\n');
	while(UARTBusy(UART0_BASE)) {;}

	//
	// Enable the UART interrupt.
	//
	IntDisable(INT_UART0);
	//UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX);
	UARTIntClear(UART0_BASE, UART_INT_TX);
	IntEnable(INT_UART0);

}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data)
{
	// If the buffer is empty, the UART FIFO has space for data and there is no pending interrupt
	// we must restart the tx.
	bool restartReq = (serial_tx_buffer_head == serial_tx_buffer_tail)
			&& UARTSpaceAvail(UART0_BASE) && (UARTIntStatus(UART0_BASE, UART_INT_TX) == 0);

	// Calculate next head
	uint8_t next_head = serial_tx_buffer_head + 1;
	if (next_head == TX_RING_BUFFER) { next_head = 0; }

	// Wait until there is space in the buffer
	while (next_head == serial_tx_buffer_tail)
	{
		// TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
		if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
	}

	// Store data and advance head
	serial_tx_buffer[serial_tx_buffer_head] = data;
	serial_tx_buffer_head = next_head;

	// if the if the buffer was empty and the UART was too, it is the interrupt will not fire
	//and we must force it to start. Set SW Interrupt bit.
	//restartReq = restartReq	|| (UARTSpaceAvail(UART0_BASE) && (UARTIntStatus(UART0_BASE, UART_INT_TX) == 0));
	//if(restartReq) IntPendSet(INT_UART0);

	//If we always force a sw interrupt we will at least check for space in the UART
	IntPendSet(INT_UART0);
}


// Data Register Empty Interrupt (Event) handler
void OnSerialTxEmpty(void)
{

	uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
	//pull the data from the buffer
	uint8_t data_to_send = serial_tx_buffer[tail];

	//UART char put is blocking.
	//Check that there is space before trying to write to UART
	//if not exit interrupt (interrupt will not clear) and allow any other interrupts to run while we wait
	if(UARTSpaceAvail(UART0_BASE))
	{
		// If the buffer is empty, clear the interrupt and do nothing
		if (tail == serial_tx_buffer_head)
		{
			//manually clear the interrupt without writing to the FIFO
			//will have to manually restart transmission
			UARTIntClear(UART0_BASE, UART_INT_TX);
		}
		else
		{
			// Update tail position
			tail++;
			if (tail == TX_RING_BUFFER) { tail = 0; }
			serial_tx_buffer_tail = tail;
			// Send a byte from the buffer (this also clears the interrupt)
			// WARNING! this function will block if there is not space available!
			UARTCharPut(UART0_BASE, data_to_send);
		}
	}
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data;
  }
}

// Data available interrupt (event) handler
void OnSerialRx(uint8_t data)
{
  uint8_t next_head;

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}


void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}
