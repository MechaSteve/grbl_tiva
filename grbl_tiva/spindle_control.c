/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"


static float rpm_per_hz = 60.0; // Precalulated value to speed up rpm to Hz conversions.
static int hz_x_100_setpoint; //RPM times 100 for setpoint write command
static int hz_x_100_feedback; //RPM times 100 for storing read command
static int spindle_comm_time; //Counter to detect comm timeout failure
static int read_buffer[16];
static int read_buffer_position = 0;
static bool write_started = false;
static bool read_started = false;
static bool readback_started = false;

/////////////////////////////////////////////
//                                         //
//     Spindle Comm State Machine          //
//     0: Reset/Fail/Restart               //
//     1: Setpoint Write                   //
//     2: Setpoint Confirm                 //
//     3: Command Write                    //
//     4: Command Confirm                  //
//     5: Feedback Request                 //
//     6: Feedback Read                    //
//     7: Status Request                   //
//     8: Feedback Read                    //
//                                         //
/////////////////////////////////////////////
static int spindle_comm_step;


void spindle_init()
{

    // Configure Spindle Timer Counter Interrupt (~1ms counter)
    SysCtlPeripheralEnable(SPINDLE_TIMER);
    TimerDisable(SPINDLE_TIMER_BASE, SPINDLE_SUB_TIMER);
    TimerConfigure(SPINDLE_TIMER_BASE, SPINDLE_TIMER_CFG);
    TimerConfigure(SPINDLE_TIMER_BASE, SPINDLE_TIMER_CFG);
    TimerLoadSet(SPINDLE_TIMER_BASE, SPINDLE_SUB_TIMER, SysCtlClockGet() / 1000);
    //TimerIntEnable(SPINDLE_TIMER_BASE, SPINDLE_INT_CFG);
    TimerEnable(SPINDLE_TIMER_BASE, SPINDLE_SUB_TIMER);

	// Configure variable spindle
    spindle_initDSI();
    OutputSpindleEnableInit();
	spindle_stop();
}

//Configure UART 5 as a DSI communications port
void spindle_initDSI()
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SPINDLE_RX_PORT);
    SysCtlPeripheralEnable(SPINDLE_TX_PORT);
    SysCtlPeripheralEnable(SPINDLE_RTS_PORT);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(SPINDLE_RX_CONFIG);
    GPIOPinConfigure(SPINDLE_RX_CONFIG);
    GPIOPinTypeUART(SPINDLE_RX_BASE, SPINDLE_RX_PIN);
    GPIOPinTypeUART(SPINDLE_TX_BASE, SPINDLE_TX_PIN);
    GPIOPinTypeGPIOOutput(SPINDLE_RTS_BASE, SPINDLE_RTS_PIN);

    //
    // Enable UART5
    //
    SysCtlPeripheralEnable(SPINDLE_UART_PORT);

    //
    // Initialize the UART for DSI Connection at 115200 baud
    //
    UARTConfigSetExpClk(SPINDLE_UART_BASE, 16000000, 9600, (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));
    UARTFIFOEnable(SPINDLE_UART_BASE);

    //
    // Enable the UART operation.
    //
    //UARTStdioConfig(1, 115200, 16000000);
    UARTEnable(SPINDLE_UART_BASE);
}

// Spindle free running counter (~1ms)
void spindle_counter_tick()
{
    spindle_comm_time++;
}

// Spindle state machine cyclicly polls feedback and sends set point
void spindle_update()
{
    int comm_status;
    switch (spindle_comm_step)
    {
        case 0: //Idle, Reset, Restart
            write_started = false;
            read_started = false;
            spindle_comm_step = 1;
            break;
        case 1: //Write Setpoint
            comm_status = ModWriteSingle(100, 0x2001, hz_x_100_setpoint);
            if (comm_status != -1)
            {
                spindle_comm_step = 2;
                spindle_comm_time = 0;
            }
            break;
        case 2:
            comm_status = ModReadbackSingle(100);
            if (comm_status != -1)
            {
                spindle_comm_step = 5;
                spindle_comm_time = 0;
            }
            break;
        case 5: // Read Speed Feedback
            comm_status = ModReadSingle(100, 0x2103);
            if (comm_status != -1)
            {
                spindle_comm_step = 6;
                spindle_comm_time = 0;
            }
            break;
        case 6:
            comm_status = ModReadbackSingle(100);
            if (comm_status != -1)
            {
                hz_x_100_feedback = comm_status;
                spindle_comm_step = 0;
                spindle_comm_time = 0;
            }
            break;
        default:
            spindle_comm_step = 0;
            break;
    }
    spindle_comm_time++;
    if (spindle_comm_time > 5000) spindle_comm_step = 0;

}

// Compute the MODBUS RTU CRC
unsigned int ModRTU_CRC(unsigned char buf[], int len)
{
    uint16_t crc = 0xFFFF;
    int pos, i;

    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc

        for (i = 8; i != 0; i--)
        {    // Loop over each bit
            if ((crc & 0x0001) != 0)
            {      // If the LSB is set
                crc >>= 1;                    // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else                            // Else LSB is not set
            {
                crc >>= 1;                    // Just shift right
            }
        }
    }

    // Note, this number has low and high bytes swapped, so swap bytes
    return (crc >> 8) + (crc << 8);
}

// Write a single holding register over DSI
int ModWriteSingle(unsigned char node, uint16_t address, uint16_t value)
{
    unsigned char uc_MessageBuffer[] = {node, 0x06, 0, 0, 0, 0, 0, 0};
    uint16_t crc = 0;
    int i;

    if(!write_started)
    {
        write_started = true;
        //Split address
        uc_MessageBuffer[2] = address >> 8;
        uc_MessageBuffer[3] = address & 0xFF;

        //Split Data
        uc_MessageBuffer[4] = value >> 8;
        uc_MessageBuffer[5] = value & 0xFF;

        //Calculate and split crc
        crc = ModRTU_CRC(uc_MessageBuffer, 6);
        uc_MessageBuffer[6] = crc >> 8;
        uc_MessageBuffer[7] = crc & 0xFF;

        //Assert control of the line
        GPIOPinWrite(SPINDLE_RTS_BASE, SPINDLE_RTS_PIN, SPINDLE_RTS_PIN);

        //Stuff message bytes in fifo
        for(i = 0; i < 8; i++)
        {
            UARTCharPut(UART5_BASE, uc_MessageBuffer[i]);
        }

        //reset readback status
        readback_started = false;
        //Return signal -1 : Not complete
        return -1;
    }
    else
    {
        //after transmission completes, drop RTS
        if(UARTBusy(SPINDLE_UART_BASE))
        {
            //Return signal -1 : Not complete
            return -1;
        }
        else
        {
            GPIOPinWrite(SPINDLE_RTS_BASE, SPINDLE_RTS_PIN, 0);
            //Return signal 1 : Transmission complete
            return 1;
        }
    }
}

// Write a single holding register over DSI
// TODO: update to match write function
int ModReadSingle(unsigned char node, uint16_t address)
{
    unsigned char uc_MessageBuffer[] = {node, 0x03, 0, 0, 0, 1, 0, 0};
    uint16_t crc = 0;
    int i;


    if(!read_started)
    {
        read_started = true;
        //Split address
        uc_MessageBuffer[2] = address >> 8;
        uc_MessageBuffer[3] = address & 0xFF;

        //Split Data (register count)
        uc_MessageBuffer[4] = 0;
        uc_MessageBuffer[5] = 1;

        //Calculate and split crc
        crc = ModRTU_CRC(uc_MessageBuffer, 6);
        uc_MessageBuffer[6] = crc >> 8;
        uc_MessageBuffer[7] = crc & 0xFF;

        //Assert control of the line
        GPIOPinWrite(SPINDLE_RTS_BASE, SPINDLE_RTS_PIN, SPINDLE_RTS_PIN);

        //Stuff message bytes in fifo
        for(i = 0; i < 8; i++)
        {
            UARTCharPut(UART5_BASE, uc_MessageBuffer[i]);
        }

        //reset readback status
        readback_started = false;
        //Return signal -1 : Not complete
        return -1;
    }
    else
    {
        //after transmission completes, drop RTS
        if(UARTBusy(SPINDLE_UART_BASE))
        {
            //Return signal -1 : Not complete
            return -1;
        }
        else
        {
            GPIOPinWrite(SPINDLE_RTS_BASE, SPINDLE_RTS_PIN, 0);
            //Return signal 1 : Transmission complete
            return 1;
        }
    }
}

// Parse single 16bit response
// TODO: , allow for RX over multiple calls and assembly once complete
int ModReadbackSingle(unsigned char node)
{
    //read back the confirmation
    //Response format is Node|Function|Byte Count|Value Hi|Value Lo|CRC hi|CRC lo
    if(!readback_started)
    {
        read_buffer_position = 0;
        read_buffer[0] = 0;
        read_buffer[1] = 0;
        readback_started = true;
    }
    else
    {
        if(UARTCharsAvail(SPINDLE_UART_BASE))
        {
            read_buffer[read_buffer_position] = UARTCharGet(SPINDLE_UART_BASE);
            read_buffer_position++;
            if((read_buffer_position == 1) && (read_buffer[0] != node)) read_buffer_position = 0;
        }

        if((read_buffer[1] == 0x03) && (read_buffer_position >= 7))
        {
            readback_started = false;
            return (read_buffer[4] << 8) + read_buffer[5];
        }

        if((read_buffer[1] == 0x06) && (read_buffer_position >= 8))
        {
            readback_started = false;
            return (read_buffer[4] << 8) + read_buffer[5];
        }
    }
    return -1;
}


int spindle_get_state()
{
    if(OutputSpindleEnableRead())
        {
            if(OutputSpindleDirectionIsCW()) return (SPINDLE_STATE_CW);
            else return (SPINDLE_STATE_CCW);
        }

    return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
	OutputSpindleEnableOff();
}


//TODO: Update these methods for PWM output or FM output
#ifdef VARIABLE_SPINDLE
  // Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
  // and stepper ISR. Keep routine small and efficient.
  void spindle_set_speed(int hz_100_value)
  {
      hz_x_100_setpoint = hz_100_value;
  }

  // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  int spindle_compute_hz_value(float rpm) // PF525 register is 16-bit, unit is 0.01 Hz.
  {
      //Spindle is 2-pole: Hz = (Poles / 2) * RPM / 60
      return (int) ( (rpm * 100.0) / rpm_per_hz );
  }
#endif


// Immediately sets spindle running state with direction and spindle rpm, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
  void spindle_set_state(int state, float rpm)
#else
  void _spindle_set_state(int state)
#endif
{
	if (sys.abort) { return; } // Block during abort.
	if (state == SPINDLE_DISABLE)
	{
		// Halt or set spindle direction and rpm.
#ifdef VARIABLE_SPINDLE
		sys.spindle_speed = 0.0;
#endif //VARIABLE_SPINDLE
		spindle_stop();
	}
	else
	{
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
		if (state == SPINDLE_ENABLE_CW) OutputSpindleDirectionCW();
		else  OutputSpindleDirectionCCW();
#endif // USE_SPINDLE_DIR_AS_ENABLE_PIN

#ifdef VARIABLE_SPINDLE
		// NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
		if (settings.flags & BITFLAG_LASER_MODE) {
		if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
		}
		spindle_set_speed(spindle_compute_hz_value(rpm));
		//spindle enable should still be used with variable speed (particularly with direction changes)
		OutputSpindleEnableOn();
#else // VARIABLE_SPINDLE
		// NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
		// if the spindle speed value is zero, as its ignored anyhow.
		OutputSpindleEnableOn();
#endif // VARIABLE_SPINDLE

	}

	sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
  void spindle_sync(int state, float rpm)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(int state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    _spindle_set_state(state);
  }
#endif
