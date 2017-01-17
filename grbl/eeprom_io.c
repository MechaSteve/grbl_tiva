/*! \file ********************************************************************
*
* Stephen Culpepper
*
* \li File:               eeprom.c
*
* \li Supported devices:  Stellaris LM4F120H5QR
*
* \li Description:        Source rewritten for Stellaris target.
*                         Makes use of DriverLib Functions to Complete read/write
*
*
*                         $Revision: 0.1 $
*                         $Date: Saturday, January 14, 2017 06:19 PM EST $
****************************************************************************/
#include "grbl.h"
#include "driverlib/eeprom.h"

void eeprom_init(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	EEPROMInit();
}


/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \note  The CPU is not halted during EEPROM read, however there is a 1 cycle delay.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
	unsigned long wordOffset = addr >> 2;
	unsigned long byteShift = (addr & 3) * 8;
	unsigned long readData = 0;
	unsigned long count = 4;

	//EEPROMRead(unsigned long *pulData, unsigned long ulAddress, unsigned long ulCount)
	EEPROMRead(&readData, (wordOffset<<2), count);

	return (char)(readData>>byteShift);
}


/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *  The differences between the existing byte and the new value is used
 *  to select the most efficient EEPROM programming mode.
 *
 *  \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 *  \note  When this function returns, the new EEPROM value is not available
 *         until the EEPROM programming time has passed. The EEPE bit in EECR
 *         should be polled to check whether the programming is finished.
 *
 *  \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	unsigned long wordOffset = addr >> 2;
	unsigned long byteShift = (addr & 3) * 8;
	unsigned long writeData = (unsigned long)new_value << byteShift;
	unsigned long old_value = 0;
	unsigned long writeSuccess = 0;
	unsigned long count = 4;

	//read EEPROM Word
	EEPROMRead(&old_value, (wordOffset<<2), count);
	//Compose new word
	writeData |= (old_value & ~(0x000000FF<<byteShift));
	//write back to EEPROM
	writeSuccess = EEPROMProgram(&writeData, (wordOffset<<2), count);
}



// Extensions added as part of Grbl 


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
