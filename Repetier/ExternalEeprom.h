#ifndef EXTERNAL_EEPROM_H
#define EXTERNAL_EEPROM_H

namespace ExternalEeprom {

bool writeByte(int device, unsigned int address, unsigned char value);
bool writeWord(int device, unsigned int address, unsigned short value);
bool readWord(int device, unsigned int address, unsigned short& value);
// The convenience overload returns 0xffff when the chip cannot be read.
unsigned short readWord(int device, unsigned int address);

} // namespace ExternalEeprom

#endif
