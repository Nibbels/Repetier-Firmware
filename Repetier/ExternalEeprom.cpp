/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "HAL.h"
#include "ExternalEeprom.h"
#include "TwiTransfer.h"

namespace ExternalEeprom {

bool writeByte(int addressI2C, unsigned int addressEEPROM, unsigned char data) {
    TwiTransfer::finishActive();
    HAL::delayMilliseconds(EEPROM_DELAY);
    const uint8_t bytes[] = {byte(addressEEPROM >> 8), byte(addressEEPROM), data};
    TwiTransfer transfer;
    return transfer.write(addressI2C, bytes, sizeof(bytes), HAL::timeInMilliseconds()) && transfer.wait();
}

bool writeWord(int addressI2C, unsigned int addressEEPROM, unsigned short data) {
    return writeByte(addressI2C, addressEEPROM, byte(data >> 8))
        && writeByte(addressI2C, addressEEPROM + 1, byte(data));
}

static bool readByte(int addressI2C, unsigned int addressEEPROM, uint8_t& data) {
    TwiTransfer::finishActive();
    HAL::delayMilliseconds(EEPROM_DELAY);
    const uint8_t bytes[] = {byte(addressEEPROM >> 8), byte(addressEEPROM)};
    TwiTransfer transfer;
    if (!transfer.write(addressI2C, bytes, sizeof(bytes), HAL::timeInMilliseconds()) || !transfer.wait()
        || !transfer.read(addressI2C, 1, HAL::timeInMilliseconds()) || !transfer.wait())
        return false;
    data = transfer.byteAt(0);
    return true;
} // readByte

bool readWord(int addressI2C, unsigned int addressEEPROM, unsigned short& data) {
    uint8_t high, low;
    if (!readByte(addressI2C, addressEEPROM, high) || !readByte(addressI2C, addressEEPROM + 1, low))
        return false;
    data = (uint16_t(high) << 8) | low;
    return true;
} // readWord

unsigned short readWord(int addressI2C, unsigned int addressEEPROM) {
    unsigned short data;
    return readWord(addressI2C, addressEEPROM, data) ? data : 0xffff;
}

} // namespace ExternalEeprom
