// Portable CRC primitives used by the EEPROM zone store and its host replay.
#ifndef RF_EEPROM_ZONE_INTEGRITY_H
#define RF_EEPROM_ZONE_INTEGRITY_H
#include <stdint.h>

namespace EepromZones {
inline uint16_t crc16Byte(uint16_t crc, uint8_t value) {
    crc ^= uint16_t(value) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
        crc = (crc & 0x8000U) ? uint16_t((crc << 1) ^ 0x1021U) : uint16_t(crc << 1);
    return crc;
}
inline uint32_t crc32Byte(uint32_t crc, uint8_t value) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit)
        crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
    return crc;
}
}
#endif
