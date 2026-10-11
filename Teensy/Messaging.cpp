#include "Arduino.h"
#include "Messaging.hpp"

// ----------------------------------------------------------------
//  CRC-8-ATM
// ----------------------------------------------------------------
uint8_t calcCRC(uint8_t data, uint8_t crc) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    return crc;
}
uint8_t calcCRC(const uint8_t* data, size_t len, uint8_t crc) {
    for (size_t i = 0; i < len; i++) crc = calcCRC(data[i], crc);
    return crc;
}

// ----------------------------------------------------------------
//  Reader
// ----------------------------------------------------------------
Reader& Reader::unread(size_t num) {
    if (num > cursor) return *this;
    cursor -= num; return *this;
}
size_t Reader::setCursor(size_t cur) {
    size_t old = cursor; cursor = cur; return old;
}
Reader& Reader::skip(size_t num) {
    if (cursor + num > size) return *this;
    cursor += num; return *this;
}
