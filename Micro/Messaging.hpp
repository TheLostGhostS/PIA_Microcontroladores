#pragma once
// ============================================================
//  Messaging_Core.hpp
//  Protocolo compartido entre I2C y CAN.
//
//  Incluye:
//    - Constantes del protocolo (MAX_PAYLOAD_SIZE, HANDLERS, etc.)
//    - CRC-8-ATM
//    - Writer / Reader
//    - DataRing<SIZE> SPSC lock-free
//    - Deadman
//    - MsgHandler
//    - TxProtocol_Base<Derived> (CRTP) — lógica de loadpackage
//    - RxProtocol — máquina de estados RX completa
//
//  NO incluye nada de Wire, TWAI, SPI ni periféricos.
//  Los archivos Messaging_I2C.hpp y Messaging_CAN.hpp incluyen
//  este header y añaden su TxProtocol concreto + ISRs/tareas.
// ============================================================

#include <Arduino.h>
#include "Timer.hpp"
#include <stdint.h>

// ============================================================
//  Constantes del protocolo
//  MAX_PACKET_SIZE se define en el header específico del transport
//  porque varía (64 en I2C, 8 en CAN 2.0B).
//  El resto es compartido.
// ============================================================
constexpr int HANDLERS       { 24 };
constexpr int WRITE_HANDLERS { 24 };
constexpr int READ_HANDLERS  { 24 };

constexpr unsigned int MSG_TIMEOUT_US  { 1000 };
constexpr size_t PROCESS_BUDGET_BYTES  { 64 };
constexpr int    RX_RING_SIZE          { 256 };

// ============================================================
//  CRC-8-ATM
// ============================================================
uint8_t calcCRC(uint8_t data, uint8_t crc);
uint8_t calcCRC(const uint8_t* data, size_t len, uint8_t crc = 0);

// ============================================================
//  Writer — serializa hacia un buffer de salida
// ============================================================
struct Writer {
    uint8_t* out;
    size_t   offset { 0    };
    bool     info   { true };

    template<typename T> bool write(const T& value);
    Writer& reset();
};

// ============================================================
//  Reader — deserializa desde un buffer de entrada
// ============================================================
struct Reader {
    uint8_t* in     { nullptr };
    size_t   size;
    size_t   cursor { 0      };

    Reader(uint8_t* input, size_t len);

    template<typename T> bool read(T& out);
    template<typename T> T    readSafe(const T& fallback = T{});

    bool    isDone()             const;
    bool    hasSpace(size_t ask) const;
    Reader& unread(size_t num);
    size_t  getCursor()          const;
    size_t  setCursor(size_t cur);
    Reader& skip(size_t num);
};

// ============================================================
//  Deadman — detecta ausencia del master
// ============================================================
class Deadman {
public:
    explicit Deadman(uint32_t timeoutUs, void (*onDead)() = nullptr)
        : timeoutUs_{timeoutUs}, onDead_{onDead} {}

    void kick()  { lastKick_ = micros(); fired_ = false; }

    void check() {
        if (fired_) return;
        volatile uint32_t t = lastKick_;
        if ((micros() - t) >= timeoutUs_) {
            fired_ = true;
            if (onDead_) onDead_();
        }
    }

    bool isAlive()                const { return !fired_; }
    void setCallback(void (*cb)())      { onDead_    = cb; }
    void setTimeout(uint32_t us)        { timeoutUs_ = us; }

private:
    volatile uint32_t lastKick_ { 0     };
    volatile bool     fired_    { false };
    uint32_t          timeoutUs_;
    void            (*onDead_)();
};

// ============================================================
//  DataRing<SIZE> — ring buffer SPSC lock-free
// ============================================================
template<int SIZE>
class DataRing {
public:
    bool     push(uint8_t v);
    bool     pop(uint8_t& v);
    uint16_t pending() const;
    bool     empty()   const;
    void     flush();
    uint16_t skip(uint16_t step);
    uint32_t getByteLoss();
    void     resetByteLoss();

    static constexpr uint16_t capacity() { return static_cast<uint16_t>(SIZE); }

protected:
    volatile uint16_t head { 0 };
    volatile uint16_t tail { 0 };
    uint8_t           buffer[SIZE];
    volatile uint32_t lost { 0 };
};

// ============================================================
//  IMPLEMENTACIONES INLINE / TEMPLATE
// ============================================================

template<typename T>
bool Writer::write(const T& value) {
    if (offset + sizeof(T) > static_cast<size_t>(64)) return false;
    memcpy(out + offset, &value, sizeof(T));
    offset += sizeof(T);
    return true;
}
inline Writer& Writer::reset() { offset = 0; return *this; }

inline Reader::Reader(uint8_t* input, size_t len)
    : in{input}, size{len} {}

template<typename T>
bool Reader::read(T& out) {
    if (cursor + sizeof(T) > size) return false;
    memcpy(&out, in + cursor, sizeof(T));
    cursor += sizeof(T);
    return true;
}

template<typename T>
T Reader::readSafe(const T&) { T p; if (!read(p)) return T{}; return p; }
inline bool   Reader::isDone()             const { return size <= cursor; }
inline bool   Reader::hasSpace(size_t ask) const { return (cursor + ask) <= size; }
inline size_t Reader::getCursor()          const { return cursor; }

template<int SIZE>
bool DataRing<SIZE>::push(uint8_t v) {
    uint16_t next = (head + 1) % SIZE;
    if (next == tail) { lost++; return false; }
    buffer[head] = v; head = next; return true;
}
template<int SIZE>
bool DataRing<SIZE>::pop(uint8_t& v) {
    if (tail == head) return false;
    v = buffer[tail]; tail = (tail + 1) % SIZE; return true;
}
template<int SIZE>
uint16_t DataRing<SIZE>::pending() const { return (head + SIZE - tail) % SIZE; }
template<int SIZE>
bool DataRing<SIZE>::empty() const { return head == tail; }
template<int SIZE>
void DataRing<SIZE>::flush() { tail = head; }
template<int SIZE>
uint16_t DataRing<SIZE>::skip(uint16_t step) {
    uint16_t avail = pending();
    step = (step > avail) ? avail : step;
    tail = (tail + step) % SIZE;
    return step;
}
template<int SIZE>
uint32_t DataRing<SIZE>::getByteLoss() { return lost; }
template<int SIZE>
void DataRing<SIZE>::resetByteLoss() { lost = 0; }



