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

// ----------------------------------------------------------------
//  RxProtocol::processPackets
// ----------------------------------------------------------------
void RxProtocol::processPackets() {
    portDISABLE_INTERRUPTS();
    uint16_t safeHead = head;
    portENABLE_INTERRUPTS();

    size_t processed = 0;
    while (tail != safeHead && processed < PROCESS_BUDGET_BYTES) {
        ++processed;
        uint8_t b = buffer[tail];
        tail = (tail + 1) % capacity();

        switch (state) {
        case RxState::WAIT_SOF:
            if (b == 0xAA) {
                timeout.startTimer(MSG_TIMEOUT_US);
                rxCrc = calcCRC(b, 0);
                state = RxState::READ_INST;
            }
            break;
        case RxState::READ_INST:
            rxInst = b;
            rxCrc  = calcCRC(b, rxCrc);
            state  = RxState::READ_SEQ;
            break;
        case RxState::READ_SEQ:
            rxSeq = b;
            rxCrc  = calcCRC(b, rxCrc);
            state  = RxState::READ_LEN;
            break;
        case RxState::READ_LEN:
            rxLen = b;
            // MAX_PAYLOAD_SIZE es 64 (buffer de rxPayload en Core)
            if (rxLen > 64) { state = RxState::WAIT_SOF; break; }
            rxCrc        = calcCRC(b, rxCrc);
            rxPayloadPos = 0;
            state = (rxLen == 0) ? RxState::READ_CRC : RxState::READ_PAYLOAD;
            break;
        case RxState::READ_PAYLOAD:
            rxPayload[rxPayloadPos++] = b;
            rxCrc = calcCRC(b, rxCrc);
            if (rxPayloadPos >= rxLen) state = RxState::READ_CRC;
            break;
        case RxState::READ_CRC:
            if (rxCrc == b) dispatchPacket(rxInst, rxSeq, rxPayload, rxPayloadPos);
            state = RxState::WAIT_SOF;
            break;
        }
    }

    if (state != RxState::WAIT_SOF && timeout.expired())
        state = RxState::WAIT_SOF;
}

// ----------------------------------------------------------------
//  RxProtocol::dispatchPacket
// ----------------------------------------------------------------
void RxProtocol::dispatchPacket(uint8_t inst, uint8_t seq, uint8_t* payload, uint8_t size) {
    if (inst >= HANDLERS) return;
    extern MsgHandler handlers[];
    if (!handlers[inst]) return;

    uint8_t msg[64];
    Writer  w { msg };
    Reader  r { payload, size };
    handlers[inst](w, r);

    if (w.info && tx_fn_ && tx_obj_)
        tx_fn_(tx_obj_, msg, w.offset, inst, seq);
}

// ----------------------------------------------------------------
//  ping
// ----------------------------------------------------------------
void ping(Writer& /*w*/, Reader& /*r*/) {}