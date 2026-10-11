#pragma once
// ============================================================
//  Messaging_CAN.hpp — Transport CAN 2.0B segmentado (ESP32/TWAI)
//
//  Incluye Messaging_Core.hpp y añade:
//    - Constantes de segmentación CAN 2.0B
//    - can_make_id helpers
//    - TxProtocol : TxProtocol_Base<TxProtocol>
//        sendBuffer → segmentación en frames de 7 bytes + twai_transmit
//    - canRxTask FreeRTOS con reensamblado
// ============================================================

#include <Arduino.h>
#include "driver/twai.h"
#include "Messaging.hpp"

// ── Constantes CAN-específicas ────────────────────────────────
constexpr int MAX_PACKET_SIZE  { 64  };           
constexpr int MAX_PAYLOAD_SIZE { MAX_PACKET_SIZE - 5 };  // 5 bytes overhead

// ── Segmentación CAN 2.0B ─────────────────────────────────────
//  byte 0 de cada frame: [tipo(2b) | seq(6b)]
//  bytes 1-7: hasta 7 bytes de datos útiles
constexpr uint8_t SEG_HEADER_SIZE    = 1;
constexpr uint8_t CAN_MAX_DLC        = 8;
constexpr uint8_t SEG_FRAME_DATA_LEN = CAN_MAX_DLC - SEG_HEADER_SIZE;  // 7

constexpr uint8_t SEG_TYPE_SINGLE = 0b00;
constexpr uint8_t SEG_TYPE_START  = 0b01;
constexpr uint8_t SEG_TYPE_CONT   = 0b10;
constexpr uint8_t SEG_TYPE_END    = 0b11;

inline uint8_t seg_make_header(uint8_t type, uint8_t seq) {
    return static_cast<uint8_t>((type & 0x03) << 6) | (seq & 0x3F);
}
inline uint8_t seg_frame_type(uint8_t h) { return (h >> 6) & 0x03; }
inline uint8_t seg_frame_seq (uint8_t h) { return  h       & 0x3F; }

// ── CAN ID encoding (29-bit extended) ────────────────────────
//  [28..26] priority (3b) [25..18] sender (8b)
//  [17..10] receiver (8b) [9..0]   channel (10b)
inline constexpr uint32_t can_make_id(uint8_t sender, uint8_t receiver,
                                       uint16_t channel  = 0,
                                       uint8_t  priority = 6) {
    return ((uint32_t)(priority & 0x07) << 26) |
           ((uint32_t)(sender           ) << 18) |
           ((uint32_t)(receiver          ) << 10) |
           ((uint32_t)(channel  & 0x3FF) );
}

// ── TxProtocol CAN ────────────────────────────────────────────
class TxProtocol : public TxProtocol_Base<TxProtocol> {
public:
    // CRTP hook
    int getMaxPayload() const { return MAX_PAYLOAD_SIZE; }

    // sendBuffer: segmenta el frame ensamblado por Core y lo transmite
    bool sendBuffer(const uint8_t* buf, size_t len) {
        uint8_t frame_buf[CAN_MAX_DLC];
        size_t  offset = 0;
        bool    first  = true;

        while (offset < len) {
            size_t  chunk = ((len - offset) < SEG_FRAME_DATA_LEN)
                            ? (len - offset) : SEG_FRAME_DATA_LEN;
            bool    last  = (offset + chunk >= len);
            uint8_t type  = (first && last) ? SEG_TYPE_SINGLE
                          : first           ? SEG_TYPE_START
                          : last            ? SEG_TYPE_END
                                            : SEG_TYPE_CONT;

            frame_buf[0] = seg_make_header(type, tx_seq_);
            tx_seq_      = (tx_seq_ + 1) & 0x3F;
            memcpy(frame_buf + 1, buf + offset, chunk);

            if (!sendFrame(frame_buf, static_cast<uint8_t>(1 + chunk)))
                return false;

            offset += chunk;
            first   = false;
        }
        return true;
    }

    void     setTxId(uint32_t id) { tx_id_ = id; }
    uint32_t getTxId()      const { return tx_id_; }

private:
    bool sendFrame(const uint8_t* data, uint8_t len) {
        if (len > CAN_MAX_DLC) return false;
        twai_message_t msg = {};
        msg.identifier       = tx_id_;
        msg.data_length_code = len;
        msg.extd             = 1;
        memcpy(msg.data, data, len);
        return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
    }

    uint32_t tx_id_  { 0 };
    uint8_t  tx_seq_ { 0 };
};

// ── Tarea FreeRTOS de recepción CAN ──────────────────────────
void setRxId(uint32_t id);
void canRxTask(void* param);

// ── Globales (definidas en el .ino) ──────────────────────────
extern TxProtocol tx;
extern RxProtocol rx;