#include "Messaging_CAN.hpp"

static uint32_t g_rx_id = 0;
void setRxId(uint32_t id) { g_rx_id = id; }

// ----------------------------------------------------------------
//  canRxTask — recibe frames CAN, reensambla y empuja al RX ring
// ----------------------------------------------------------------
void canRxTask(void* /*param*/) {
    twai_message_t msg;
    constexpr size_t RX_SEG_BUF_SIZE = 64;

    uint8_t rx_seg_buf[RX_SEG_BUF_SIZE];
    size_t  rx_seg_pos  = 0;
    uint8_t rx_seg_seq  = 0;
    bool    rx_in_prog  = false;

    for (;;) {
        if (twai_receive(&msg, portMAX_DELAY) != ESP_OK) continue;
        if (msg.identifier != g_rx_id) continue;

        deadman.kick();

        uint8_t dlc = msg.data_length_code;
        if (dlc < 1) continue;

        uint8_t  header  = msg.data[0];
        uint8_t  type    = seg_frame_type(header);
        uint8_t  seq     = seg_frame_seq(header);
        uint8_t  datalen = dlc - 1;
        uint8_t *fdata   = msg.data + 1;

        switch (type) {
        case SEG_TYPE_SINGLE:
            rx_in_prog = false;
            rx_seg_pos = 0;
            rx_seg_seq = (seq + 1) & 0x3F;
            for (uint8_t i = 0; i < datalen; i++) rx.push(fdata[i]);
            break;

        case SEG_TYPE_START:
            rx_in_prog = true;
            rx_seg_pos = 0;
            rx_seg_seq = (seq + 1) & 0x3F;
            if (datalen <= RX_SEG_BUF_SIZE) {
                memcpy(rx_seg_buf, fdata, datalen);
                rx_seg_pos = datalen;
            } else { rx_in_prog = false; }
            break;

        case SEG_TYPE_CONT:
            if (!rx_in_prog) break;
            if (seq != rx_seg_seq) { rx_in_prog = false; rx_seg_pos = 0; break; }
            rx_seg_seq = (seq + 1) & 0x3F;
            if (rx_seg_pos + datalen <= RX_SEG_BUF_SIZE) {
                memcpy(rx_seg_buf + rx_seg_pos, fdata, datalen);
                rx_seg_pos += datalen;
            } else { rx_in_prog = false; rx_seg_pos = 0; }
            break;

        case SEG_TYPE_END:
            if (!rx_in_prog) break;
            if (seq != rx_seg_seq) { rx_in_prog = false; rx_seg_pos = 0; break; }
            if (rx_seg_pos + datalen <= RX_SEG_BUF_SIZE) {
                memcpy(rx_seg_buf + rx_seg_pos, fdata, datalen);
                rx_seg_pos += datalen;
            } else { rx_in_prog = false; rx_seg_pos = 0; break; }
            for (size_t i = 0; i < rx_seg_pos; i++) rx.push(rx_seg_buf[i]);
            rx_in_prog = false;
            rx_seg_pos = 0;
            break;
        }
    }
}