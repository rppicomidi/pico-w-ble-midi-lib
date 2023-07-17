/**
 * MIT License
 *
 * Copyright (c) 2023 rppicomidi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/**
 * To the extent this code is solely for use on the Rapsberry Pi Pico W or
 * Pico WH, the license file ${PICO_SDK_PATH}/src/rp2_common/pico_btstack/LICENSE.RP may
 * apply.
 * 
 */

/**
 * This file uses code from various BlueKitchen example files, which contain
 * the following copyright notice, included per the notice below.
 *
 * Copyright (C) 2018 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */
#include "midi_service_stream_handler.h"
#include "ring_buffer_lib.h"
#include "pico/stdlib.h"
#include <assert.h>
#include <stdio.h>
#include "ble/att_server.h"
#include "btstack_debug.h"
#include "btstack_event.h"

/**
 * This structure defines a timestamped fragment of a MIDI message.
 * The upper 5 bits of nbytes encodes what sort of message is contained in msg_bytes[]
 * The lower 2 bits of nbytes encodes the number of valid bytes in msg_bytes[]
 * Only one byte per message is allowed.
 */
typedef struct ble_midi_message_s {
    uint8_t nbytes;         //!< Bit 7 is set if it is necessary to pre-pend a start of system exclusive status byte to the msg_bytes[] data
                            //!< Bit 6 is set if it is necessary to append an end of system exclusive status bytge to the msg_bytes[] data
                            //!< Bit 5 is set if the msg_bytes[] data contains payload bytes of a system exclusive message
                            //!< Bit 4 is set if msg_bytes[] contains a system real-time status byte
                            //!< Bit 3 is set if msg_bytes[] contains a channel status byte and data bytes; running status is not supported
                            //!< Bits 7:3 all clear means msg_bytes[] contains a system common status byte and data bytes.
                            //!< Bit 2 is reserved
                            //!< Bits 1:0 holdthe number of valid bytes in msg_bytes.
    uint8_t msg_bytes[3];   //!< msg_bytes[0:(nbytes & 0x7)-1] contains the timestamped data.
    uint16_t timestamp_ms;  //!< BLE MIDI timestamps are 13 bits; this value is 14 bits in case the timestamp wraps within one packet.
} __attribute__((packed)) ble_midi_message_t;


// This driver makes use of the LE Data Packet Length Extension feature to improve throughput
#ifdef ENABLE_LE_DATA_LENGTH_EXTENSION
#define MAX_BLE_MIDI_PACKET 251-3
#else
#define MAX_BLE_MIDI_PACKET ATT_DEFAULT_MTU-3
#endif

typedef struct ble_midi_packet_s {
    uint16_t nbytes;
    uint8_t pkt[MAX_BLE_MIDI_PACKET];
} __attribute__((packed)) ble_midi_packet_t;

// Use the follow to set an clear flab bits stored in the ble_midi_message_t "nbytes" field
static const uint8_t ble_midi_packet_is_start_sysex = 0x80;
//static const uint8_t ble_midi_packet_is_eox = 0x40; Never used
static const uint8_t ble_midi_packet_is_sysex = 0x20;
static const uint8_t ble_midi_packet_is_real_time = 0x10;
static const uint8_t ble_midi_packet_is_channel = 0x08;
static const uint8_t ble_midi_packet_nbytes_mask = 0x03;

// TODO: does it make sense to support more than one BLE-MIDI connection?
#define MIDI_SERVICE_STREAM_HANDLER_MAX_CONNECTION 1

#define MIDI_SERVICE_HEADER(x) (0x80 | (((x) >> 7) & 0x3F) )
#define MIDI_SERVICE_TIMESTAMP_LOW(x) (0x80 | ((x) & 0x7F))

static uint16_t midi_service_timestamp_decode(uint16_t* msb, uint8_t lsb, uint8_t* prev_lsb)
{
    // time has to either stand still or go forward
    if (*prev_lsb > lsb) {
        *msb += 1;
    }
    *prev_lsb = lsb;
    return ((*msb) | ((lsb) & 0x7F));
}

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_handler_t client_packet_handler;
typedef struct {
    ble_midi_message_t mes;                     // intermediate encoding of message stream into message packets
    uint16_t next_msg_byte_idx;                 // the next status or data byte will be written to mes.msg_bytes[next_msg_byte_idx]
    uint16_t previous_timestamp;                // the timestamp of the last MIDI status byte encoded to be sent in pending_ble_pkt
    uint8_t running_status;                     // the MIDI message stream channel message running status
    uint8_t npending_rt;                        // number of pending real-time messages
    uint8_t pending_rt_status[4];               // buffer of pending real-time messages
    uint16_t pending_rt_timestamp[4];           // buffer of pending real-time message timestamps
    ble_midi_packet_t pending_ble_pkt;          // packet currently being encoded from the message stream
    uint8_t pending_ble_midi_pkt_running_status; // the channel message running status of the pending BLE-MIDI packet
} to_ble_midi_stream_t;

// This structure defines the data context for each connection
typedef struct {
    int le_notification_enabled;
    hci_con_handle_t connection_handle;
    // data packets sent to the Blueooth stack are stored here
    uint8_t to_ble_buffer_storage[sizeof(ble_midi_packet_t) * 10];
    to_ble_midi_stream_t to_ble_midi_stream;
    bool waiting_for_send_notification_request;
    // parsed messages received from the Blueooth stack are stored here
    uint8_t from_ble_buffer_storage[sizeof(ble_midi_message_t) * 100];
    btstack_context_callback_registration_t send_request;
    ring_buffer_t to_ble;
    ring_buffer_t from_ble;
    uint16_t ble_mtu;

    char name[7]; //"MIDI x" where x is A, B, C, D
} midi_service_stream_connection_t;

static midi_service_stream_connection_t midi_service_stream_connection[MIDI_SERVICE_STREAM_HANDLER_MAX_CONNECTION];

static void midi_service_stream_init_ring_buffers(midi_service_stream_connection_t* context)
{
    context->to_ble_midi_stream.next_msg_byte_idx = 0;
    context->to_ble_midi_stream.previous_timestamp = 0xffff; // illegal value
    context->to_ble_midi_stream.running_status = 0;
    context->to_ble_midi_stream.mes.nbytes = 0;
    context->to_ble_midi_stream.mes.timestamp_ms = 0xffff;
    context->to_ble_midi_stream.npending_rt = 0;
    context->to_ble_midi_stream.pending_ble_pkt.nbytes = 0;
    ring_buffer_init(&context->from_ble, (uint8_t *)context->from_ble_buffer_storage, sizeof(context->from_ble_buffer_storage), 0);
    ring_buffer_init(&context->to_ble, (uint8_t *)context->to_ble_buffer_storage, sizeof(context->to_ble_buffer_storage), 0);

}

static uint16_t midi_service_stream_get_system_13_bit_ms_timestamp()
{
    return (uint16_t)((time_us_32()/1000) & 0x1FFF);
}

static void midi_service_stream_add_header_if_needed(ble_midi_packet_t* pkt, uint16_t timestamp)
{
    if (pkt->nbytes == 0) {
        pkt->pkt[pkt->nbytes++] = MIDI_SERVICE_HEADER(timestamp);
    }
}

static void midi_service_stream_flush_rt_midi_to_pkt(midi_service_stream_connection_t* context)
{
    to_ble_midi_stream_t* ble_midi_stream = &context->to_ble_midi_stream;
    for (uint8_t idx = 0; idx < ble_midi_stream->npending_rt; idx++) {
        // make sure there is room in the ATT packet to store the data
        if ((ble_midi_stream->pending_ble_pkt.nbytes + 2) >= context->ble_mtu) {
            ring_buffer_push(&context->to_ble, (uint8_t*)&ble_midi_stream->pending_ble_pkt, sizeof(ble_midi_stream->pending_ble_pkt));
            ble_midi_stream->pending_ble_pkt.nbytes = 0;
            ble_midi_stream->pending_ble_midi_pkt_running_status = 0;
        }
        if (ble_midi_stream->pending_ble_pkt.nbytes == 0) {
            ble_midi_stream->pending_ble_pkt.pkt[ble_midi_stream->pending_ble_pkt.nbytes++] = MIDI_SERVICE_HEADER(ble_midi_stream->pending_rt_timestamp[0]);
        }
        ble_midi_stream->pending_ble_pkt.pkt[ble_midi_stream->pending_ble_pkt.nbytes++] = MIDI_SERVICE_TIMESTAMP_LOW(ble_midi_stream->pending_rt_timestamp[idx]);
        ble_midi_stream->pending_ble_pkt.pkt[ble_midi_stream->pending_ble_pkt.nbytes++] = ble_midi_stream->pending_rt_status[idx];
    }
    ble_midi_stream->npending_rt = 0;
}

static void midi_service_stream_encode_bt_pkt(midi_service_stream_connection_t* context)
{
    to_ble_midi_stream_t* ble_midi_stream = &context->to_ble_midi_stream;
    uint8_t nbytes = ble_midi_stream->mes.nbytes & ble_midi_packet_nbytes_mask;
    if (nbytes == 0)
        return;
    if ((ble_midi_stream->mes.nbytes & ble_midi_packet_is_real_time) != ble_midi_packet_is_real_time) {
        midi_service_stream_flush_rt_midi_to_pkt(context);
    }

    // channel messages for which the midi packet running status match the status byte running status may omit byte 0 of msg_bytes
    bool requires_byte0 = (((ble_midi_stream->mes.nbytes & ble_midi_packet_is_channel) != ble_midi_packet_is_channel) ||
             (ble_midi_stream->pending_ble_midi_pkt_running_status != ble_midi_stream->mes.msg_bytes[0]));

    // Need to start the packet with a timestamp unless it is just sysex data or if channel message running status data with the same
    // timestamp as the previous timestamp.
    bool needs_timestamp = 
     (((ble_midi_stream->mes.nbytes & (ble_midi_packet_is_sysex | ble_midi_packet_is_start_sysex)) != ble_midi_packet_is_sysex) &&
     (requires_byte0 || ble_midi_stream->previous_timestamp != ble_midi_stream->mes.timestamp_ms)) ;

    uint8_t first_byte_idx = (requires_byte0 ? 0:1);
    uint8_t total_bytes = nbytes + (needs_timestamp ? 1:0) - first_byte_idx;
    if ((ble_midi_stream->pending_ble_pkt.nbytes + total_bytes) >= context->ble_mtu) {
        ring_buffer_push(&context->to_ble, (uint8_t*)&ble_midi_stream->pending_ble_pkt, sizeof(ble_midi_stream->pending_ble_pkt));
        ble_midi_stream->pending_ble_pkt.nbytes = 0;
        ble_midi_stream->pending_ble_midi_pkt_running_status = 0;
    }
    else if ((ble_midi_stream->mes.nbytes & ble_midi_packet_is_channel) != 0) {
        ble_midi_stream->pending_ble_midi_pkt_running_status = ble_midi_stream->mes.msg_bytes[0];
    }
    midi_service_stream_add_header_if_needed(&ble_midi_stream->pending_ble_pkt, ble_midi_stream->mes.timestamp_ms);
    if (needs_timestamp) {
        ble_midi_stream->pending_ble_pkt.pkt[ble_midi_stream->pending_ble_pkt.nbytes++] = MIDI_SERVICE_TIMESTAMP_LOW(ble_midi_stream->mes.timestamp_ms);
    }

    for (uint8_t idx = first_byte_idx; idx < nbytes; idx++) {
        ble_midi_stream->pending_ble_pkt.pkt[ble_midi_stream->pending_ble_pkt.nbytes++] = ble_midi_stream->mes.msg_bytes[idx];
    }
}

static void midi_service_stream_encode_full_bt_pkt(midi_service_stream_connection_t* context)
{
    midi_service_stream_encode_bt_pkt(context);
    context->to_ble_midi_stream.next_msg_byte_idx = 0;
    midi_service_stream_flush_rt_midi_to_pkt(context);
}

static void midi_service_stream_request_send(midi_service_stream_connection_t* context)
{
    midi_service_server_request_can_send_now(&context->send_request, context->connection_handle);
}

static void midi_service_stream_discard_stream(uint8_t* midi_stream, uint16_t nbytes, to_ble_midi_stream_t* ble_midi_stream)
{
    printf("MIDI stream error\r\ndiscarding unsent MIDI bytes:\r\n");
    ble_midi_stream->next_msg_byte_idx = 0;
    ble_midi_stream->pending_ble_pkt.nbytes = 0;
    ble_midi_stream->previous_timestamp = 0xffff; // illegal value
    ble_midi_stream->running_status = 0;
    ble_midi_stream->mes.nbytes = 0;
    ble_midi_stream->mes.timestamp_ms = 0xffff;
    ble_midi_stream->npending_rt = 0;
    printf_hexdump(midi_stream, nbytes);
}

uint16_t midi_service_stream_push_midi(uint8_t* midi_stream, uint16_t nbytes, midi_service_stream_connection_t* context)
{
    uint16_t timestamp = midi_service_stream_get_system_13_bit_ms_timestamp();
    uint16_t bytes_pushed = 0;
    to_ble_midi_stream_t* ble_midi_stream = &context->to_ble_midi_stream;
    while (bytes_pushed < nbytes && ble_midi_stream->pending_ble_pkt.nbytes < context->ble_mtu) {
        uint8_t ms_byte = midi_stream[bytes_pushed];
        if (ms_byte & 0x80) {
            // status byte; potential start of message
            if (ms_byte == 0xF0) {
                ble_midi_stream-> previous_timestamp = timestamp;
                // start of system exclusive
                ble_midi_stream->mes.nbytes = ble_midi_packet_is_start_sysex | ble_midi_packet_is_sysex | 1;
                ++bytes_pushed;
                ble_midi_stream->running_status = 0;
                ble_midi_stream->mes.msg_bytes[0] = ms_byte;
                ble_midi_stream->next_msg_byte_idx = 1;
            }
            else if (ms_byte == 0xF7) {
                if ((ble_midi_stream->mes.nbytes & ble_midi_packet_is_sysex) == 0) {
                    // terminated a sysex message without being inside a sysex message.
                    // Something went wrong with the MIDI stream or this design
                    // Discard the pending message and pending real-time messages
                    midi_service_stream_discard_stream(midi_stream, nbytes, ble_midi_stream);

                    return bytes_pushed;
                }
                // end of system exclusive
                if (ble_midi_stream->next_msg_byte_idx != 0) {
                    // encode all the sysex data bytes and the potential start of sysex
                    midi_service_stream_encode_bt_pkt(context);
                }
                // timestamp is the same as the start of sysex or after the last real-time message
                ble_midi_stream->mes.timestamp_ms =  ble_midi_stream->previous_timestamp;
                ble_midi_stream->mes.msg_bytes[0] = ms_byte;
                ble_midi_stream->mes.nbytes = 1;
                ++bytes_pushed;
                midi_service_stream_encode_full_bt_pkt(context);
                ble_midi_stream->running_status = 0;
            }
            else if (ms_byte >= 0xF8) {
                if ((ble_midi_stream->mes.nbytes & ble_midi_packet_is_sysex) != 0 && (ble_midi_stream->mes.nbytes & ble_midi_packet_nbytes_mask) > 0) {
                    // empty out the pending packet; it is OK for real-time to occur anywhere inside a sysex message
                    midi_service_stream_encode_bt_pkt(context);
                    context->to_ble_midi_stream.next_msg_byte_idx = 0;
                    ble_midi_stream->mes.nbytes = ble_midi_packet_is_sysex;
                }
                ble_midi_stream->pending_rt_status[ble_midi_stream->npending_rt] = ms_byte;
                ble_midi_stream->pending_rt_timestamp[ble_midi_stream->npending_rt++] = timestamp;
                ++bytes_pushed;
                if (ble_midi_stream->npending_rt >= sizeof(ble_midi_stream->pending_rt_status)) {
                    // Something went wrong with the MIDI stream or this design
                    // Discard the pending message and pending real-time messages
                    midi_service_stream_discard_stream(midi_stream, nbytes, ble_midi_stream);

                    return bytes_pushed;
                }
                midi_service_stream_flush_rt_midi_to_pkt(context);
            }
            else if (ms_byte > 0xF0) {
                // system common message
                ble_midi_stream->mes.msg_bytes[0] = ms_byte;
                ble_midi_stream->previous_timestamp = timestamp;
                ble_midi_stream->mes.timestamp_ms = timestamp;
                ++bytes_pushed;
                ble_midi_stream->running_status = 0;
                if (ms_byte == 0xF1 || ms_byte == 0xF3) {
                    ble_midi_stream->mes.nbytes = 2;
                    ble_midi_stream->next_msg_byte_idx = 1;
                }
                else if (ms_byte == 0xF2) {
                    ble_midi_stream->mes.nbytes = 3;
                    ble_midi_stream->next_msg_byte_idx = 1;
                }
                else {
                    ble_midi_stream->mes.nbytes = 1;
                    midi_service_stream_encode_full_bt_pkt(context);
                    ble_midi_stream->mes.nbytes = 0;
                }
            }
            else {
                // channel_message
                ble_midi_stream->running_status = ms_byte;
                ble_midi_stream->mes.msg_bytes[0] = ms_byte;
                ble_midi_stream->previous_timestamp = timestamp;
                ble_midi_stream->mes.timestamp_ms = timestamp;

                ++bytes_pushed;
                ble_midi_stream->next_msg_byte_idx = 1;
                ble_midi_stream->mes.nbytes = ble_midi_packet_is_channel | 3;
                uint8_t chan_msg_base = (ms_byte >> 4) & 0xF;
                if (chan_msg_base == 0xC || chan_msg_base == 0xD) {
                    ble_midi_stream->mes.nbytes = ble_midi_packet_is_channel | 2;
                }
            }
        } // end parsing status
        else {
            // data byte
            if ((ble_midi_stream->mes.nbytes & ble_midi_packet_is_sysex) == 0) {
                // this is a byte in a system exclusive message
                ble_midi_stream->mes.msg_bytes[ble_midi_stream->next_msg_byte_idx++] = ms_byte;
                ++ble_midi_stream->mes.nbytes;
                ++bytes_pushed;
                if (ble_midi_stream->next_msg_byte_idx >= 3) {
                    //midi_service_stream_request_send(context);
                    midi_service_stream_encode_full_bt_pkt(context);
                    ble_midi_stream->mes.timestamp_ms = ble_midi_stream->previous_timestamp;
                    ble_midi_stream->mes.nbytes = ble_midi_packet_is_sysex; // set length to 0
                }
            }
            else if ((ble_midi_stream->mes.nbytes & ble_midi_packet_is_channel) != 0 && ble_midi_stream->next_msg_byte_idx == 0)  {                
                if (ble_midi_stream->running_status == 0 || ble_midi_stream->running_status != ble_midi_stream->mes.msg_bytes[0]) {
                    // stream error
                    midi_service_stream_discard_stream(midi_stream, nbytes, ble_midi_stream);
                    return bytes_pushed;
                }
                else {
                    ble_midi_stream->mes.msg_bytes[1] = ms_byte;
                    ble_midi_stream->mes.timestamp_ms = timestamp;
                    ++bytes_pushed;
                    ble_midi_stream->next_msg_byte_idx = 2;
                    if ((ble_midi_stream->mes.nbytes & ble_midi_packet_nbytes_mask) == ble_midi_stream->next_msg_byte_idx) {
                        midi_service_stream_encode_full_bt_pkt(context);
                    }
                }
            }
            else {
                // normal message data bytes or a stream error
                uint8_t len = (ble_midi_stream->mes.nbytes & ble_midi_packet_nbytes_mask);
                if (ble_midi_stream->next_msg_byte_idx < len) {
                    ble_midi_stream->mes.msg_bytes[ble_midi_stream->next_msg_byte_idx++] = ms_byte;
                    ++bytes_pushed;
                    if (len == ble_midi_stream->next_msg_byte_idx) {
                        midi_service_stream_encode_full_bt_pkt(context);
                        if ((ble_midi_stream->mes.nbytes & ble_midi_packet_is_channel) == 0) {
                            ble_midi_stream->mes.nbytes = 0;
                        }
                    }
                }
                else {
                    // stream error
                    midi_service_stream_discard_stream(midi_stream, nbytes, ble_midi_stream);
                    return bytes_pushed;
                }
            }
        }
    }
    // parsed the full MIDI stream sent
    if (bytes_pushed > 0) {
        if (ble_midi_stream->pending_ble_pkt.nbytes > 0) {
            ring_buffer_push(&context->to_ble, (uint8_t*)&ble_midi_stream->pending_ble_pkt, sizeof(ble_midi_stream->pending_ble_pkt));
            ble_midi_stream->pending_ble_pkt.nbytes = 0;
            ble_midi_stream->pending_ble_midi_pkt_running_status = 0;
        }
        midi_service_stream_request_send(context);
    }
    return bytes_pushed;
}
// Return the context that contains connection_handle ==  con_handle;
// call with con_handle to HCI_CON_HANDLE_INVALID to find an available context
midi_service_stream_connection_t* get_context_for_conn_handle(hci_con_handle_t con_handle)
{
    for (uint8_t idx = 0; idx < MIDI_SERVICE_STREAM_HANDLER_MAX_CONNECTION; idx++) {
        if (midi_service_stream_connection[idx].connection_handle == con_handle) {
            return midi_service_stream_connection + idx;
        }
    }
    return NULL;
}

static void midi_can_send(void * void_context)
{
    midi_service_stream_connection_t* context = (midi_service_stream_connection_t*)void_context;
    ble_midi_packet_t pending_ble_pkt;

    if (ring_buffer_pop(&context->to_ble, (uint8_t*)&pending_ble_pkt, sizeof(pending_ble_pkt)) == sizeof(pending_ble_pkt)) {
        midi_service_server_send(context->connection_handle, pending_ble_pkt.pkt, pending_ble_pkt.nbytes);
    }
    if (ring_buffer_get_num_bytes(&context->to_ble) == 0) {
        context->waiting_for_send_notification_request = false;
    }
    else {
        // more packets in the buffer
        midi_service_server_request_can_send_now(&context->send_request, context->connection_handle);
    }
}

static void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    uint16_t conn_interval;
    hci_con_handle_t con_handle;

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    // print connection parameters (without using float operations)
                    con_handle    = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    conn_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
                    printf("LE Connection - Connection Interval: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
                    printf("LE Connection - Connection Latency: %u\n", hci_subevent_le_connection_complete_get_conn_latency(packet));

                    // request min con interval 15 ms for iOS per Apple accessory design guidelines, so we have to allow it:
                    // https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf
                    printf("LE Connection - Request 7.5ms-15 ms connection interval\n");
                    gap_request_connection_parameter_update(con_handle, 6, 12, 4, 0x0048);
                    break;
                case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
                    // print connection parameters (without using float operations)
                    con_handle    = hci_subevent_le_connection_update_complete_get_connection_handle(packet);
                    conn_interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
                    printf("LE Connection - Connection Param update - connection interval %u.%02u ms, latency %u\n", conn_interval * 125 / 100,
                        25 * (conn_interval & 3), hci_subevent_le_connection_update_complete_get_conn_latency(packet));
                    break;
                default:
                    client_packet_handler(packet_type, channel, packet, size);
                    break;
            }
            break;
        default:
            client_packet_handler(packet_type, channel, packet, size);
            break;
    }
}

static bool midi_service_stream_push(ring_buffer_t* buf, uint8_t* data, RING_BUFFER_SIZE_TYPE size)
{
    bool success = false;
    RING_BUFFER_SIZE_TYPE after_push = ring_buffer_get_num_bytes(buf);
    after_push += size;
    if (buf->bufsize >= after_push) {
        ring_buffer_push(buf, data, size);
        success = true;
    }
    return success;
}

static uint16_t midi_service_stream_ble_midi_decode_push(uint8_t* pkt, uint16_t nbytes, midi_service_stream_connection_t* context)
{
    // The shortest possible packet would be a packet header with one sysex byte
    if (pkt == NULL || nbytes < 2) {
        return 0;
    }
    // make sure the header byte is present
    if ((pkt[0] & 0xc0) != 0x80) {
        return 0;
    }
    uint16_t timestamp = ((uint16_t)(pkt[0] & 0x3f)) << 7;
    uint8_t prev_lsb = 0;
    uint16_t ndecoded = 1;
    uint8_t running_status = 0;
    uint8_t running_status_nbytes = 0;
    ble_midi_message_t mes = {0, {0,0,0}, 0};
    // check to see if the start of the packet is sysex continuation data
    if ((pkt[ndecoded] & 0x80) == 0) {
        // This has to be sysex continuation data or an error that we can't detect easily
        // ##### TODO move this block to a function
        mes.nbytes = ble_midi_packet_is_sysex;
        uint8_t idx = 0;
        while (ndecoded < nbytes && (pkt[ndecoded] & 0x80) == 0) {
            mes.msg_bytes[idx++] = pkt[ndecoded++];
            mes.nbytes++;
            if (idx == 3) {
                if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                    ndecoded -= 3;
                    return ndecoded;
                }
                idx = 0;
                mes.nbytes = ble_midi_packet_is_sysex;
            }
        }
        uint8_t bytecount = (mes.nbytes & ble_midi_packet_nbytes_mask);
        if (bytecount > 0) {
            if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                ndecoded -= bytecount;
                return ndecoded;
            }
            // if MSB==1 in the BLE packet, it could be a real-time message's timestamp or EOX message's timestamp
            mes.nbytes = ble_midi_packet_is_sysex;
        }
        // ##### END TODO
    }
    while (ndecoded < nbytes) {
        // message start usually is a timestamp followed by a status byte
        if (ndecoded+1 < nbytes) {
            if ((pkt[ndecoded] & 0x80) != 0 && (pkt[ndecoded+1] & 0x80) != 0) {
                // As long as it is not a sysex message, this is a timestamp followed by a status byte
                if ((mes.nbytes & ble_midi_packet_is_sysex) == 0) {
                    // 1st byte is the timestamp; 2nd byte is status byte
                    mes.timestamp_ms = midi_service_timestamp_decode(&timestamp, pkt[ndecoded++], &prev_lsb);
                    mes.msg_bytes[0] = pkt[ndecoded++];
                    if (mes.msg_bytes[0] >= 0xF8) {
                        mes.nbytes = ble_midi_packet_is_real_time + 1;
                        if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                            return ndecoded;
                        }
                    }
                    else if (mes.msg_bytes[0] > 0xF0 && mes.msg_bytes[0] <= 0xF7) {
                        // system common
                        if (mes.msg_bytes[0] == 0xF1 || mes.msg_bytes[0] == 0xF3) {
                            mes.nbytes = 2;
                        }
                        else if (mes.msg_bytes[0] == 0xF2) {
                            mes.nbytes = 3;
                        }
                        else {
                            mes.nbytes = 1;
                        }
                        if ((ndecoded + mes.nbytes - 1) <= nbytes) {
                            for (uint8_t idx = 1; idx < mes.nbytes; idx++) {
                                mes.msg_bytes[idx] = pkt[ndecoded++];
                            }
                            if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                                --ndecoded;
                                return ndecoded;
                            }
                        }
                        else {
                            return ndecoded;
                        }
                    }
                    else if (mes.msg_bytes[0] < 0xF0) {
                        // channel message
                        running_status = mes.msg_bytes[0];
                        uint8_t stat = (mes.msg_bytes[0] >> 4) & 0xF; // ignore the channel
                        if (stat == 0xC || stat == 0xD) {
                            mes.nbytes = 2;
                            running_status_nbytes = 2;
                        }
                        else {
                            running_status_nbytes = 3;
                        }
                        if ((running_status_nbytes + ndecoded -1) <= nbytes) {
                            for (uint8_t idx = 1; idx < running_status_nbytes; idx++) {
                                mes.msg_bytes[idx] = pkt[ndecoded++];
                            }
                            mes.nbytes = running_status_nbytes | ble_midi_packet_is_channel;
                            if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                                --ndecoded;
                                return ndecoded;
                            }
                        }
                    }
                    else { // 0xF0; start of sysex
                        // keep decoding until encountering a new timestamp.
                        mes.nbytes = ble_midi_packet_is_sysex + 1;
                        uint8_t idx = 1;
                        while (ndecoded < nbytes && (pkt[ndecoded] & 0x80) == 0) {
                            mes.msg_bytes[idx++] = pkt[ndecoded++];
                            mes.nbytes++;
                            if (idx == 3) {
                                if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                                    ndecoded -= 3;
                                    return ndecoded;
                                }
                                idx = 0;
                                mes.nbytes = ble_midi_packet_is_sysex;
                            }
                        }
                        uint8_t bytecount = (mes.nbytes & ble_midi_packet_nbytes_mask);
                        if (bytecount > 0) {
                            if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                                ndecoded -= bytecount;
                                return ndecoded;
                            }
                            mes.nbytes = ble_midi_packet_is_sysex;
                        }
                    }
                }
                else if (pkt[ndecoded+1] >= 0xF8) {
                    uint8_t bytecount = (mes.nbytes & ble_midi_packet_nbytes_mask);
                    if (bytecount > 0) {
                        if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                            ndecoded -= bytecount;
                            return ndecoded;
                        }
                    }
                    // This is a real-time message timestamp and status byte within a sysex message
                    ble_midi_message_t mes_rt = {0, {0,0,0}, 0};
                    mes_rt.nbytes = ble_midi_packet_is_real_time + 1;
                    mes_rt.timestamp_ms = midi_service_timestamp_decode(&timestamp, pkt[ndecoded++], &prev_lsb);
                    mes_rt.msg_bytes[0] = pkt[ndecoded++];
                    if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes_rt, sizeof(mes_rt))) {
                        ndecoded -= 2;
                        return ndecoded;
                    }
                    // real-time packet is out of the way, continue parsing the data
                    mes.nbytes = ble_midi_packet_is_sysex;
                    uint8_t idx = 0;
                    while (ndecoded < nbytes && (pkt[ndecoded] & 0x80) == 0) {
                        mes.msg_bytes[idx++] = pkt[ndecoded++];
                        mes.nbytes++;
                        if (idx == 3) {
                            if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                                ndecoded -= 3;
                                return ndecoded;
                            }
                            idx = 0;
                            mes.nbytes = ble_midi_packet_is_sysex;
                        }
                    }
                    bytecount = (mes.nbytes & ble_midi_packet_nbytes_mask);
                    if (bytecount > 0) {
                        if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                            ndecoded -= bytecount;
                            return ndecoded;
                        }
                        // if MSB==1 in the BLE packet, it could be a real-time message's timestamp or EOX message's timestamp
                        mes.nbytes = ble_midi_packet_is_sysex;
                    }
                }
                else if (pkt[ndecoded+1] == 0xF7) {
                    // was sysex, now sysex is over; push the last data packet, then the F7 packet on the next iteration.
                    uint8_t bytecount = (mes.nbytes & ble_midi_packet_nbytes_mask);
                    if (bytecount > 0) {
                        if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                            ndecoded -= bytecount;
                            return ndecoded;
                        }
                    }
                    mes.nbytes = 0; // clear is_sysex flag
                }
            }
            else if ((pkt[ndecoded] & 0x80) != 0 && (pkt[ndecoded+1] & 0x80) == 0) {
                // This pattern could be a channel message running status timestamp followed by the channel message data byte(s)
                if (running_status != 0 && (ndecoded + running_status_nbytes) <= nbytes) {
                    mes.timestamp_ms = midi_service_timestamp_decode(&timestamp, pkt[ndecoded++], &prev_lsb);
                    mes.msg_bytes[0] = running_status;
                    mes.nbytes = running_status_nbytes | ble_midi_packet_is_channel;
                    for (uint8_t idx = 1; idx < running_status_nbytes; idx++) {
                        mes.msg_bytes[idx] = pkt[ndecoded++];
                    }
                    if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                        ndecoded -= running_status_nbytes;
                        return ndecoded;
                    }
                }
                else {
                    // the pattern of timestamp plus not-timestamp is not legal otherwise at the beginning of a message
                    return ndecoded;
                }
            }
            else if ((pkt[ndecoded] & 0x80) == 0 && (pkt[ndecoded+1] & 0x80) == 0) {
                if (running_status == mes.msg_bytes[0] && (mes.nbytes & ble_midi_packet_is_channel) != 0 &&  (ndecoded + running_status_nbytes - 1) <= nbytes) {
                    // this is a running status channel message same as previous message with the same timestamp and new data
                    // status byte is already correct, and the there are enough data bytes in the undecoded (at least the full message length - 1)
                    for (uint8_t idx = 1; idx < running_status_nbytes; idx++) {
                        mes.msg_bytes[idx] = pkt[ndecoded++];
                    }
                    if (!midi_service_stream_push(&context->from_ble, (uint8_t*)&mes, sizeof(mes))) {
                        ndecoded -= (running_status_nbytes-1);
                        return ndecoded;
                    }
                }
                else {
                    // this should not happen
                    return ndecoded;
                }
            }
        }
    }
    return ndecoded;
}

/**
 * @brief This function handles packets from the Blueooth stack
 * 
 * @param packet_type 
 * @param channel 
 * @param packet 
 * @param size 
 */
static void midi_service_stream_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    hci_con_handle_t con_handle;
    midi_service_stream_connection_t* context;
    switch (packet_type){
        case HCI_EVENT_PACKET:
            if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) break;
            switch (hci_event_gattservice_meta_get_subevent_code(packet)){
                case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
                    con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
                    context = get_context_for_conn_handle(con_handle);
                    if (!context) break;
                    context->le_notification_enabled = 1;
                    midi_service_stream_init_ring_buffers(context);
                    context->waiting_for_send_notification_request = false;
                    context->send_request.callback = midi_can_send;
                    context->send_request.context = context;
                    context->ble_mtu = att_server_get_mtu(con_handle);
                    // also send this event up to the application so it can record the connection handle
                    client_packet_handler(packet_type, channel, packet, size);
                    break;
                case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
                    con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
                    context = get_context_for_conn_handle(con_handle);
                    if (!context) break;
                    context->le_notification_enabled = 0;
                    context->connection_handle = HCI_CON_HANDLE_INVALID;
                    client_packet_handler(packet_type, channel, packet, size);
                    break;
                default:
                    client_packet_handler(packet_type, channel, packet, size);
                    break;
            }
            break;
        case RFCOMM_DATA_PACKET:
            //printf("RECV: ");
            //printf_hexdump(packet, size);
            context = get_context_for_conn_handle((hci_con_handle_t) channel);
            uint16_t ndecoded = midi_service_stream_ble_midi_decode_push(packet, size, context);
            if (ndecoded != size) {
                printf("Parse error decoding midi packet\r\n");
                printf_hexdump(packet, size);
            }
            break;
        default:
            break;
    }
}

static void att_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    int mtu;
    midi_service_stream_connection_t* context;
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case ATT_EVENT_CONNECTED:
                    // setup new 
                    context = get_context_for_conn_handle(HCI_CON_HANDLE_INVALID);
                    if (!context) break;
                    // use the connection handle from this notification going forward
                    context->connection_handle = att_event_connected_get_handle(packet);
                    printf("%s: ATT connected, handle 0x%04x, max MIDI packet len %u\n", context->name, context->connection_handle, sizeof(context->to_ble_midi_stream.pending_ble_pkt.pkt));
                    break;
                case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
                    mtu = att_event_mtu_exchange_complete_get_MTU(packet) - 3;
                    context = get_context_for_conn_handle(att_event_mtu_exchange_complete_get_handle(packet));
                    if (!context) break;
                    context->ble_mtu = btstack_min(mtu - 3, sizeof(context->to_ble_midi_stream.pending_ble_pkt.pkt));
                    printf("%s: ATT MTU = %u => max MIDI packet len %u\n", context->name, mtu, context->ble_mtu);
                    break;
                case ATT_EVENT_CAN_SEND_NOW:
                    //TODO send pending data

                    break;
                case ATT_EVENT_DISCONNECTED:
                    context = get_context_for_conn_handle(att_event_disconnected_get_handle(packet));
                    if (!context) break;
                    // free connection
                    printf("%s: ATT disconnected, handle 0x%04x\n", context->name, context->connection_handle);
                    context->le_notification_enabled = 0;
                    context->connection_handle = HCI_CON_HANDLE_INVALID;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void midi_service_stream_init(btstack_packet_handler_t packet_handler)
{
    for (uint8_t idx = 0; idx < MIDI_SERVICE_STREAM_HANDLER_MAX_CONNECTION; idx++) {
        midi_service_stream_connection_t* context = midi_service_stream_connection+idx;
        context->connection_handle = HCI_CON_HANDLE_INVALID;
        midi_service_stream_init_ring_buffers(context);
        char name[] = "MIDI A";
        name[5] += idx;
        strcpy(context->name, name);
    }
    // register for HCI events
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(att_packet_handler);
    midi_service_server_init(midi_service_stream_packet_handler);
    client_packet_handler = packet_handler;
}

uint8_t midi_service_stream_write(hci_con_handle_t con_handle, uint8_t nbytes, uint8_t* midi_stream_bytes)
{
    midi_service_stream_connection_t* context = get_context_for_conn_handle(con_handle);
    if (context) {
        midi_service_stream_push_midi(midi_stream_bytes, nbytes, context);
    }
    return 0;
}

uint8_t midi_service_stream_read(hci_con_handle_t con_handle, uint8_t max_bytes, uint8_t* midi_stream_bytes, uint16_t* timestamp)
{
    midi_service_stream_connection_t* context = get_context_for_conn_handle(con_handle);
    ble_midi_message_t mes;
    uint8_t nread = 0;
    if (context != NULL) {
        uint8_t nbytes = ring_buffer_pop(&context->from_ble, (uint8_t*)&mes, sizeof(mes));
        if (nbytes == sizeof(mes)) {
            uint8_t bytecount = mes.nbytes & ble_midi_packet_nbytes_mask;
            if (bytecount <= max_bytes) {
                *timestamp = mes.timestamp_ms;
                memcpy(midi_stream_bytes, mes.msg_bytes, bytecount);
                nread = bytecount;
            }
        }
    }

    return nread;
}