/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Contains some code from MouseJack (developed by BastilleResearch)
 * see: https://github.com/BastilleResearch/nrf-research-firmware/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 * Copyright (c) 2016 Damien Cauquil
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "MicroBit.h"

extern "C" {

#include <stdio.h>
#include <string.h>

#include "py/runtime0.h"
#include "py/runtime.h"
#include "microbitobj.h"
#include <nrf_delay.h>

#define RADIO_DEFAULT_MODE          (0)
#define RADIO_SNIFF_MODE            (1)
#define RADIO_DEFAULT_MAX_PAYLOAD   (32)
#define RADIO_SNIFF_MAX_PAYLOAD     (38)
#define RADIO_DEFAULT_QUEUE_LEN     (10)
#define RADIO_DEFAULT_CHANNEL       (7)
#define RADIO_DEFAULT_POWER_DBM     (0)
#define RADIO_DEFAULT_BASE0         (0x75626974) // "uBit"
#define RADIO_DEFAULT_PREFIX0       (0)
#define RADIO_DEFAULT_DATA_RATE     (RADIO_MODE_MODE_Nrf_1Mbit)

#define RADIO_SHORTS_COMMON ( RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
                                                      RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk )

typedef enum {
    MODE_DEFAULT,
    MODE_SNIFF,
    MODE_ESB,
    MODE_SB,
    MODE_CX,
    MODE_BLE,
    MODE_BLE_LL,
} radio_mode_t;

typedef struct _radio_state_t {
    uint8_t mode;           // radio mode (tx/rx or sniff)
    uint8_t max_payload;    // 1-251 inclusive
    uint8_t queue_len;      // 1-254 inclusive
    uint8_t channel;        // 0-100 inclusive
    int8_t power_dbm;       // one of: -30, -20, -16, -12, -8, -4, 0, 4
    uint32_t base0;         // for BASE0 register
    uint8_t prefix0;        // for PREFIX0 register (lower 8 bits only)
    uint8_t data_rate;      // one of: RADIO_MODE_MODE_Nrf_{250Kbit,1Mbit,2Mbit}
    uint8_t pid;            // PID for ESB DPL
    uint8_t previous_pid;
    uint8_t sniff_raw;      // raw sniffing
} radio_state_t;

static radio_state_t radio_state;
static uint8_t *buf_end = NULL;
static uint8_t *rx_buf = NULL;
static uint8_t esb_ready = 1;
static uint8_t ping_pkt[] = {0x0F, 0x0F, 0x0F, 0x0F};
static uint32_t timestamp = 0;

/* BE to LE (and inv.) helper. */
static uint32_t bytewise_bit_swap(uint32_t inp)
{
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    return (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
}

/* CRC16-CCITT with 1-8 bits from a given byte */
uint16_t crc_update(uint16_t crc, uint8_t byte, uint8_t bits)
{
    crc = crc ^ (byte << 8);
    while(bits--)
        if((crc & 0x8000) == 0x8000) crc = (crc << 1) ^ 0x1021;
        else crc = crc << 1;
    crc = crc & 0xFFFF;
    return crc;
}

/* XN297 routines */
static const uint8_t xn297_scramble[] = {
    0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
    0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
    0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
    0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
    0x8e, 0xc5, 0x2f};

static const uint16_t xn297_crc_xorout[] = {
    0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C,
    0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828, 0x814B,
    0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
    0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401,
    0x2138, 0x129F, 0xB3A0, 0x2988};

uint8_t bit_reverse(uint8_t b_in)
{
    uint8_t b_out = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}

static const uint16_t polynomial = 0x1021;
static const uint16_t initial    = 0xb5d2;
uint16_t crc16_update(uint16_t crc, unsigned char a)
{
    crc ^= a << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ polynomial;
            } else {
            crc = crc << 1;
        }
    }
    return crc;
}

/**
 * BLE channel computation routine.
 **/

uint8_t channel_resolver_get_frequency(uint8_t channel)
{
	uint8_t freq;

	/* Special cases for the advertise channels */
	if(channel == 37)
		freq = 2;
	else if(channel == 38)
		freq = 26;
	else if(channel == 39)
		freq = 80;
	else
		freq = channel + (channel < 11 ? 2 : 3) * 2; // Spec Vol. 6, Part B, 1.4.1

	return freq;
}


/**
 * The sniffing code comes from BastilleResearch's MouseJack firmware for
 * the CrazyRadio PA (based on NRF24LU1P).
 **/

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
        NRF_RADIO->TASKS_START = 1;
    }

    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;

        if (radio_state.mode == MODE_DEFAULT) {
            size_t max_len = NRF_RADIO->PCNF1 & 0xff;
            size_t len = rx_buf[0];
            if (len > max_len) {
                len = max_len;
                rx_buf[0] = len;
            }

            //printf("radio end pos=%d len=%d [%d %d %d %d]\r\n", rx_buf - MP_STATE_PORT(radio_buf), len, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

            // if the CRC is valid then accept the packet
            if (NRF_RADIO->CRCSTATUS == 1) {
                //printf("rssi: %d\r\n", -NRF_RADIO->RSSISAMPLE);

                // only move the rx_buf pointer if there is enough room for another full packet
                if (rx_buf + 1 + len + 1 + max_len <= buf_end) {
                    rx_buf += 1 + len;
                    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
                }
            }
        } else if (radio_state.mode == MODE_SNIFF) {
            int x, offset;
            uint8_t payload_length;
            uint16_t crc, crc_given;
            uint8_t payload[RADIO_SNIFF_MAX_PAYLOAD];

            /* Copy sniffed packet. */
            memcpy(payload, rx_buf, radio_state.max_payload);

            if (!radio_state.sniff_raw) {
                // In promiscuous mode without a defined address prefix, we attempt to
                // decode the payload as-is, and then shift it by one bit and try again
                // if the first attempt did not pass the CRC check. The purpose of this
                // is to minimize missed detections that happen if we were to use both
                // 0xAA and 0x55 as the nonzero promiscuous mode address bytes.
                for(offset = 0; offset < 2; offset++) {
                    // Shift the payload right by one bit if this is the second pass
                    if(offset == 1) {
                        for(x = 31; x >= 0; x--) {
                            if(x > 0) payload[x] = payload[x - 1] << 7 | payload[x] >> 1;
                            else payload[x] = payload[x] >> 1;
                        }
                    }

                    // Read the payload length
                    payload_length = payload[5] >> 2;

                    // Check for a valid payload length, which is less than the usual 32 bytes
                    // because we need to account for the packet header, CRC, and part or all
                    // of the address bytes.
                    if(payload_length <= 40) {
                        // Read the given CRC
                        crc_given = (payload[6 + payload_length] << 9) | ((payload[7 + payload_length]) << 1);
                        crc_given = (crc_given << 8) | (crc_given >> 8);
                        if(payload[8 + payload_length] & 0x80) crc_given |= 0x100;

                        // Calculate the CRC
                        crc = 0xFFFF;
                        for(x = 0; x < 6 + payload_length; x++) crc = crc_update(crc, payload[x], 8);
                        crc = crc_update(crc, payload[6 + payload_length] & 0x80, 1);
                        crc = (crc << 8) | (crc >> 8);

                        /* CRC match ? */
                        if(crc == crc_given)
                        {
                            /* Kinda lock we use to avoid conflicts. */
                            esb_ready = 0;

                            /* Write packet size to rx_buf. */
                            // rx_buf[0] = 0xC0 | payload_length;
                            rx_buf[0] = 0xC0 | (payload_length + 2);

                            /* Write address to rx_buf. */
                            memcpy(&rx_buf[1], payload, 5);

                            rx_buf[6] = payload[5];        // LEN | PID
                            rx_buf[7] = payload[6] & 0x80; // NO_ACK

                            for(x = 0; x < payload_length + 3; x++)
                                rx_buf[8+x] = ((payload[6 + x] << 1) & 0xFF) | (payload[7 + x] >> 7);

                            // only move the rx_buf pointer if there is enough room for another full packet
                            if (rx_buf + 2*radio_state.max_payload <= buf_end) {
                                rx_buf += radio_state.max_payload;
                                NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
                            }

                            /* Data updated, ready to be read. */
                            esb_ready = 1;
                        }
                    }
                }
            }
        } else if (radio_state.mode == MODE_ESB) {
            size_t len = rx_buf[0];

            // if the CRC was valid then accept the packet
            if (NRF_RADIO->CRCSTATUS == 1) {
                // only move the rx_buf pointer if there is enough room for another full packet
                if (rx_buf + 2 + len + 34 <= buf_end) {
                    rx_buf += len + 2;
                    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
                }
            }
        } 

        NRF_RADIO->TASKS_START = 1;
    }
}

static void ensure_enabled(void) {
    if (MP_STATE_PORT(radio_buf) == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "radio is not enabled"));
    }
}

static void radio_disable(void) {
    NVIC_DisableIRQ(RADIO_IRQn);
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    // free any old buffers
    if (MP_STATE_PORT(radio_buf) != NULL) {
        m_del(uint8_t, MP_STATE_PORT(radio_buf), buf_end - MP_STATE_PORT(radio_buf));
        MP_STATE_PORT(radio_buf) = NULL;
    }
}

static void radio_enable(void) {
    radio_disable();

    // Enable default mode
    radio_state.mode = MODE_DEFAULT;
    radio_state.sniff_raw = 0;

    // allocate tx and rx buffers
    size_t max_payload = radio_state.max_payload + 1; // an extra byte to store the length
    size_t queue_len = radio_state.queue_len + 1; // one extra for tx buffer
    MP_STATE_PORT(radio_buf) = m_new(uint8_t, max_payload * queue_len);
    buf_end = MP_STATE_PORT(radio_buf) + max_payload * queue_len;
    rx_buf = MP_STATE_PORT(radio_buf) + max_payload; // start is tx buffer

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
    NRF_RADIO->TXPOWER = radio_state.power_dbm;

    // should be between 0 and 100 inclusive (actual physical freq is 2400MHz + this register)
    NRF_RADIO->FREQUENCY = radio_state.channel;

    // configure data rate
    NRF_RADIO->MODE = radio_state.data_rate;

    // The radio supports filtering packets at the hardware level based on an address.
    // We use a 5-byte address comprised of 4 bytes (set by BALEN=4 below) from the BASEx
    // register, plus 1 byte from PREFIXm.APn.
    // The (x,m,n) values are selected by the logical address.  We use logical address 0
    // which means using BASE0 with PREFIX0.AP0.
    NRF_RADIO->BASE0 = radio_state.base0;
    NRF_RADIO->PREFIX0 = radio_state.prefix0;
    NRF_RADIO->TXADDRESS = 0; // transmit on logical address 0
    NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

    // LFLEN=8 bits, S0LEN=0, S1LEN=0
    NRF_RADIO->PCNF0 = 0x00000008;
    // STATLEN=0, BALEN=4, ENDIAN=0 (little), WHITEEN=1
    NRF_RADIO->PCNF1 = 0x02040000 | radio_state.max_payload;

    // Enable automatic 16bit CRC generation and checking, and configure how the CRC is calculated.
    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // Set the start random value of the data whitening algorithm. This can be any non zero number.
    NRF_RADIO->DATAWHITEIV = 0x18;

    // set receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;

    // configure interrupts
    NRF_RADIO->INTENSET = 0x00000008;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

    // enable receiver
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
}

/* Enable sniff mode based on Goodspeed's hack */
static void radio_enable_sniff(void) {
    radio_disable();

    // Sniff mode on
    radio_state.mode = MODE_SNIFF;
    radio_state.max_payload = 38; // 38 bytes to be sure to get 32 bytes payloads :)

    // allocate tx and rx buffers
    size_t max_payload = 40; // an extra byte to store the length
    size_t queue_len = radio_state.queue_len; // one extra for tx buffer
    MP_STATE_PORT(radio_buf) = m_new(uint8_t, max_payload * queue_len);
    buf_end = MP_STATE_PORT(radio_buf) + max_payload * queue_len;
    rx_buf = MP_STATE_PORT(radio_buf) + max_payload; // start is tx buffer

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
    NRF_RADIO->TXPOWER = radio_state.power_dbm;

    // should be between 0 and 100 inclusive (actual physical freq is 2400MHz + this register)
    NRF_RADIO->FREQUENCY = radio_state.channel;

    // configure data rate
    NRF_RADIO->MODE = radio_state.data_rate;

    // The radio supports filtering packets at the hardware level based on an address.
    // We use a 5-byte address comprised of 4 bytes (set by BALEN=4 below) from the BASEx
    // register, plus 1 byte from PREFIXm.APn.
    // The (x,m,n) values are selected by the logical address.  We use logical address 0
    // which means using BASE0 with PREFIX0.AP0.
    NRF_RADIO->BASE0 = 0x00000000;
    NRF_RADIO->PREFIX0 = 0x55; // preamble

    // LFLEN=0 bits, S0LEN=0, S1LEN=0
    NRF_RADIO->PCNF0 = 0x00000000;
    // STATLEN=40, MAXLEN=40, BALEN=1, ENDIAN=1 (big), WHITEEN=0
    NRF_RADIO->PCNF1 = 0x01012828;

    // Disable CRC
    NRF_RADIO->CRCCNF = 0x0;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // set receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;

    // configure interrupts
    NRF_RADIO->INTENSET = 0x00000008;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    //NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
    NRF_RADIO->SHORTS = 0;

    // enable receiver
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
}

/**
 * Enhanced ShockBurst mode
 *
 * This mode is a simplistic implementation of ESB with no Auto-ack feature and
 * a fixed 16-bit CRC as used by the default implementation in NRF24L01+. It
 * uses a small amount of memory but allows to transmit and receive data based
 * on one of the most classic configuration.
 *
 * Sniffing is also possible thanks to Travis Goodspeed's NRF24L01+ hack, that
 * still works with NRF51 SDK \o/
 *
 * This mode should be used from Python as following:
 *
 * >> radio.on()
 * >> radio.config(channel=10, address=0x11223344, group=0x55)
 * >> radio.esb()
 * >> radio.send_bytes(b'TROLOLO')
 * >> pkt = radio.receive()
 **/

static void radio_enable_esb(void) {
    radio_disable();

    // Sniff mode on
    radio_state.mode = MODE_ESB;
    radio_state.previous_pid = 0xFF;

    // allocate tx and rx buffers
    size_t max_payload = 34; // an extra byte to store the length
    size_t queue_len = radio_state.queue_len; // one extra for tx buffer
    MP_STATE_PORT(radio_buf) = m_new(uint8_t, max_payload * queue_len);
    buf_end = MP_STATE_PORT(radio_buf) + max_payload * queue_len;
    rx_buf = MP_STATE_PORT(radio_buf) + max_payload; // start is tx buffer

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
    NRF_RADIO->TXPOWER = radio_state.power_dbm;

    // should be between 0 and 100 inclusive (actual physical freq is 2400MHz + this register)
    NRF_RADIO->FREQUENCY = radio_state.channel;

    // configure default data rate
    NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit;
    radio_state.data_rate = RADIO_MODE_MODE_Nrf_2Mbit;

    // The radio supports filtering packets at the hardware level based on an address.
    // We use a 5-byte address comprised of 4 bytes (set by BALEN=4 below) from the BASEx
    // register, plus 1 byte from PREFIXm.APn.
    // The (x,m,n) values are selected by the logical address.  We use logical address 0
    // which means using BASE0 with PREFIX0.AP0.
    //
    // ESB mode uses address as big endian, combined with group with a default address
    // size of 5 bytes (mostly used on ESB compatible devices).
    // ESB address is composed of address.group, for instance to address device
    // 11:22:33:44:55 then use: address=0x11223344 and group=0x55.
    NRF_RADIO->BASE0 = bytewise_bit_swap(
        (radio_state.base0 & 0x000000ff)<<24 |
        (radio_state.base0 & 0xff000000)>>24 |
        (radio_state.base0 & 0x0000ff00)<< 8 |
        (radio_state.base0 & 0x00ff0000)>> 8
    );
    NRF_RADIO->PREFIX0 = bytewise_bit_swap(radio_state.prefix0);

    // LFLEN=6 bits, S0LEN=0, S1LEN=3
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (6 << RADIO_PCNF0_LFLEN_Pos) | (3 << RADIO_PCNF0_S1LEN_Pos);
    // STATLEN=0, BALEN=4, ENDIAN=1 (big), WHITEEN=0
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled        << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big              << RADIO_PCNF1_ENDIAN_Pos)  |
                       ((4) << RADIO_PCNF1_BALEN_Pos)   |
                       (0                                   << RADIO_PCNF1_STATLEN_Pos) |
                       (38        << RADIO_PCNF1_MAXLEN_Pos);

    NRF_RADIO->TXADDRESS = 0; // transmit on logical address 0
    NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // set receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;

    // configure interrupts
    NRF_RADIO->INTENSET = 0x00000008;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->SHORTS = 0;

    // enable receiver
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
}

static void radio_enable_sb(void) {
    // REMOVE
}

static void radio_enable_cx(void) {
    // REMOVE
}


/**
 * Raw BLE mode
 *
 * This mode allows BLE advertisement and connection request sniffing (at the moment),
 * but provides the data as-is without any processing.
 */

void radio_enable_ble(void) {
    // REMOVE
}

/**
 * Raw BLE Link Layer mode
 *
 * This mode allows BLE link layer sniffing, but does not handle CRC natively.
 * However, chance of a false positive on a 4-byte Access Address is low, so we
 * may manage to check CRC manually.
 * but provides the data as-is without any processing.
 */

void radio_enable_ble_ll(void) {
    // REMOVE
}



void radio_send(const void *buf, size_t len, const void *buf2, size_t len2) {
    ensure_enabled();

    /* Not available in sniffing and BLE mode. */
    if (radio_state.mode == MODE_SNIFF)
        return;

    if (radio_state.mode == MODE_ESB)
    {
        if (len > 254)
            len = 254;

        /* Write header (size + PID/ACK). */
        MP_STATE_PORT(radio_buf)[0] = len;
        MP_STATE_PORT(radio_buf)[1] = (radio_state.pid<<1)|0; /* NO ACK required. */
        radio_state.pid = (radio_state.pid + 1)%4; /* Increment PID. */

        /* Copy payload into memory. */
        memcpy(MP_STATE_PORT(radio_buf)+2, buf, len);
    } 

    // transmission will occur synchronously
    NVIC_DisableIRQ(RADIO_IRQn);

    // Turn off the transceiver.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);

    // Configure the radio to send the buffer provided.
    NRF_RADIO->PACKETPTR = (uint32_t)MP_STATE_PORT(radio_buf);

    // Turn on the transmitter, and wait for it to signal that it's ready to use.
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    // Start transmission and wait for end of packet.
    NRF_RADIO->TASKS_START = 1;
    NRF_RADIO->EVENTS_END = 0;
    while (NRF_RADIO->EVENTS_END == 0);

    // Return the radio to using the default receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;

    // Turn off the transmitter.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);

    // Start listening for the next packet
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
}

// REMOVE

static mp_obj_t radio_receive(bool typed_packet) {
    uint8_t *buf = NULL;
    size_t len = 0;
    mp_obj_t ret = mp_const_none;

    ensure_enabled();

    // disable the radio irq while we receive the packet
    NVIC_DisableIRQ(RADIO_IRQn);

    switch(radio_state.mode) {
        case MODE_ESB:
            {
                buf = MP_STATE_PORT(radio_buf) + 34; // skip tx buf

                // return None if there are no packets waiting
                if (rx_buf == buf) {
                    NVIC_EnableIRQ(RADIO_IRQn);
                    return mp_const_none;
                }

                // Get packet len and PID
                len = buf[0];
                uint8_t pid = buf[1] >> 1;

                // if (pid != radio_state.previous_pid) {
                    /* Don't care about acks for now :) */
                    ret = mp_obj_new_bytes((const unsigned char *)buf, len+2);
                    // radio_state.previous_pid = pid;

                    // copy the rest of the packets down and restart the radio
                    memmove(buf, buf + 2 + len, rx_buf - (buf + 2 + len));
                    rx_buf -= (2 + len);                    
                //}
            }
            break;
    }

    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    NVIC_EnableIRQ(RADIO_IRQn);

    return ret;
}

static mp_obj_t radio_sniff(void) {
    ensure_enabled();

    // disable the radio irq while we receive the packet
    NVIC_DisableIRQ(RADIO_IRQn);

    // get the pointer to the next packet
    uint8_t *buf = MP_STATE_PORT(radio_buf) + (NRF_RADIO->PCNF1 & 0xff) ; // skip tx buf

    // must wait (writing in progress ?)
    if (esb_ready == 0) {
        NVIC_EnableIRQ(RADIO_IRQn);
        return mp_const_none;
    }

    // return None if there are no packets waiting
    if (rx_buf == buf) {
        NVIC_EnableIRQ(RADIO_IRQn);
        return mp_const_none;
    }

    /* Wrong packet format */
    if ((buf[0] & 0xC0) != 0xC0) {
        NVIC_EnableIRQ(RADIO_IRQn);
        return mp_const_none;
    }


    /*
    size_t len = buf[0]&0x3F;
    if (len > 32)
        len = 32;
    mp_obj_t ret;
    ret = mp_obj_new_bytes(&buf[1], len + 5); // if it raises the radio irq remains disabled...

    // copy the rest of the packets down and restart the radio
    memmove(buf, buf + radio_state.max_payload, rx_buf - (buf + radio_state.max_payload));
    rx_buf -= radio_state.max_payload;
    */

    size_t len = rx_buf - buf;
    mp_obj_t ret;
    ret = mp_obj_new_bytes(&buf[0], len);
    // copy the rest of the packets down and restart the radio
    rx_buf = buf;

    NRF_RADIO->PACKETPTR = (uint32_t)rx_buf;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    NVIC_EnableIRQ(RADIO_IRQn);

    return ret;
}


/*****************************************************************************/
// MicroPython bindings and module

STATIC mp_obj_t mod_radio_reset(void) {
    radio_state.mode = RADIO_DEFAULT_MODE;
    radio_state.max_payload = RADIO_DEFAULT_MAX_PAYLOAD;
    radio_state.queue_len = RADIO_DEFAULT_QUEUE_LEN;
    radio_state.channel = RADIO_DEFAULT_CHANNEL;
    radio_state.power_dbm = RADIO_DEFAULT_POWER_DBM;
    radio_state.base0 = RADIO_DEFAULT_BASE0;
    radio_state.prefix0 = RADIO_DEFAULT_PREFIX0;
    radio_state.data_rate = RADIO_DEFAULT_DATA_RATE;

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_reset_obj, mod_radio_reset);

STATIC mp_obj_t mod_radio_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    (void)pos_args; // unused

    if (n_args != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_TypeError, "arguments must be keyword arguments"));
    }

    // make a copy of the radio state so we don't change anything if there are value errors
    radio_state_t new_state = radio_state;

    qstr arg_name = MP_QSTR_;
    for (size_t i = 0; i < kw_args->alloc; ++i) {
        if (MP_MAP_SLOT_IS_FILLED(kw_args, i)) {
            mp_int_t value = mp_obj_get_int_truncated(kw_args->table[i].value);
            arg_name = mp_obj_str_get_qstr(kw_args->table[i].key);
            switch (arg_name) {
                case MP_QSTR_length:
                    if (!(1 <= value && value <= 251)) {
                        goto value_error;
                    }
                    new_state.max_payload = value;
                    break;

                case MP_QSTR_queue:
                    if (!(1 <= value && value <= 254)) {
                        goto value_error;
                    }
                    new_state.queue_len = value;
                    break;

                case MP_QSTR_channel:
                    if (!(0 <= value && value <= 100)) {
                        goto value_error;
                    }
                    new_state.channel = value;
                    break;

                case MP_QSTR_power: {
                    if (!(0 <= value && value <= 7)) {
                        goto value_error;
                    }
                    static int8_t power_dbm_table[8] = {-30, -20, -16, -12, -8, -4, 0, 4};
                    new_state.power_dbm = power_dbm_table[value];
                    break;
                }

                case MP_QSTR_data_rate:
                    if (!(value == RADIO_MODE_MODE_Nrf_250Kbit
                        || value == RADIO_MODE_MODE_Nrf_1Mbit
                        || value == RADIO_MODE_MODE_Nrf_2Mbit
                        || value == RADIO_MODE_MODE_Ble_1Mbit)) {
                        goto value_error;
                    }
                    new_state.data_rate = value;
                    break;

                case MP_QSTR_address:
                    new_state.base0 = value;
                    break;

                case MP_QSTR_group:
                    if (!(0 <= value && value <= 255)) {
                        goto value_error;
                    }
                    new_state.prefix0 = value;
                    break;

                case MP_QSTR_raw:
                    if (!(value == 0 || value == 1)) {
                        goto value_error;
                    }
                    new_state.sniff_raw = value;
                    break;

                default:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "unknown argument '%q'", arg_name));
                    break;
            }
        }
    }

    // reconfigure the radio with the new state

    if (MP_STATE_PORT(radio_buf) == NULL) {
        // radio disabled, just copy state
        radio_state = new_state;
    } else {
        // radio eabled
        if (new_state.max_payload != radio_state.max_payload || new_state.queue_len != radio_state.queue_len) {
            // tx/rx buffer size changed which requires reallocating the buffers
            radio_disable();
            radio_state = new_state;
            radio_enable();
        } else {
            // only registers changed so make the changes go through efficiently

            // disable radio
            NVIC_DisableIRQ(RADIO_IRQn);
            NRF_RADIO->EVENTS_DISABLED = 0;
            NRF_RADIO->TASKS_DISABLE = 1;
            while (NRF_RADIO->EVENTS_DISABLED == 0);

            // change state
            radio_state = new_state;
            NRF_RADIO->TXPOWER = radio_state.power_dbm;

            if (radio_state.mode != MODE_BLE) {
                NRF_RADIO->FREQUENCY = radio_state.channel;
                NRF_RADIO->MODE = radio_state.data_rate;
            } else {
                /* BLE only, switch channel and update whitening IV. */
                NRF_RADIO->FREQUENCY = channel_resolver_get_frequency(radio_state.channel);
                NRF_RADIO->DATAWHITEIV = radio_state.channel;
            }

            /* Update BASE0 and PREFIX0 if required, except when sniffing. */
            if ((radio_state.mode == MODE_DEFAULT) || (radio_state.mode == MODE_BLE)) {
                NRF_RADIO->BASE0 = radio_state.base0;
                NRF_RADIO->PREFIX0 = radio_state.prefix0;
            } else if (radio_state.mode == MODE_SNIFF){
                /* Specific values used for sniffing. */
                NRF_RADIO->BASE0 = 0x00000000;
                NRF_RADIO->PREFIX0 = 0x55;
            } else {
                /* Used in ESB and SB mode .*/
                NRF_RADIO->BASE0 = bytewise_bit_swap(
                    (radio_state.base0 & 0x000000ff)<<24 |
                    (radio_state.base0 & 0xff000000)>>24 |
                    (radio_state.base0 & 0x0000ff00)<< 8 |
                    (radio_state.base0 & 0x00ff0000)>> 8
                );
                NRF_RADIO->PREFIX0 = bytewise_bit_swap(radio_state.prefix0);
            }

            // need to set RXEN for FREQUENCY decision point
            NRF_RADIO->EVENTS_READY = 0;
            NRF_RADIO->TASKS_RXEN = 1;
            while (NRF_RADIO->EVENTS_READY == 0);

            // need to set START for BASE0 and PREFIX0 decision point
            NRF_RADIO->EVENTS_END = 0;
            NRF_RADIO->TASKS_START = 1;

            NVIC_ClearPendingIRQ(RADIO_IRQn);
            NVIC_EnableIRQ(RADIO_IRQn);
        }
    }

    return mp_const_none;

value_error:
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "value out of range for argument '%q'", arg_name));
}
MP_DEFINE_CONST_FUN_OBJ_KW(mod_radio_config_obj, 0, mod_radio_config);

STATIC mp_obj_t mod_radio_on(void) {
    radio_enable();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_on_obj, mod_radio_on);

STATIC mp_obj_t mod_radio_sniff_on(void) {
    radio_enable_sniff();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_sniff_on_obj, mod_radio_sniff_on);

STATIC mp_obj_t mod_radio_esb(void) {
    radio_enable_esb();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_esb_obj, mod_radio_esb);

STATIC mp_obj_t mod_radio_sb(void) {
    radio_enable_sb();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_sb_obj, mod_radio_sb);

STATIC mp_obj_t mod_radio_cx(void) {
    radio_enable_cx();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_cx_obj, mod_radio_cx);

STATIC mp_obj_t mod_radio_ble(void) {
    radio_enable_ble();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_ble_obj, mod_radio_ble);

STATIC mp_obj_t mod_radio_off(void) {
    radio_disable();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_off_obj, mod_radio_off);

STATIC mp_obj_t mod_radio_send_bytes(mp_obj_t buf_in) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    radio_send(bufinfo.buf, bufinfo.len, NULL, 0);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_send_bytes_obj, mod_radio_send_bytes);

STATIC mp_obj_t mod_radio_ping(void) {
    // REMOVE
    return mp_const_false;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_ping_obj, mod_radio_ping);

STATIC mp_obj_t mod_radio_find(void) {
    // REMOVE
    return mp_const_false;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_find_obj, mod_radio_find);


STATIC mp_obj_t mod_radio_receive_bytes(void) {
    return radio_receive(false);
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_receive_bytes_obj, mod_radio_receive_bytes);

STATIC mp_obj_t mod_radio_send(mp_obj_t buf_in) {
    mp_uint_t len;

    /* Switch to send_bytes if ESB mode is enabled. */
    if ((radio_state.mode == MODE_ESB) || (radio_state.mode == MODE_SB) || (radio_state.mode == MODE_CX) || (radio_state.mode == MODE_BLE)) {
        const char *data = mp_obj_str_get_data(buf_in, &len);
        radio_send(data, len, NULL, 0);
    }
    else
    {
        const char *data = mp_obj_str_get_data(buf_in, &len);
        radio_send("\x01\x00\x01", 3, data, len);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_send_obj, mod_radio_send);

STATIC mp_obj_t mod_radio_receive(void) {
    if ((radio_state.mode == MODE_ESB) || (radio_state.mode == MODE_SB) || (radio_state.mode == MODE_CX) || (radio_state.mode == MODE_BLE))
        return radio_receive(false);
    else if (radio_state.mode == MODE_DEFAULT)
        return radio_receive(true);
    else
        return radio_sniff();
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_receive_obj, mod_radio_receive);

STATIC mp_obj_t mod_radio_sniff(void) {
    return radio_sniff();
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_sniff_obj, mod_radio_sniff);





void esb_send_bytes_cksum(char* buffer, uint8_t buffer_size) {
    char payload[32];
    uint8_t payload_size = buffer_size + 1;

    for(int i=0; i < buffer_size; i++) {
        payload[i] = buffer[i];
    }

    uint8_t cksum = 0xff;

    for (int n = 0; n < buffer_size; n++) {
        cksum = (cksum - payload[n]) & 0xff;
    }
    cksum = (cksum + 1) & 0xff;
    payload[buffer_size] = cksum;

    radio_send(payload, payload_size, NULL, 0);


    // uint8_t retry_count = 4;
    int ack_received = 0;

    // while(retry_count-- && ack_received == 0) {
    /* Wait a bit for an ACK ... */
    nrf_delay_us(250);

    /* Check if we got a valid packet. */
    if(NRF_RADIO->EVENTS_END && NRF_RADIO->CRCSTATUS != 0)
    {
        /* Ack received \o/ ! */
        ack_received = 1;
    }

    /* Cleaning and resuming IRQ handler. */
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->INTENSET = 0x00000008;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    // }
}


// STATIC mp_obj_t mod_radio_esb_send_keys(mp_obj_t buf_in) {
//     char payload[32];
//     uint8_t payload_size = 10;
    
//     mp_uint_t len;
//     const char *data = mp_obj_str_get_data(buf_in, &len);


//     for(int i=0; i < payload_size; i++) {
//         payload[i] = 0;
//     }

//     payload[0] = 0x00;
//     payload[1] = 0xC1; // opcode
//     payload[2] = 0x00; // meta


//     for(int i=0; i < len; i++) {
//         payload[3+i] = data[i];
//     }

//     uint8_t cksum = 0xff;
//     int last = payload_size - 1;  

//     for (int n = 0; n < last; n++) {
//         cksum = (cksum - payload[n]) & 0xff;
//     }
//     cksum = (cksum + 1) & 0xff;
//     payload[last] = cksum;

//     printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7], payload[8], payload[9]);
//     radio_send(payload, payload_size, NULL, 0);


//     return mp_const_none;
// }
// MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_esb_send_keys_obj, mod_radio_esb_send_keys);

STATIC mp_obj_t mod_radio_esb_send_keys(mp_obj_t buf_in) {
    char payload[32];
    uint8_t payload_size = 10;
    
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(buf_in, &len);


    for(int i=0; i < payload_size; i++) {
        payload[i] = 0;
    }

    payload[0] = 0x00;
    payload[1] = 0xC1; // opcode

    for(int i=0; i < len; i++) {
        payload[2+i] = data[i];
    }

    esb_send_bytes_cksum(payload, 9);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_esb_send_keys_obj, mod_radio_esb_send_keys);


STATIC mp_obj_t mod_radio_esb_send_bytes(mp_obj_t buf_in) {
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(buf_in, &len);

    // esb_send_bytes_cksum(data, len):

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_esb_send_bytes_obj, mod_radio_esb_send_bytes);



// STATIC mp_obj_t mod_radio_esb_send_full_keep_alive(void) {
//     // 00 4f 00 00 55 00 00 00 00
//     char *keep_alive_set_00_55 = "\x00\x4f\x00\x00\x55\x00\x00\x00\x00";
//     char *keep_alive_set_03_70 = "\x00\x4f\x00\x03\x70\x00\x00\x00\x00";
//     char *keep_alive_00_55 = "\x00\x40\x00\x55";
//     char *keep_alive_03_70 = "\x00\x40\x03\x70";

//     esb_send_bytes_cksum(keep_alive_set_00_55, 9);

//     for (int i=0; i< 12; i++) {
//         esb_send_bytes_cksum(keep_alive_00_55, 4);
//         nrf_delay_ms(65);
//     }


//     esb_send_bytes_cksum(keep_alive_set_03_70, 9);

//     return mp_const_none;
// }
// MP_DEFINE_CONST_FUN_OBJ_0(mod_radio_esb_send_keep_alive_obj, mod_radio_esb_send_keep_alive);


STATIC mp_obj_t mod_radio_esb_send_set_keep_alive(mp_obj_t buf_in) {
    char payload[32];
    uint8_t payload_size = 10;
    
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(buf_in, &len);

    payload[0] = 0x00;
    payload[1] = 0x4f; // opcode
    payload[2] = 0x00;
    payload[3] = data[0];
    payload[4] = data[1];
    payload[5] = 0x00;
    payload[6] = 0x00;
    payload[7] = 0x00;
    payload[8] = 0x00;
    payload[9] = 0x00;

    esb_send_bytes_cksum(payload, 9);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_esb_send_set_keep_alive_obj, mod_radio_esb_send_set_keep_alive);

STATIC mp_obj_t mod_radio_esb_send_keep_alive(mp_obj_t buf_in) {
    char payload[32];
    uint8_t payload_size = 10;
    
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(buf_in, &len);

    payload[0] = 0x00;
    payload[1] = 0x40; // opcode
    payload[2] = data[0];
    payload[3] = data[1];
    payload[4] = 0x00;

    esb_send_bytes_cksum(payload, 4);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mod_radio_esb_send_keep_alive_obj, mod_radio_esb_send_keep_alive);



STATIC const mp_map_elem_t radio_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_radio) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___init__), (mp_obj_t)&mod_radio_reset_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_reset), (mp_obj_t)&mod_radio_reset_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_config), (mp_obj_t)&mod_radio_config_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_on), (mp_obj_t)&mod_radio_on_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_off), (mp_obj_t)&mod_radio_off_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_bytes), (mp_obj_t)&mod_radio_send_bytes_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_receive_bytes), (mp_obj_t)&mod_radio_receive_bytes_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send), (mp_obj_t)&mod_radio_send_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_receive), (mp_obj_t)&mod_radio_receive_obj },

    /* ESB Sniffing. */
    { MP_OBJ_NEW_QSTR(MP_QSTR_sniff_on), (mp_obj_t)&mod_radio_sniff_on_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sniff), (mp_obj_t)&mod_radio_sniff_obj },

    /* ESB functions */
    { MP_OBJ_NEW_QSTR(MP_QSTR_esb_send_keys), (mp_obj_t)&mod_radio_esb_send_keys_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_esb_send_bytes), (mp_obj_t)&mod_radio_esb_send_bytes_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_esb_send_set_keep_alive), (mp_obj_t)&mod_radio_esb_send_set_keep_alive_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_esb_send_keep_alive), (mp_obj_t)&mod_radio_esb_send_keep_alive_obj },
    // { MP_OBJ_NEW_QSTR(MP_QSTR_esb_send_keep_alive2), (mp_obj_t)&mod_radio_esb_send_keep_alive2_obj },

    /* ultra-light ESB mode. */
    { MP_OBJ_NEW_QSTR(MP_QSTR_esb), (mp_obj_t)&mod_radio_esb_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sb), (mp_obj_t)&mod_radio_sb_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_cx), (mp_obj_t)&mod_radio_cx_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ble), (mp_obj_t)&mod_radio_ble_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ping), (mp_obj_t)&mod_radio_ping_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_find), (mp_obj_t)&mod_radio_find_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_RATE_250KBIT), MP_OBJ_NEW_SMALL_INT(RADIO_MODE_MODE_Nrf_250Kbit) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_RATE_1MBIT), MP_OBJ_NEW_SMALL_INT(RADIO_MODE_MODE_Nrf_1Mbit) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_RATE_2MBIT), MP_OBJ_NEW_SMALL_INT(RADIO_MODE_MODE_Nrf_2Mbit) },
};

STATIC MP_DEFINE_CONST_DICT(radio_module_globals, radio_module_globals_table);

const mp_obj_module_t radio_module = {
    .base = { &mp_type_module },
    .name = MP_QSTR_radio,
    .globals = (mp_obj_dict_t*)&radio_module_globals,
};

}
