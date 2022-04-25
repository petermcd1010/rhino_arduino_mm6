/*
 * Implementation for Simple Serial Protocol (SSP).
 * Adapted from https://github.com/HanaMostafa/SSProtocol.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "ssp.h"
#include "Arduino.h"

#define SSP_FEND 0
#define SSP_DT 236  // Frame size? Buffer length?
#define SSP_DEST 1
#define SSP_SRC 2
#define SSP_KIND 3
// #define SSP_HEADER 8
#define SSP_INFO 229  // Data length without framing.
#define SSP_FULL 1
#define SSP_EMPTY 0
#define SSP_TX 1
#define SSP_RX 2
#define SSP_IDLE 0

// Set a certain bit in any register.
#define SET_BIT(REG, BIT) (REG |= (1 << BIT))

// Clear a certain bit in any register.
#define CLEAR_BIT(REG, BIT) (REG &= (~(1 << BIT)))

// Toggle a certain bit in any register.
#define TOGGLE_BIT(REG, BIT) (REG ^= (1 << BIT))

// Rotate right the register value with specific number of rotates.
#define ROR(REG, num) (REG = (REG >> num) | (REG << (8 - num)))

// Rotate left the register value with specific number of rotates.
#define ROL(REG, num) (REG = (REG << num) | (REG >> (8 - num)))

// Check if a specific bit is set in any register and return true if yes.
#define BIT_IS_SET(REG, BIT) (REG & (1 << BIT))

// Check if a specific bit is cleared in any register and return true if yes.
#define BIT_IS_CLEAR(REG, BIT) (!(REG & (1 << BIT)))

uint16_t compute_crc16(uint8_t *data, uint8_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--) {
        // Reverse the bits in each 8-bit byte going in.
        *data = (*data & 0x55555555) << 1 | (*data & 0xAAAAAAAA) >> 1;
        *data = (*data & 0x33333333) << 2 | (*data & 0xCCCCCCCC) >> 2;
        *data = (*data & 0x0F0F0F0F) << 4 | (*data & 0xF0F0F0F0) >> 4;

        x = crc >> 8 ^ *data++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12))
              ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    // Reverse the 16-bit CRC.
    crc = (crc & 0x55555555) << 1 | (crc & 0xAAAAAAAA) >> 1;
    crc = (crc & 0x33333333) << 2 | (crc & 0xCCCCCCCC) >> 2;
    crc = (crc & 0x0F0F0F0F) << 4 | (crc & 0xF0F0F0F0) >> 4;
    crc = (crc & 0x00FF00FF) << 8 | (crc & 0xFF00FF00) >> 8;

    return crc;
}

static void control_layer(uint8_t *tx_app_data, uint16_t data_length, uint8_t tx_app_dest,
                          uint8_t *tx_frm_srce, uint8_t tx_app_kind, uint8_t *tx_frm_kind,
                          uint8_t *tx_frm_data, uint8_t *tx_frm_dest, uint8_t *rx_frm_kind,
                          uint8_t *rx_frm_data, uint8_t *rx_frm_dest, uint16_t rx_length,
                          uint8_t *data_flag, uint8_t *deframe_flag, uint8_t *tx_flag, uint8_t *rx_app_data,
                          uint8_t crc_flag, uint16_t *tx_size, uint8_t *rx_frm_src, uint8_t *layer_flag, uint8_t *check_control)
{
    static uint8_t control_flag = SSP_IDLE;
    static uint8_t counter = 0;
    uint8_t source = 0x05;
    uint8_t i;

    if (control_flag == SSP_IDLE) {
        if (*data_flag == SSP_FULL && *tx_flag == SSP_EMPTY) {
            Serial1.println("\n Sending  Data \n");
            Serial1.flush();

            control_flag = SSP_TX;
            *tx_frm_srce = source;

            for (i = 0; i < data_length; i++) {
                tx_frm_data[i] = tx_app_data[i];
            }

            *tx_size = data_length;
            *tx_frm_dest = tx_app_dest;
            *tx_frm_kind = tx_app_kind;
            *tx_flag = SSP_FULL;
            *data_flag = SSP_EMPTY;
        } else if (*deframe_flag == SSP_FULL && *layer_flag == SSP_EMPTY) {
            if (*rx_frm_dest == source) {
                Serial1.print("\n Received Data \n");
                Serial1.flush();
                control_flag = SSP_RX;
                *layer_flag = SSP_FULL;
                *deframe_flag = SSP_EMPTY;

                for (i = 0; i < rx_length; i++) {
                    rx_app_data[i] = rx_frm_data[i];
                }
            } else {
                *deframe_flag = SSP_EMPTY;
            }
        }
    } else if (control_flag == SSP_TX) {
        if (*deframe_flag == SSP_FULL) {
            if (*rx_frm_dest == source && *rx_frm_kind == 0x02) {
                Serial1.println("\n Respond with an ACK \n");
                Serial1.flush();
                control_flag = SSP_IDLE;
                *deframe_flag = SSP_EMPTY;
                *check_control = SSP_EMPTY;
                counter = 0;
            } else if ((*rx_frm_dest == source)
                       && (*rx_frm_kind == 0x03 || *rx_frm_kind == 0x13
                           || *rx_frm_kind == 0x23)) {
                Serial1.println("\n Response with NACK \n");
                Serial1.println("\n Sending Data Again \n");
                Serial1.flush();
                for (i = 0; i < data_length; i++) {
                    tx_frm_data[i] = tx_app_data[i];
                }
                *tx_size = data_length;
                *tx_flag = SSP_FULL;
                *deframe_flag = SSP_EMPTY;
                *check_control = SSP_FULL;
                counter++;
                if (counter == 3) {
                    Serial1.println("\n NACK Counter= 3 \n");
                    Serial1.flush();
                    control_flag = SSP_IDLE;
                    counter = 0;
                    *tx_flag = SSP_EMPTY;
                    *check_control = SSP_EMPTY;
                }
            } else if (*rx_frm_dest != source) {
                *deframe_flag = SSP_EMPTY;
            }
        }
    } else if (control_flag == SSP_RX) {
        if (crc_flag == SSP_EMPTY) {
            Serial1.println("\n Correct CRC \n");
            Serial1.flush();
            *tx_frm_srce = *rx_frm_dest;
            *tx_size = 0;

            *tx_frm_dest = *rx_frm_src;
            *tx_frm_kind = 0x02;
            *tx_flag = SSP_FULL;
            control_flag = SSP_IDLE;
            *layer_flag = SSP_FULL;
        } else if (crc_flag == SSP_FULL) {
            Serial1.println("\n Wrong CRC \n");
            Serial1.flush();
            *tx_frm_srce = *rx_frm_dest;
            *tx_size = 0;
            *tx_frm_dest = *rx_frm_src;
            *tx_frm_kind = 0x03;
            *tx_flag = SSP_FULL;
            control_flag = SSP_IDLE;
        }
    }
}

void ssp_build_frame(uint8_t *tx_frame, uint8_t *data, uint8_t dest, uint8_t srce,
                     uint8_t kinde, uint16_t tx_size, uint8_t *tx_flag)
{
    uint16_t p, k;

    tx_frame[SSP_FEND] = 0xc0;
    tx_frame[SSP_DEST] = dest;
    tx_frame[SSP_SRC] = srce;
    tx_frame[SSP_KIND] = kinde;

    uint8_t f, d, count = 0, w = 0, count2 = 0, arr[SSP_DT];
    int temp = 0, temp2 = 0;
    uint16_t crc, crc0, crc1;

    for (k = 0; k < tx_size; k++) {
        if (data[k] == 0xc0)
            count++;
    }
    for (k = 0; k < tx_size; k++) {
        if (data[k] == 0xdb)
            count2++;
    }
    temp = tx_size + count;
    temp2 = tx_size + count2;

    for (k = 0; k <= temp; k++) {
        if (data[k] == 0xc0) {
            data[k] = 0xdb;

            for (f = temp; f >= (k + 1); f--) {
                data[f] = data[f - 1];
            }
            data[k + 1] = 0xdc;
        } else if (data[k] == 0xdb) {
            data[k] = 0xdb;
            for (f = temp2; f >= (k + 1); f--) {
                data[f] = data[f - 1];
            }

            data[k + 1] = 0xdd;
        }
    }

    w = temp + count2 + 4;

    for (p = 4; p < (w); p++) {
        tx_frame[p] = data[p - 4];
    }
    for (d = 1; d < w; d++) {
        arr[d - 1] = tx_frame[d];
    }
    crc = compute_crc16(arr, (w - 1));
    crc0 = (crc & 0x00ff);
    crc1 = ((crc & 0xff00) >> 8);
    tx_frame[w] = crc0;                // crc0
    tx_frame[(w += 1)] = crc1;         //crc1
    tx_frame[(w + 1)] = 0xc0;

    uint8_t countttt = 1, j, i;

    for (j = 1; j < SSP_DT; j++) {
        if (tx_frame[j] == 0xc0) {
            countttt++;
            break;
        } else {
            countttt++;
        }
    }

    for (i = 0; i < countttt; i++) {
        Serial.write(tx_frame[i]);
    }

    *tx_flag = SSP_EMPTY;
}

void ssp_deframing(uint8_t *rx_frame, uint8_t *dest, uint8_t *src, uint8_t *kind,
                   uint8_t *rx_data, uint16_t *length, uint8_t *rx_flag, uint8_t *crc_flag, uint8_t *deframe_flag)
{
    uint16_t i, j, d, size2, size = 1, crc, size3;
    uint8_t count = 0, k, y = 0, arr[SSP_DT], datta[SSP_INFO + 4];

    *dest = rx_frame[SSP_DEST];
    *src = rx_frame[SSP_SRC];
    *kind = rx_frame[SSP_KIND];

    for (j = 1; j < SSP_DT; j++) {
        if (rx_frame[j] == 0xc0) {
            size++;
            break;
        } else {
            size++;
        }
    }

    for (d = 1; d < size - 1; d++) {
        arr[y] = rx_frame[d];

        y++;
    }

    crc = compute_crc16(arr, y);

    size2 = size - 3;

    if (rx_frame[SSP_FEND] == 0xc0 && crc == 0x00) {
        *dest = rx_frame[SSP_DEST];
        *src = rx_frame[SSP_SRC];

        for (i = 4; i < (size - 3); i++) {
            datta[i] = rx_frame[i];
        }
        for (i = 4; i < (size - 3); i++) {
            if ((datta[i] == 0xdb) && (datta[i + 1] == 0xdc))
                size2--;
        }

        for (i = 4; i < (size - 3); i++) {
            if ((datta[i] == 0xdb) && (datta[i + 1] == 0xdd))
                count++;
        }

        for (i = 4; i < (size - 3); i++) {
            if ((datta[i] == 0xdb) && (datta[i + 1] == 0xdc)) {
                datta[i] = 0xc0;
                for (k = i + 1; k < size - 3; k++) {
                    datta[k] = datta[k + 1];
                }
            }
        }

        for (i = 4; i < (size - 3); i++) {
            if ((datta[i] == 0xdb) && (datta[i + 1] == 0xdd)) {
                datta[i] = 0xdb;
                for (k = i + 1; k < size - 3; k++) {
                    datta[k] = datta[k + 1];
                }
            }
        }
        size3 = size2 - count;
        *length = size3 - 4;

        for (i = 0; i < (*length); i++) {
            rx_data[i] = datta[i + 4];
        }
        *crc_flag = SSP_EMPTY;
    } else {
        *crc_flag = SSP_FULL;
    }

    *rx_flag = SSP_EMPTY;
    *deframe_flag = SSP_FULL;
}

// From main.

#include "Arduino.h"
#include "ssp.h"

static uint8_t tx_frame[SSP_DT];
static uint8_t data[SSP_INFO];
static uint8_t data2[SSP_INFO];
static uint8_t rx_frame[SSP_DT];
static uint8_t rx_data[SSP_INFO];
static uint8_t layer_data[SSP_INFO];

static uint8_t kind2;

static uint8_t check_control = SSP_EMPTY;
static uint16_t data_length;
static uint8_t data_flag = SSP_EMPTY;
static uint8_t rx_flag = SSP_EMPTY;


static void serial_flush_buffer()
{
    while (Serial.read() >= 0) {
        ;                              // do nothing
    }
}

static void receive_frame_here()
{
    if (rx_flag == SSP_EMPTY) {
        if (Serial.available() > 0) {
            Serial.readBytes(rx_frame, 236);
            serial_flush_buffer();
            rx_flag = SSP_FULL;
            Serial1.println("\n Received frame\n");
            Serial1.flush();
        }
    }
}

static void generate_random_data(uint8_t *data, uint16_t *data_length, uint8_t *data_flag)
{
    *data_flag = SSP_EMPTY;
    uint8_t i;
    uint8_t arr[SSP_INFO];

    *data_length = random(0, 229);
    for (i = 0; i < *data_length; i++) {
        arr[i] = random(0x00, 0xff);
    }

    for (i = 0; i < *data_length; i++) {
        data[i] = arr[i];
    }

    *data_flag = SSP_FULL;
}

static void ssp_setup()
{
    Serial.begin(9600);
    Serial1.begin(9600);
    if (check_control == SSP_EMPTY) {
        generate_random_data(data, &data_length, &data_flag);
        check_control = SSP_FULL;
    }
}

static void ssp_loop()
{
    static uint8_t tx_flag = SSP_EMPTY;
    uint8_t i;
    static uint8_t crc_flag = SSP_EMPTY;
    uint8_t dest = 0x01;
    uint8_t srce;
    uint8_t kinde = 0x02;
    uint8_t dest2;
    static uint16_t tx_size = 0;
    uint8_t adddest;
    uint8_t addsrc;
    static uint8_t kind = 0;
    static uint16_t rx_length = 0;
    static uint8_t layer_flag = SSP_EMPTY;
    static uint8_t deframe_flag = SSP_EMPTY;
    static uint8_t framingflag = SSP_EMPTY;

    delay(100);

    if (check_control == SSP_EMPTY) {
        generate_random_data(data, &data_length, &data_flag);
        check_control = SSP_FULL;
    }

    if ((check_control == SSP_FULL && tx_flag == SSP_EMPTY)
        || (check_control == SSP_EMPTY && layer_flag == SSP_EMPTY)) {
        control_layer(data, data_length, dest, &srce, kinde, &kind2, data2, &dest2,
                      &kind, rx_data, &adddest, rx_length, &data_flag, &deframe_flag,
                      &tx_flag, layer_data, crc_flag, &tx_size, &addsrc, &layer_flag, &check_control);

        layer_flag = SSP_EMPTY;
    }

    if (tx_flag == SSP_FULL)
        ssp_build_frame(tx_frame, data2, dest2, srce, kind2, tx_size, &tx_flag);

    receive_frame_here();

    if (rx_flag == SSP_FULL && deframe_flag == SSP_EMPTY)
        ssp_deframing(rx_frame, &adddest, &addsrc, &kind, rx_data, &rx_length,
                      &rx_flag, &crc_flag, &deframe_flag);
}
