/*
 * Implementation for Simple Serial Protocol (SSP)
 * Adapted from https://github.com/HanaMostafa/SSProtocol
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "ssp.h"
#include "Arduino.h"

#define SSP_FEND 0
#define SSP_DT 236
#define SSP_DEST 1
#define SSP_SRC 2
#define SSP_TYPE 3
// #define SSP_HEADER 8
#define SSP_INFO 229
#define SSP_FULL 1
#define SSP_EMPTY 0
#define SSP_TX 1
#define SSP_RX 2
#define SSP_IDLE 0

/* Set a certain bit in any register */
#define SET_BIT(REG, BIT) (REG |= (1 << BIT))

/* Clear a certain bit in any register */
#define CLEAR_BIT(REG, BIT) (REG &= (~(1 << BIT)))

/* Toggle a certain bit in any register */
#define TOGGLE_BIT(REG, BIT) (REG ^= (1 << BIT))

/* Rotate right the register value with specific number of rotates */
#define ROR(REG, num) (REG = (REG >> num) | (REG << (8 - num)))

/* Rotate left the register value with specific number of rotates */
#define ROL(REG, num) (REG = (REG << num) | (REG >> (8 - num)))

/* Check if a specific bit is set in any register and return true if yes */
#define BIT_IS_SET(REG, BIT) (REG & (1 << BIT))

/* Check if a specific bit is cleared in any register and return true if yes */
#define BIT_IS_CLEAR(REG, BIT) (!(REG & (1 << BIT)))

unsigned short compute_crc16(unsigned char *data_p, unsigned char length)
{
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--) {
        /*  reverse the bits in each 8-bit byte going in */
        *data_p = (*data_p & 0x55555555) << 1 | (*data_p & 0xAAAAAAAA) >> 1;
        *data_p = (*data_p & 0x33333333) << 2 | (*data_p & 0xCCCCCCCC) >> 2;
        *data_p = (*data_p & 0x0F0F0F0F) << 4 | (*data_p & 0xF0F0F0F0) >> 4;

        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12))
              ^ ((unsigned short)(x << 5)) ^ ((unsigned short)x);
    }
    /*reverse the 16-bit CRC*/
    crc = (crc & 0x55555555) << 1 | (crc & 0xAAAAAAAA) >> 1;
    crc = (crc & 0x33333333) << 2 | (crc & 0xCCCCCCCC) >> 2;
    crc = (crc & 0x0F0F0F0F) << 4 | (crc & 0xF0F0F0F0) >> 4;
    crc = (crc & 0x00FF00FF) << 8 | (crc & 0xFF00FF00) >> 8;

    return crc;
}

static void getdata(uint8_t *data, uint16_t *data_length, uint8_t *dataflag)
{
    *dataflag = SSP_EMPTY;
    uint8_t i;
    uint8_t arr[SSP_INFO];


    *data_length = random(0, 229);
    for (i = 0; i < *data_length; i++) {
        arr[i] = random(0x00, 0xff);
    }

    for (i = 0; i < *data_length; i++) {
        data[i] = arr[i];
    }

    *dataflag = SSP_FULL;
}

static void control_layer(uint8_t *Tx_App_data, uint16_t data_length, uint8_t Tx_App_desti,
                          uint8_t *Tx_Frm_srce, uint8_t Tx_App_type, uint8_t *Tx_Frm_type,
                          uint8_t *Tx_Frm_data, uint8_t *Tx_Frm_desti, uint8_t *Rx_Frm_type,
                          uint8_t *Rx_Frm_data, uint8_t *Rx_Frm_dest, uint16_t Rx_length,
                          uint8_t *dataflag, uint8_t *deframeflag, uint8_t *txflag, uint8_t *Rx_App_data,
                          uint8_t crcflag, uint16_t *tx_size, uint8_t *Rx_Frm_src, uint8_t *layerflag, uint8_t *checkcontrol)
{
    static uint8_t controlflag = SSP_IDLE;
    static uint8_t counter = 0;
    uint8_t source = 0x05;
    uint8_t i;

    if (controlflag == SSP_IDLE) {
        if (*dataflag == SSP_FULL && *txflag == SSP_EMPTY) {
            Serial1.println("\n Sending  Data \n");
            Serial1.flush();

            controlflag = SSP_TX;
            *Tx_Frm_srce = source;

            for (i = 0; i < data_length; i++) {
                Tx_Frm_data[i] = Tx_App_data[i];
            }

            *tx_size = data_length;
            *Tx_Frm_desti = Tx_App_desti;
            *Tx_Frm_type = Tx_App_type;
            *txflag = SSP_FULL;
            *dataflag = SSP_EMPTY;
        } else if (*deframeflag == SSP_FULL && *layerflag == SSP_EMPTY) {
            if (*Rx_Frm_dest == source) {
                Serial1.print("\n Received Data \n");
                Serial1.flush();
                controlflag = SSP_RX;
                *layerflag = SSP_FULL;
                *deframeflag = SSP_EMPTY;

                for (i = 0; i < Rx_length; i++) {
                    Rx_App_data[i] = Rx_Frm_data[i];
                }
            } else {
                *deframeflag = SSP_EMPTY;
            }
        }
    } else if (controlflag == SSP_TX) {
        if (*deframeflag == SSP_FULL) {
            if (*Rx_Frm_dest == source && *Rx_Frm_type == 0x02) {
                Serial1.println("\n Respond with an ACK \n");
                Serial1.flush();
                controlflag = SSP_IDLE;
                *deframeflag = SSP_EMPTY;
                *checkcontrol = SSP_EMPTY;
                counter = 0;
            } else if ((*Rx_Frm_dest == source)
                       && (*Rx_Frm_type == 0x03 || *Rx_Frm_type == 0x13
                           || *Rx_Frm_type == 0x23)) {
                Serial1.println("\n Response with NACK \n");
                Serial1.println("\n Sending Data Again \n");
                Serial1.flush();
                for (i = 0; i < data_length; i++) {
                    Tx_Frm_data[i] = Tx_App_data[i];
                }
                *tx_size = data_length;
                *txflag = SSP_FULL;
                *deframeflag = SSP_EMPTY;
                *checkcontrol = SSP_FULL;
                counter++;
                if (counter == 3) {
                    Serial1.println("\n NACK Counter= 3 \n");
                    Serial1.flush();
                    controlflag = SSP_IDLE;
                    counter = 0;
                    *txflag = SSP_EMPTY;
                    *checkcontrol = SSP_EMPTY;
                }
            } else if (*Rx_Frm_dest != source) {
                *deframeflag = SSP_EMPTY;
            }
        }
    } else if (controlflag == SSP_RX) {
        if (crcflag == SSP_EMPTY) {
            Serial1.println("\n Correct CRC \n");
            Serial1.flush();
            *Tx_Frm_srce = *Rx_Frm_dest;
            *tx_size = 0;

            *Tx_Frm_desti = *Rx_Frm_src;
            *Tx_Frm_type = 0x02;
            *txflag = SSP_FULL;
            controlflag = SSP_IDLE;
            *layerflag = SSP_FULL;
        } else if (crcflag == SSP_FULL) {
            Serial1.println("\n Wrong CRC \n");
            Serial1.flush();
            *Tx_Frm_srce = *Rx_Frm_dest;
            *tx_size = 0;
            *Tx_Frm_desti = *Rx_Frm_src;
            *Tx_Frm_type = 0x03;
            *txflag = SSP_FULL;
            controlflag = SSP_IDLE;
        }
    }
}

void ssp_build_frame(uint8_t *txframe, uint8_t *data, uint8_t desti, uint8_t srce,
                     uint8_t typee, uint16_t tx_size, uint8_t *txflag)
{
    uint16_t p, k;

    txframe[SSP_FEND] = 0xc0;
    txframe[SSP_DEST] = desti;
    txframe[SSP_SRC] = srce;
    txframe[SSP_TYPE] = typee;

    uint8_t f, d, count = 0, w = 0, count2 = 0, arr[SSP_DT];
    int temp = 0, temp2 = 0;
    uint16_t crc, crc0, crc1;

    for (k = 0; k < tx_size; k++) {
        if (data[k] == 0xc0) {
            count++;
        }
    }
    for (k = 0; k < tx_size; k++) {
        if (data[k] == 0xdb) {
            count2++;
        }
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
        txframe[p] = data[p - 4];
    }
    for (d = 1; d < w; d++) {
        arr[d - 1] = txframe[d];
    }
    crc = compute_crc16(arr, (w - 1));
    crc0 = (crc & 0x00ff);
    crc1 = ((crc & 0xff00) >> 8);
    txframe[w] = crc0;                 // crc0
    txframe[(w += 1)] = crc1;          //crc1
    txframe[(w + 1)] = 0xc0;

    uint8_t countttt = 1, j, i;

    for (j = 1; j < SSP_DT; j++) {
        if (txframe[j] == 0xc0) {
            countttt++;
            break;
        } else {
            countttt++;
        }
    }

    for (i = 0; i < countttt; i++) {
        Serial.write(txframe[i]);
    }

    *txflag = SSP_EMPTY;
}

void ssp_deframing(uint8_t *rxframe, uint8_t *adddest, uint8_t *addsrc, uint8_t *type,
                   uint8_t *Rx_data, uint16_t *length, uint8_t *rxflag, uint8_t *crcflag, uint8_t *deframeflag)
{
    uint16_t i, j, d, size2, size = 1, crc, size3;
    uint8_t count = 0, k, y = 0, arr[SSP_DT], datta[SSP_INFO + 4];

    *adddest = rxframe[SSP_DEST];
    *addsrc = rxframe[SSP_SRC];
    *type = rxframe[SSP_TYPE];

    for (j = 1; j < SSP_DT; j++) {
        if (rxframe[j] == 0xc0) {
            size++;
            break;
        } else {
            size++;
        }
    }

    for (d = 1; d < size - 1; d++) {
        arr[y] = rxframe[d];

        y++;
    }

    crc = compute_crc16(arr, y);

    size2 = size - 3;

    if (rxframe[SSP_FEND] == 0xc0 && crc == 0x00) {
        *adddest = rxframe[SSP_DEST];
        *addsrc = rxframe[SSP_SRC];

        for (i = 4; i < (size - 3); i++) {
            datta[i] = rxframe[i];
        }
        for (i = 4; i < (size - 3); i++) {
            if ((datta[i] == 0xdb) && (datta[i + 1] == 0xdc)) {
                size2--;
            }
        }

        for (i = 4; i < (size - 3); i++) {
            if ((datta[i] == 0xdb) && (datta[i + 1] == 0xdd)) {
                count++;
            }
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
            Rx_data[i] = datta[i + 4];
        }
        *crcflag = SSP_EMPTY;
    } else {
        *crcflag = SSP_FULL;
    }

    *rxflag = SSP_EMPTY;
    *deframeflag = SSP_FULL;
}


#include "Arduino.h"
#include "ssp.h"

static uint8_t txframe[SSP_DT];
static uint8_t data[SSP_INFO];
static uint8_t data2[SSP_INFO];
static uint8_t rxframe[SSP_DT];
static uint8_t Rx_data[SSP_INFO];
static uint8_t layerdata[SSP_INFO];

static uint8_t type2;

static uint8_t checkcontrol = SSP_EMPTY;
static uint16_t data_length;
static uint8_t dataflag = SSP_EMPTY;
static uint8_t rxflag = SSP_EMPTY;


static void serial_flush_buffer()
{
    while (Serial.read() >= 0) {
        ;                     // do nothing
    }
}

static void receive_frame_here()
{
    if (rxflag == SSP_EMPTY) {
        if (Serial.available() > 0) {
            Serial.readBytes(rxframe, 236);
            serial_flush_buffer();
            rxflag = SSP_FULL;
            Serial1.println("\n Received frame\n");
            Serial1.flush();
        }
    }
}

static void ssp_setup()
{
    Serial.begin(9600);
    Serial1.begin(9600);
    if (checkcontrol == SSP_EMPTY) {
        getdata(data, &data_length, &dataflag);
        checkcontrol = SSP_FULL;
    }
}

static void ssp_loop()
{
    static uint8_t txflag = SSP_EMPTY;
    uint8_t i;
    static uint8_t crcflag = SSP_EMPTY;
    uint8_t desti = 0x01;
    uint8_t srce;
    uint8_t typee = 0x02;
    uint8_t desti2;
    static uint16_t tx_size = 0;
    uint8_t adddest;
    uint8_t addsrc;
    static uint8_t type = 0;
    static uint16_t Rx_length = 0;
    static uint8_t layerflag = SSP_EMPTY;
    static uint8_t deframeflag = SSP_EMPTY;
    static uint8_t framingflag = SSP_EMPTY;




    delay(100);

    if (checkcontrol == SSP_EMPTY) {
        getdata(data, &data_length, &dataflag);
        checkcontrol = SSP_FULL;
    }

    if ((checkcontrol == SSP_FULL && txflag == SSP_EMPTY)
        || (checkcontrol == SSP_EMPTY && layerflag == SSP_EMPTY)) {
        control_layer(data, data_length, desti, &srce, typee, &type2, data2, &desti2,
                      &type, Rx_data, &adddest, Rx_length, &dataflag, &deframeflag,
                      &txflag, layerdata, crcflag, &tx_size, &addsrc, &layerflag, &checkcontrol);

        layerflag = SSP_EMPTY;
    }


    if (txflag == SSP_FULL)
        ssp_build_frame(txframe, data2, desti2, srce, type2, tx_size, &txflag);

    receive_frame_here();


    if (rxflag == SSP_FULL && deframeflag == SSP_EMPTY) {
        ssp_deframing(rxframe, &adddest, &addsrc, &type, Rx_data, &Rx_length,
                      &rxflag, &crcflag, &deframeflag);
    }
}
