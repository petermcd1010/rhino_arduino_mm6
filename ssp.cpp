#pragma once
/*
 * Implementation for Simple Serial Protocol (SSP)
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "ssp.h"
#include "Arduino.h"

void serial_flush_buffer();

unsigned short compute_crc16(unsigned char *data_p, unsigned char length) {
  unsigned char x;
  unsigned short crc = 0xFFFF;

  while (length--) {
    /*  reverse the bits in each 8-bit byte going in */
    *data_p = (*data_p & 0x55555555) << 1 | (*data_p & 0xAAAAAAAA) >> 1;
    *data_p = (*data_p & 0x33333333) << 2 | (*data_p & 0xCCCCCCCC) >> 2;
    *data_p = (*data_p & 0x0F0F0F0F) << 4 | (*data_p & 0xF0F0F0F0) >> 4;

    x = crc >> 8 ^ *data_p++;
    x ^= x >> 4;
    crc = (crc << 8) ^ ((unsigned short) (x << 12))
          ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
  }
  /*reverse the 16-bit CRC*/
  crc = (crc & 0x55555555) << 1 | (crc & 0xAAAAAAAA) >> 1;
  crc = (crc & 0x33333333) << 2 | (crc & 0xCCCCCCCC) >> 2;
  crc = (crc & 0x0F0F0F0F) << 4 | (crc & 0xF0F0F0F0) >> 4;
  crc = (crc & 0x00FF00FF) << 8 | (crc & 0xFF00FF00) >> 8;

  return crc;
}

void getdata(uint8 *data, uint16 *data_length, uint8 *dataflag) {
  *dataflag = EMPTY;
  uint8 i;
  uint8 arr[info];


  *data_length = random(0, 229);
  for (i = 0; i < *data_length; i++) {
    arr[i] = random(0x00, 0xff);
  }

  for (i = 0; i < *data_length; i++) {
    data[i] = arr[i];

  }

  *dataflag = FULL;
}

void control_layer(uint8 *Tx_App_data, uint16 data_length, uint8 Tx_App_desti,
                   uint8 *Tx_Frm_srce, uint8 Tx_App_type, uint8 *Tx_Frm_type,
                   uint8 *Tx_Frm_data, uint8 *Tx_Frm_desti, uint8 *Rx_Frm_type,
                   uint8 *Rx_Frm_data, uint8 *Rx_Frm_dest, uint16 Rx_length,
                   uint8 *dataflag, uint8 *deframeflag, uint8 *txflag, uint8 *Rx_App_data,
                   uint8 crcflag, uint16 *tx_size, uint8 *Rx_Frm_src, uint8 *layerflag, uint8 *checkcontrol) {


  static uint8 controlflag = idle;
  static uint8 counter = 0;
  uint8 source = 0x05;
  uint8 i;

  if (controlflag == idle) {
    if (*dataflag == FULL  && *txflag == EMPTY) {
      Serial1.println("\n Sending  Data \n");
      Serial1.flush();

      controlflag = tx;
      *Tx_Frm_srce = source;

      for (i = 0; i < data_length; i++) {
        Tx_Frm_data[i] = Tx_App_data[i];
      }

      *tx_size = data_length;
      *Tx_Frm_desti = Tx_App_desti;
      *Tx_Frm_type = Tx_App_type;
      *txflag = FULL;
      *dataflag = EMPTY;


    } else if (*deframeflag == FULL && *layerflag == EMPTY ) {

      if (*Rx_Frm_dest == source) {
        Serial1.print("\n Received Data \n");
        Serial1.flush();
        controlflag = rx;
        *layerflag = FULL;
        *deframeflag = EMPTY;

        for (i = 0; i < Rx_length; i++) {
          Rx_App_data[i] = Rx_Frm_data[i];
        }
      } else {

        *deframeflag = EMPTY;
      }
    }

  }


  else if (controlflag == tx) {

    if (*deframeflag == FULL) {

      if (*Rx_Frm_dest == source && *Rx_Frm_type == 0x02) {
        Serial1.println("\n Respond with an ACK \n");
        Serial1.flush();
        controlflag = idle;
        *deframeflag = EMPTY;
        *checkcontrol = EMPTY;
        counter = 0;
      }

      else if ((*Rx_Frm_dest == source)
               && (*Rx_Frm_type == 0x03 || *Rx_Frm_type == 0x13
                   || *Rx_Frm_type == 0x23)) {
        Serial1.println("\n Response with NACK \n");
        Serial1.println("\n Sending Data Again \n");
        Serial1.flush();
        for (i = 0; i < data_length; i++) {
          Tx_Frm_data[i] = Tx_App_data[i];
        }
        *tx_size = data_length;
        *txflag = FULL;
        *deframeflag = EMPTY;
        *checkcontrol = FULL;
        counter++;
        if (counter == 3)
        {
          Serial1.println("\n NACK Counter= 3 \n");
          Serial1.flush();
          controlflag = idle;
          counter = 0;
          *txflag = EMPTY;
          *checkcontrol = EMPTY;
        }
      } else if (*Rx_Frm_dest != source) {
        *deframeflag = EMPTY;


      }
    }
  }
  else if (controlflag == rx) {

    if (crcflag == EMPTY) {
      Serial1.println("\n Correct CRC \n");
      Serial1.flush();
      *Tx_Frm_srce = *Rx_Frm_dest;
      *tx_size = 0;

      *Tx_Frm_desti = *Rx_Frm_src;
      *Tx_Frm_type = 0x02;
      *txflag = FULL;
      controlflag = idle;
      *layerflag = FULL;
    } else if (crcflag == FULL) {
      Serial1.println("\n Wrong CRC \n");
      Serial1.flush();
      *Tx_Frm_srce = *Rx_Frm_dest;
      *tx_size = 0;
      *Tx_Frm_desti = *Rx_Frm_src;
      *Tx_Frm_type = 0x03;
      *txflag = FULL;
      controlflag = idle;
    }
  }
}

void ssp_build_frame(uint8 *txframe, uint8 *data, uint8 desti, uint8 srce,
                     uint8 typee, uint16 tx_size, uint8 *txflag) {

  uint16 p, k;

  txframe[fend] = 0xc0;

  txframe[DEST] = desti;

  txframe[SRC] = srce;


  txframe[typ] = typee;

  uint8 f, d, count = 0, w = 0, count2 = 0, arr[dt];
  int temp = 0, temp2 = 0;
  uint16 crc, crc0, crc1;
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
  txframe[w] = crc0;                    // crc0
  txframe[(w += 1)] = crc1;                //crc1
  txframe[(w + 1)] = 0xc0;

  uint8 countttt = 1, j, i;
  for (j = 1; j < dt; j++) {

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

  *txflag = EMPTY;
}

void ssp_deframing(uint8 *rxframe, uint8 *adddest, uint8 *addsrc, uint8 *type,
                   uint8 *Rx_data, uint16 *length, uint8 *rxflag, uint8 *crcflag, uint8 *deframeflag) {
  uint16 i, j, d, size2, size = 1, crc, size3;
  uint8 count = 0, k, y = 0, arr[dt], datta[info + 4];

  *adddest = rxframe[DEST];
  *addsrc = rxframe[SRC];
  *type = rxframe[typ];

  for (j = 1; j < dt; j++) {

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

  if (rxframe[fend] == 0xc0 && crc == 0x00) {
    *adddest = rxframe[DEST];
    *addsrc = rxframe[SRC];

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
    *crcflag = EMPTY;

  } else {
    *crcflag = FULL;
  }

  *rxflag = EMPTY;
  *deframeflag = FULL;
}


#include "Arduino.h"
#include "ssp.h"

uint8 txframe[dt];
uint8 data[info];
uint8 data2[info];
uint8 rxframe[dt];
uint8 Rx_data[info];
uint8 layerdata[info];

uint8 type2;

uint8 checkcontrol = EMPTY;
uint16 data_length;
uint8 dataflag = EMPTY;
uint8 rxflag = EMPTY;
void receive_frame_here() {
  if (rxflag == EMPTY) {
    if (Serial.available() > 0) {
      Serial.readBytes(rxframe, 236);
      serial_flush_buffer();
      rxflag = FULL;
      Serial1.println("\n Received frame\n");
      Serial1.flush();
    }
  }
}

void serial_flush_buffer()
{
  while (Serial.read() >= 0); // do nothing
}

void ssp_setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  if (checkcontrol == EMPTY) {
    getdata(data, &data_length, &dataflag);
    checkcontrol = FULL;
  }

}

void ssp_loop() {

  static uint8 txflag = EMPTY;
  uint8 i;
  static uint8 crcflag = EMPTY;
  uint8 desti = 0x01;
  uint8 srce;
  uint8 typee = 0x02;
  uint8 desti2;
  static uint16 tx_size = 0;
  uint8 adddest;
  uint8 addsrc;
  static uint8 type = 0;
  static uint16 Rx_length = 0;
  static uint8 layerflag = EMPTY;
  static uint8 deframeflag = EMPTY;
  static uint8 framingflag = EMPTY;




  delay(100);

  if (checkcontrol == EMPTY) {
    getdata(data, &data_length, &dataflag);
    checkcontrol = FULL;
  }

  if ((checkcontrol == FULL && txflag == EMPTY)
      || (checkcontrol == EMPTY && layerflag == EMPTY)) {
    control_layer(data, data_length, desti, &srce, typee, &type2, data2, &desti2,
                  &type, Rx_data, &adddest, Rx_length, &dataflag, &deframeflag,
                  &txflag, layerdata, crcflag, &tx_size, &addsrc, &layerflag, &checkcontrol);

    layerflag = EMPTY;

  }


  if (txflag == FULL) {
    ssp_build_frame(txframe, data2, desti2, srce, type2, tx_size, &txflag);
  }

  receive_frame_here();


  if (rxflag == FULL && deframeflag == EMPTY ) {

    ssp_deframing(rxframe, &adddest, &addsrc, &type, Rx_data, &Rx_length,
                  &rxflag, &crcflag, &deframeflag);

  }



}

