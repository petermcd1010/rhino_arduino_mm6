#pragma once
/*
 * Declarations for Simple Serial Protocol (SSP)
 * Adapted from https://github.com/HanaMostafa/SSProtocol
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <stdint.h>  // uint8_t, uint16_t.

uint16_t ssp_compute_crc16(uint8_t *data_p, uint8_t length);
void ssp_build_frame(uint8_t *txframe, uint8_t *data, uint8_t desti, uint8_t srce, uint8_t typee, uint16_t tx_size, uint8_t *txflag);
void ssp_deframing(uint8_t *rxframe, uint8_t *adddest, uint8_t *addsrc, uint8_t *type, uint8_t *datta, uint16_t *size3, uint8_t *rxflag, uint8_t *crcflag, uint8_t *deframeflag);
void ssp_control_layer(uint8_t *Tx_App_data, uint16_t data_length, uint8_t Tx_App_desti, uint8_t *Tx_Frm_srce, uint8_t Tx_App_type, uint8_t *Tx_Frm_type, uint8_t *Tx_Frm_data, uint8_t *Tx_Frm_desti, uint8_t *Rx_Frm_type, uint8_t *Rx_Frm_data, uint8_t *Rx_Frm_dest, uint16_t Rx_length, uint8_t *dataflag, uint8_t *deframeflag, uint8_t *txflag, uint8_t *Rx_App_data, uint8_t crcflag, uint16_t *tx_size, uint8_t *Rx_Frm_src, uint8_t *layerflag, uint8_t *checkcontrol);
