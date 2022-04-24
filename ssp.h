#pragma once
/*
 * Declarations for Simple Serial Protocol (SSP).
 * Adapted from https://github.com/HanaMostafa/SSProtocol.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <stdint.h>  // uint8_t, uint16_t.

uint16_t ssp_compute_crc16(uint8_t *data, uint8_t length);
void ssp_build_frame(uint8_t *tx_frame, uint8_t *data, uint8_t desti, uint8_t srce, uint8_t typee, uint16_t tx_size, uint8_t *tx_flag);
void ssp_deframing(uint8_t *rx_frame, uint8_t *adddest, uint8_t *addsrc, uint8_t *type, uint8_t *datta, uint16_t *size3, uint8_t *rx_flag, uint8_t *crc_flag, uint8_t *deframe_flag);
void ssp_control_layer(uint8_t *tx_app_data, uint16_t data_length, uint8_t tx_app_desti, uint8_t *tx_frm_srce, uint8_t tx_app_type, uint8_t *tx_frm_type, uint8_t *tx_frm_data, uint8_t *tx_frm_desti, uint8_t *rx_frm_type, uint8_t *rx_frm_data, uint8_t *rx_frm_dest, uint16_t rx_length, uint8_t *data_flag, uint8_t *deframe_flag, uint8_t *tx_flag, uint8_t *rx_app_data, uint8_t crc_flag, uint16_t *tx_size, uint8_t *rx_frm_src, uint8_t *layer_flag, uint8_t *check_control);
