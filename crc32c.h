#pragma once

/*
 * Declarations for 32-bit Castagnoli Cyclical Redundancy Check (CRC32C) functionality.
 */

#include <Arduino.h>

uint32_t crc32c_calculate(void *pdata, size_t nbytes);
bool crc32c_test();
