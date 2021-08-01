#pragma once

#include <Arduino.h>

uint32_t crc32c_calculate(void *pdata, size_t nbytes);
bool test_crc32c();
