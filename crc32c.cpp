/*
 * Implementation for 32-bit Castagnoli Cyclical Redundancy Check (CRC32C) functionality.
 * From https://stackoverflow.com/questions/27939882/fast-crc-algorithm.
 */

#include <stdlib.h>
#include "crc32c.h"
#include "log.h"

// CRC-32C (iSCSI) polynomial in reversed bit order.
#define POLY 0x82f63b78

// CRC-32 (Ethernet, ZIP, etc.) polynomial in reversed bit order.
// #define POLY 0xedb88320

uint32_t crc32c_calculate(void *pdata, size_t nbytes)
{
  int k;
  const uint32_t initial_crc = 0;
  unsigned char *pucdata = (unsigned char*)pdata;

  uint32_t crc = ~initial_crc;
  while (nbytes--) {
      crc ^= *pucdata++;
      for (k = 0; k < 8; k++)
          crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
  }
  return ~crc;
}

#define TEST_ASSERT(test, format, args ...) do { if (!(test)) { LOG_E(format, ## args); ret = false; } } while (0)

bool crc32c_test() {
  bool ret = true;
  long value = 55;
  uint32_t crc32c;

  const uint32_t empty_crc32 = 0;
  crc32c = crc32c_calculate((void*)0, 0);
  TEST_ASSERT(crc32c == empty_crc32, 
      F("expected crc_calculate(_, 0) == 0x%lx, got 0x%0lx"), 
      empty_crc32, crc32c);

  // Test vectors from iSCSI protocol spec section A.4 'CRC Examples' at 
  // https://datatracker.ietf.org/doc/html/rfc7143.

  // TODO: Should these arrays be in PROGMEM?
  uint8_t const zeroes[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0,
  };
  const uint32_t zeroes_crc32c = 0x8a9136aa;

  crc32c = crc32c_calculate(zeroes, 32);
  TEST_ASSERT(crc32c == zeroes_crc32c, 
      F("expected crc_calculate(zeroes, 32) == 0x%lx, got 0x%0lx"), 
      zeroes_crc32c, crc32c);

  uint8_t const ones[32] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
  };
  const uint32_t ones_crc32c = 0x62a8ab43;

  crc32c = crc32c_calculate(ones, 32);
  TEST_ASSERT(crc32c == ones_crc32c, 
      F("expected crc_calculate(ones, 32) == 0x%lx, got 0x%0lx"), 
      ones_crc32c, crc32c);

  uint8_t const increments[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
  };
  const uint32_t increments_crc32c = 0x46dd794e;

  crc32c = crc32c_calculate(increments, 32);
  TEST_ASSERT(crc32c == increments_crc32c, 
      F("expected crc_calculate(increments, 32) == 0x%lx, got 0x%0lx"), 
      increments_crc32c, crc32c);

  uint8_t const decrements[32] = {
    0x1f, 0x1e, 0x1d, 0x1c, 0x1b, 0x1a, 0x19, 0x18,
    0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10,
    0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08,
    0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
  };
  const uint32_t decrements_crc32c = 0x113fdb5c;

  crc32c = crc32c_calculate(decrements, 32);

  TEST_ASSERT(crc32c == decrements_crc32c, 
      F("expected crc_calculate(decrements, 32) == 0x%lx, got 0x%0lx"), 
      decrements_crc32c, crc32c);

  return ret;
}

