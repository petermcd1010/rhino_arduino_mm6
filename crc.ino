/*
 * Cyclical Redundancy Check (CRC) functions.
 */

// From https://www.arduino.cc/en/Tutorial/LibraryExamples/EEPROMCrc, which is based on
// https://github.com/tpircher/pycrc.
unsigned long crc_calculate(void *pdata, size_t nbytes) {
  assert(pdata);

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;
  unsigned char *pucdata = (unsigned char*)pdata;
  for (int index = 0 ; index < nbytes; ++index) {
    crc = crc_table[(crc ^ pucdata[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (pucdata[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }

  return crc;
}

#define TEST_ASSERT(test, format, args ...) do { if (!(test)) { LOG_E(format, ## args); ret = false; } } while (0)

bool test_crc_calculate() {
  bool ret = true;
  long value = 55;
  TEST_ASSERT(crc_calculate(&value, 0) == -1, F("expected crc_calculate(0) == -1"));
  
  // TODO: more.
  return ret;
}

