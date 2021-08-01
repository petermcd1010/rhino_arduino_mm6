/*
 * Time functions.
 * References:
 *   https://arduino.stackexchange.com/questions/12587/how-can-i-handle-the-millis-rollover/12588#12588 
 */

unsigned long time_micros_since(unsigned long previous_micros)
{
  return micros() - previous_micros;
}

unsigned long time_millis_since(unsigned long previous_millis)
{
  return millis() - previous_millis;
}

#define TEST_ASSERT(test, format, args ...) do { if (!(test)) { LOG_E(format, ## args); ret = false; } } while (0)

bool test_time()
{
  bool ret = true;

#if 0
  extern unsigned long timer0_millis;
  timer0_millis = 0;

  extern unsigned long timer0_micros;
  timer0_micros = 0;
#endif
  long t = time_micros_since(0);
  TEST_ASSERT(t == 0, 
      F("expected time_micros_since(0) == %d, got %d"), 
      0, t);

  return ret;
}

