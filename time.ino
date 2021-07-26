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

bool test_time()
{
  // TODO.
}

