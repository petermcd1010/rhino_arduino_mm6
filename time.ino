/*
 * Time functions.
 */

static uint32_t seconds_offset = 0;
static uint32_t factional_seconds_offset = 0;
const int time_str_len = 20;

void time_set(uint32_t seconds, uint32_t fractional_seconds)
{
  assert(false);
}

uint32_t time_micros()
{
  assert(false);
  return -1;
}

uint32_t time_millis()
{
  assert(false);
  return -1;
}

float time_seconds_float()
{
  assert(false);
  return -1.0f;
}

void time_string(char *pbuffer, size_t nbytes_buffer)
{
  assert(pbuffer);
  snprintf(pbuffer, nbytes_buffer, "yyyy-mm-dd hh:mm:ss.0123456");
}

float time_uptime_float()
{
  return -1.0f;
}

bool test_time()
{
  // TODO.
}

