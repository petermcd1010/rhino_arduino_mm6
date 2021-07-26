/*
 * Utility functions.
 */

bool util_buffer_contains(char *pbuffer, size_t buffer_nbytes, char c)
{
  assert(pbuffer);

  for (int i = 0; i < buffer_nbytes; i++) {
    if (pbuffer[i] == c)
      return true;
  }

  return false;
}