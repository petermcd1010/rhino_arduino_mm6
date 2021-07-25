/*
 * Software version function.
 */

void version_get_string(char *pbuffer, size_t nbytes_buffer, float version_number)
{
  assert(pbuffer);

  char version_number_string[10] = {};
  dtostrf(version_number, 3, 2, version_number_string);

  snprintf(pbuffer, nbytes_buffer, "%s (%s %s)", version_number_string, __DATE__, __TIME__);
  assert(strlen(pbuffer) != nbytes_buffer - 1);
}
