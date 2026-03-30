#include "common/cli.h"

#include <stdlib.h>

int cli_env_u32(const char *name, uint32_t *out)
{
  const char *v;
  char *endp = NULL;
  unsigned long x;

  if (!name || !out)
    return -1;
  v = getenv(name);
  if (!v)
    return -1;
  x = strtoul(v, &endp, 10);
  if (endp == v)
    return -1;
  *out = (uint32_t)x;
  return 0;
}

int cli_env_f64(const char *name, double *out)
{
  const char *v;
  char *endp = NULL;
  double x;

  if (!name || !out)
    return -1;
  v = getenv(name);
  if (!v)
    return -1;
  x = strtod(v, &endp);
  if (endp == v)
    return -1;
  *out = x;
  return 0;
}
