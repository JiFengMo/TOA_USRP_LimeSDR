#ifndef COMMON_CLI_H
#define COMMON_CLI_H

#include <stdint.h>

/*
 * Parse environment variables (apps only; libraries should not call getenv).
 * Return 0 on success, -1 if unset or invalid.
 */
int cli_env_u32(const char *name, uint32_t *out);
int cli_env_f64(const char *name, double *out);

#endif
