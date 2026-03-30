#ifndef COMMON_ERROR_H
#define COMMON_ERROR_H

typedef enum {
  TOA_OK = 0,
  TOA_ERR_INVALID_ARG = -1,
  TOA_ERR_NO_MEMORY = -2,
  TOA_ERR_IO = -3,
  TOA_ERR_UNSUPPORTED = -4,
  TOA_ERR_NOT_READY = -5,
  TOA_ERR_TIMEOUT = -6,
  TOA_ERR_STATE = -7
} toa_error_t;

#endif
