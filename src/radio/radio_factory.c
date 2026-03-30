#include "radio/radio_factory.h"
#include "radio/radio_lime.h"
#include "common/error.h"

#include <string.h>

int radio_device_init(radio_device_t *device, radio_device_type_t type)
{
  if (!device)
    return TOA_ERR_INVALID_ARG;

  /* 先整体清�? */
  memset(device, 0, sizeof(*device));

  switch (type) {
    case RADIO_DEVICE_LIME:
      return radio_lime_device_init(device);

    case RADIO_DEVICE_USRP:
      /* 后续再补 USRP 实现 */
      return TOA_ERR_UNSUPPORTED;

    default:
      return TOA_ERR_UNSUPPORTED;
  }
}