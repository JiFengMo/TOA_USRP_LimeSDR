#include "radio_factory.h"
#include "radio_lime.h"

#include <string.h>

int radio_device_init(radio_device_t *device, radio_device_type_t type)
{
  if (!device)
    return -1;

  /* 先整体清零 */
  memset(device, 0, sizeof(*device));

  switch (type) {
    case RADIO_DEVICE_LIME:
      return radio_lime_device_init(device);

    case RADIO_DEVICE_USRP:
      /* 后续再补 USRP 实现 */
      return -2;

    default:
      return -3;
  }
}