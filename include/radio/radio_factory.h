#ifndef RADIO_FACTORY_H
#define RADIO_FACTORY_H

#include "radio_device.h"

/* 根据设备类型初始化统一设备对象 */
int radio_device_init(radio_device_t *device, radio_device_type_t type);

#endif