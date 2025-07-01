#pragma once
#include "esp_err.h"

/** Инициализировать Wi-Fi-STA+TCP client или BLE Central. */
esp_err_t transport_init(void);