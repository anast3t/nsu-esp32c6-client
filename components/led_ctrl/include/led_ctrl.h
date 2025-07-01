#pragma once
#include <stdbool.h>
#include "esp_err.h"

/** Инициализация ленты и GPIO. */
esp_err_t led_ctrl_init(void);

/** Установить состояние LED (ON=true, OFF=false). */
void      led_ctrl_set(bool on);
