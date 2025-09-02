#pragma once
#include "esp_err.h"
#include <stdint.h>

/** Инициализация ленты WS2812. */
esp_err_t led_ctrl_init(void);

/** Задать цвет пикселя: r,g,b в диапазоне 0..32 */
void led_ctrl_set_color(uint8_t r, uint8_t g, uint8_t b);
