#pragma once

#include "nvs_flash.h"
#include "esp_log.h"
#include "transport.h"
#include "led_ctrl.h"


/**
 * Точка входа приложения.
 * Реализована в app_main.c — ESP-IDF вызовет эту функцию автоматически.
 */
void app_main(void);
