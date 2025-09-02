#include "led_ctrl.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_err.h"

#define LED_GPIO   8
#define LED_COUNT  1

static led_strip_handle_t strip;

esp_err_t led_ctrl_init(void) {
    // Настраиваем RMT + WS2812
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds       = LED_COUNT,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model      = LED_MODEL_WS2812
    };
    led_strip_rmt_config_t rmt = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000
    };
    esp_err_t err = led_strip_new_rmt_device(&cfg, &rmt, &strip);
    if (err == ESP_OK) {
        // Сразу гасим
        led_strip_set_pixel(strip, 0, 0, 0, 0);
        led_strip_refresh(strip);
    }
    return err;
}

void led_ctrl_set_color(uint8_t r, uint8_t g, uint8_t b) {
    // Ограничиваем диапазон 0..32
    if (r > 32) r = 32;
    if (g > 32) g = 32;
    if (b > 32) b = 32;
    // Рисуем и обновляем
    led_strip_set_pixel(strip, 0, r, g, b);
    led_strip_refresh(strip);
}
