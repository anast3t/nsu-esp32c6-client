#include "led_ctrl.h"
#include "driver/gpio.h"
#include "led_strip.h"

#define LED_GPIO       8
#define LED_COUNT      1
static led_strip_handle_t strip;

esp_err_t led_ctrl_init(void) {
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
    return led_strip_new_rmt_device(&cfg, &rmt, &strip);
}

void led_ctrl_set(bool on) {
    led_strip_set_pixel(strip, 0, on ? 32 : 0, 0, 0);
    led_strip_refresh(strip);
}
