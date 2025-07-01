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

// esp_err_t led_ctrl_init(void) {
//     // Настроить GPIO8 как выход без подтягивающих резисторов
//     gpio_config_t io_conf = {
//         .pin_bit_mask     = 1ULL << LED_GPIO,
//         .mode             = GPIO_MODE_OUTPUT,
//         .pull_up_en       = GPIO_PULLUP_DISABLE,
//         .pull_down_en     = GPIO_PULLDOWN_DISABLE,
//         .intr_type        = GPIO_INTR_DISABLE
//     };
//     esp_err_t ret = gpio_config(&io_conf);
//     if (ret == ESP_OK) {
//         // Инициализируем светодиод в выключенном состоянии
//         gpio_set_level(LED_GPIO, 0);
//     }
//     return ret;
// }

// void led_ctrl_set(bool on) {
//     // Включить (1) или выключить (0) GPIO8
//     gpio_set_level(LED_GPIO, on ? 1 : 0);
// }
