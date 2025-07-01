#include "nvs_flash.h"
#include "esp_log.h"
#include "transport.h"
#include "led_ctrl.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // Сначала LED-controller, чтобы transport мог сразу обновлять пиксель
    ESP_ERROR_CHECK(led_ctrl_init());

    // Затем стартуем клиентский транспорт
    ESP_ERROR_CHECK(transport_init());

    // ESP_LOGI("MAIN", "Client ready (mode: %s)",
    //          CONFIG_PROTO_BT_ENABLED ? "BLE" : "Wi-Fi");
}