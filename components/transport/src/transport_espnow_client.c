// transport_espnow_client.c
#include "transport.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "led_ctrl.h"
#include <string.h>

static const char *TAG = "ESPNOW_CLT";
// static const char *AP_SSID = "ESPNOW_AP";    // должен совпадать с сервером
static const uint8_t ESPNOW_CHANNEL = 5;

// Callback от ESP-NOW — строка “LED:ON\n” или “LED:OFF\n”
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    bool on = (len > 0 && strstr((char*)data, "LED:ON") != NULL);
    led_ctrl_set(on);
    ESP_LOGI(TAG, "Received %.*s → LED %s", len, data, on?"ON":"OFF");
}

esp_err_t transport_init(void)
{
    // 1) NVS уже инициализирован в app_main
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 2) STA
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta_cfg = {
        .sta = {
            .ssid            = "ESPNOW_AP",
            .password        = "",
            .threshold = { .authmode = WIFI_AUTH_OPEN },
            .pmf_cfg   = { .capable = false, .required = false },
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

    // ↓↓↓ дублируем из TCP-клиента
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_ERROR_CHECK( esp_wifi_set_protocol(
        WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N
    ) );

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG, "Wi-Fi STA started (chan:%d)", ESPNOW_CHANNEL);

    // 3) ESPNOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_LOGI(TAG, "ESP-NOW client ready");

    return ESP_OK;
}

