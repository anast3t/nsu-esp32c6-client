/* --------------------------------------------------------------------------
 *  Wi‑Fi LED Client (ESP32‑C6)
 *  – STA‑режим, подключается к SoftAP "ESP32C6_AP"
 *  – TCP‑клиент на 192.168.4.1:5000
 *  – Принимает строки "LED:ON" / "LED:OFF" и управляет WS2812 на GPIO 8
 *  – Жёсткий выбор стандарта Wi‑Fi (4 или 6) аналогично серверу
 *  – Использует select() + тайм‑аут вместо блокирующего recv
 *  – Логирует задержку от получения данных до переключения светодиода
 *  – Переносим на любые ESP‑SoC: задача создаётся через xTaskCreate,
 *    а на многоядерных чипах при необходимости закрепляется
 *  – Компилируется на ESP‑IDF ≥ v5.4
 * -------------------------------------------------------------------------*/
#include <string.h>
#include <sys/select.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_cpu.h"
#include "esp_system.h"   /* CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ */

/* -------------------------- Wi‑Fi ---------------------------------------- */
#define WIFI_SSID      "ESP32C6_AP"
#define WIFI_PASS      "12345678"
#define WIFI_MAX_RETRY 10
#define SERVER_IP      "192.168.4.1"   /* IP SoftAP */
#define SERVER_PORT    5000

/* ---- Жёсткий выбор стандарта Wi‑Fi для STA (BGN‑only или AX‑only) ------ */
typedef enum {
    WIFI_STD_4,   // 802.11b/g/n
    WIFI_STD_6    // 802.11ax
} wifi_standard_t;

static esp_err_t wifi_sta_set_standard_strict(wifi_standard_t std)
{
    wifi_protocols_t proto = {0};
    switch (std) {
        case WIFI_STD_4:
            proto.ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
            break;
        case WIFI_STD_6:
            proto.ghz_2g = WIFI_PROTOCOL_11AX;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    return esp_wifi_set_protocols(WIFI_IF_STA, &proto);
}

/* ------------------------- Светодиод ------------------------------------ */
#define LED_STRIP_GPIO   8
#define LED_STRIP_COUNT  1

static const char *TAG = "LED_CLIENT";
static EventGroupHandle_t s_wifi_event_group;
static led_strip_handle_t led_strip;
static TaskHandle_t tcp_task_handle = NULL;
static int retry_cnt = 0;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* ------------------- Wi‑Fi event handler -------------------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_cnt < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            retry_cnt++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip_str[16];
        ip4_addr_t tmp; tmp.addr = event->ip_info.ip.addr;
        ip4addr_ntoa_r(&tmp, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "got ip: %s", ip_str);
        retry_cnt = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t any_id, got_ip_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, &any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        wifi_event_handler, NULL, &got_ip_id));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* Жёстко задаём протокол: Wi‑Fi 6 only (можно сменить на WIFI_STD_4) */
    ESP_ERROR_CHECK(wifi_sta_set_standard_strict(WIFI_STD_6));

    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID %s", WIFI_SSID);
    } else {
        ESP_LOGI(TAG, "Connected to SSID %s", WIFI_SSID);
    }
}

/* ------------------------- LED helper ----------------------------------- */
static void led_set(bool on)
{
    uint8_t val = on ? 32 : 0;
    led_strip_set_pixel(led_strip, 0, val, 0, 0);
    led_strip_refresh(led_strip);
}

/* ------------------------- TCP client ----------------------------------- */
static void tcp_client_task(void *arg)
{
    for (;;) {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        struct sockaddr_in dest = {
            .sin_family      = AF_INET,
            .sin_port        = htons(SERVER_PORT),
            .sin_addr.s_addr = inet_addr(SERVER_IP)
        };
        ESP_LOGI(TAG, "Connecting to %s:%d", SERVER_IP, SERVER_PORT);
        if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
            ESP_LOGE(TAG, "connect failed, retry in 1 s");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int yes = 1;
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        ESP_LOGI(TAG, "Connected, waiting for data");
        const uint32_t cpu_hz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000UL;
        char rx[128];
        fd_set rfds; struct timeval tv;

        while (1) {
            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);
            tv.tv_sec = 5; tv.tv_usec = 0;
            int sel = select(sock + 1, &rfds, NULL, NULL, &tv);
            if (sel < 0) {
                ESP_LOGE(TAG, "select error");
                break;
            }
            if (sel == 0) continue; // timeout

            uint32_t t_start = esp_cpu_get_cycle_count();
            int n = recv(sock, rx, sizeof(rx) - 1, 0);
            if (n <= 0) {
                ESP_LOGW(TAG, "connection closed");
                break;
            }
            rx[n] = '\0';
            char *saveptr;
            char *line = strtok_r(rx, "\n", &saveptr);
            while (line) {
                if (strstr(line, "LED:ON") || strstr(line, "LED:OFF")) {
                    bool on = strstr(line, "LED:ON") != NULL;
                    led_set(on);
                    uint32_t cycles = esp_cpu_get_cycle_count() - t_start;
                    float ns = (float)cycles * 1e9f / (float)cpu_hz;
                    float ms = ns / 1e6f;
                    ESP_LOGI(TAG, "LED %s | cycles:%lu %.2f ns (%.6f ms)",
                             on ? "ON" : "OFF", cycles, ns, ms);
                }
                line = strtok_r(NULL, "\n", &saveptr);
            }
        }
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* --------------------------- app_main ----------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    /* LED strip init */
    led_strip_config_t cfg = {
        .strip_gpio_num         = LED_STRIP_GPIO,
        .max_leds               = LED_STRIP_COUNT,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model              = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&cfg, &rmt, &led_strip));
    led_set(false);

    BaseType_t ret = xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 9, &tcp_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create tcp_client task");
    }
#if portNUM_PROCESSORS > 1
    vTaskCoreAffinitySet(tcp_task_handle, 1 << 0); /* pin to core0 */
#endif
}
