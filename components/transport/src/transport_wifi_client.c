#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_cpu.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

#include "led_ctrl.h"

#define TAG "CLIENT_TRANSPORT_WIFI"

#define WIFI_SSID "ESP32C6_AP"
#define WIFI_PASS "12345678"
#define SRV_IP "192.168.4.1"
#define SRV_PORT 5000

/* ---------- Wi-Fi 4 / 6 ------------------------------------------------- */
typedef enum
{
    WIFI_STD_4,
    WIFI_STD_6
} wifi_std_t;
static esp_err_t wifi_sta_proto(wifi_std_t s)
{
    wifi_protocols_t p = {0};
    p.ghz_2g = (s == WIFI_STD_4) ? (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N) : WIFI_PROTOCOL_11AX;
    return esp_wifi_set_protocols(WIFI_IF_STA, &p);
}

/* ---------- Wi-Fi init -------------------------------------------------- */
static EventGroupHandle_t eg;
#define BIT_OK BIT0
#define BIT_FAIL BIT1

static void evt(void *a, esp_event_base_t b, int32_t id, void *d)
{
    if (b == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (b == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        ESP_LOGW(TAG, "re-connect");
    }
    else if (b == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(eg, BIT_OK);
    }
}

static void wifi_init(void)
{
    eg = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t h1, h2;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, evt, NULL, &h1);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, evt, NULL, &h2);

    wifi_config_t c = {.sta = {.ssid = WIFI_SSID, .password = WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK}};
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &c);

    wifi_sta_proto(WIFI_STD_6);    /* Wi-Fi 6 only */
    esp_wifi_set_ps(WIFI_PS_NONE); /* no power-save */
    esp_wifi_start();

    xEventGroupWaitBits(eg, BIT_OK, pdFALSE, pdFALSE, portMAX_DELAY);
    wifi_protocols_t chk;
    esp_wifi_get_protocols(WIFI_IF_STA, &chk);
    ESP_LOGI(TAG, "STA proto 2G=0x%X (AX=0x20)", chk.ghz_2g);
}

/* ---------- TCP client task -------------------------------------------- */
static void client(void *a)
{
    const uint32_t cpu = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000UL;
    for (;;)
    {
        int s = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in d = {.sin_family = AF_INET, .sin_port = htons(SRV_PORT), .sin_addr.s_addr = inet_addr(SRV_IP)};
        if (connect(s, (void *)&d, sizeof(d)) != 0)
        {
            ESP_LOGE(TAG, "connect fail");
            close(s);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        int yes = 1;
        setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        fd_set rfds;
        struct timeval tv;
        char buf[128];
        for (;;)
        {
            FD_ZERO(&rfds);
            FD_SET(s, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 20000; /* 20 ms дворник */
            if (select(s + 1, &rfds, NULL, NULL, &tv) <= 0)
                continue;
            uint32_t t0 = esp_cpu_get_cycle_count();
            int n = recv(s, buf, sizeof(buf) - 1, 0);
            if (n <= 0)
            {
                ESP_LOGW(TAG, "closed...");
                break;
            }
            buf[n] = '\0';
            char *save, *ln = strtok_r(buf, "\n", &save);
            while (ln)
            {
                bool on = strstr(ln, "LED:ON");
                if (on || strstr(ln, "LED:OFF"))
                {
                    led_ctrl_set(on);
                    uint32_t cyc = esp_cpu_get_cycle_count() - t0;
                    float ns = cyc * 1e9f / cpu, ms = ns / 1e6f;
                    ESP_LOGI(TAG, "LED %s | cycles:%lu %.2f ns (%.6f ms)", on ? "ON" : "OFF", cyc, ns, ms);
                }
                ln = strtok_r(NULL, "\n", &save);
            }
        }
        close(s);
    }
}

esp_err_t transport_init(){
    wifi_init();
    xTaskCreatePinnedToCore(client, "tcp", 4096, NULL, 9, NULL, 0);
    return ESP_OK;
}