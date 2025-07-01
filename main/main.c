/* ------------------------------------------------------------------
 *  ESP32-C6  —  универсальный КЛИЕНТ
 *      • Wi-Fi STA  + TCP            (PROTO_BT = 0)
 *      • BLE Central + GATT Notify   (PROTO_BT = 1)
 * ----------------------------------------------------------------- */
#define PROTO_BT 0 /* 0 = Wi-Fi,  1 = Bluetooth LE */

/* ───────── общие includes ───────────────────────────────────────── */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_cpu.h"

/* ───────── BLE ──────────────────────────────────────────────────── */
#if PROTO_BT
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#else /* ─────── Wi-Fi / TCP ───────────── */
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#endif

/* ───────── конфигурация ─────────────────────────────────────────── */
#define LED_GPIO 8
#define LED_COUNT 1
#define TAG "LED_CLIENT"

/* ---------- Wi-Fi параметры (остаются прежними) ------------------ */
#if !PROTO_BT
#define WIFI_SSID "ESP32C6_AP"
#define WIFI_PASS "12345678"
#define SRV_IP "192.168.4.1"
#define SRV_PORT 5000
#endif

static led_strip_handle_t strip;

/* ================================================================== */
/*                          Wi-Fi  (без изменений)                    */
/* ================================================================== */
#if !PROTO_BT
static TaskHandle_t cli_task;

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
                ESP_LOGW(TAG, "closed");
                break;
            }
            buf[n] = '\0';
            char *save, *ln = strtok_r(buf, "\n", &save);
            while (ln)
            {
                bool on = strstr(ln, "LED:ON");
                if (on || strstr(ln, "LED:OFF"))
                {
                    led_strip_set_pixel(strip, 0, on ? 32 : 0, 0, 0);
                    led_strip_refresh(strip);
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
#endif /* !PROTO_BT */

/* ================================================================== */
/*                BLE  (NimBLE Central 0x1815 / 0x2A56)               */
/* ================================================================== */
#if PROTO_BT
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"

#define BLE_SVC_UUID16 0x1815
#define BLE_CHR_UUID16 0x2A56

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t chr_val_hdl = 0;

/* ---------- Notify → обновить светодиод + метка времени ---------- */
static int notify_cb(
    uint16_t conn,
    struct ble_gatt_error *err,
    struct ble_gatt_attr *attr,
    void *arg)
{
    if (err->status || !attr || !attr->om)
        return err->status;

    uint32_t t0 = esp_cpu_get_cycle_count();

    char str[12] = {0};
    ble_hs_mbuf_to_flat(attr->om, str, sizeof(str), NULL); /* четвёртый arg */

    bool on = strstr(str, "LED:ON");
    led_strip_set_pixel(strip, 0, on ? 32 : 0, 0, 0);
    led_strip_refresh(strip);

    uint32_t cyc = esp_cpu_get_cycle_count() - t0;
    float us = cyc * 1e6f / (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1e6);
    ESP_LOGI(TAG, "LED %s | %lu cyc | %.3f µs", on ? "ON" : "OFF", cyc, us);
    return 0;
}

static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr,
                         void *arg)
{
    if (error->status != 0)
    {
        ESP_LOGW(TAG, "CCCD write failed, status=%d", error->status);
    }
    else
    {
        ESP_LOGI(TAG, "CCCD write OK");
    }
    return 0;
}

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (err->status == 0 && chr)
    {
        if (ble_uuid_u16(&chr->uuid.u) == BLE_CHR_UUID16)
        {
            chr_val_hdl = chr->val_handle;
        }
        return 0; // continue discovery
    }

    if (err->status == BLE_HS_EDONE && chr_val_hdl)
    {

        // 2️⃣ Записываем в CCCD для активации Notify
        uint16_t cccd_val = 0x0001;
        int rc = ble_gattc_write_flat(conn_handle,
                                      chr_val_hdl + 1,
                                      &cccd_val,
                                      sizeof(cccd_val),
                                      cccd_write_cb,
                                      NULL);
        if (rc != 0)
        {
            ESP_LOGW(TAG, "Failed to write CCCD rc=%d", rc);
        }
        else
        {
            ESP_LOGI(TAG, "CCCD write initiated");
        }
    }
    return 0;
}

/* ---------- GAP events: scan / connect / disconnect -------------- */
static int gap_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type)
    {

    case BLE_GAP_EVENT_DISC:
    { /* реклама */
        struct ble_hs_adv_fields f = {0};
        ble_hs_adv_parse_fields(&f, ev->disc.data, ev->disc.length_data);

        bool found = false;
        for (int i = 0; i < f.num_uuids16; i++)
        {
            ESP_LOGI(TAG, "BLE Checking: %d", f.uuids16[i].value);
            if (f.uuids16[i].value == BLE_SVC_UUID16)
                found = true;
        }

        if (found)
        {
            ble_gap_disc_cancel();
            ESP_LOGI(TAG, "Target found → connect");
            ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &ev->disc.addr,
                            30000, NULL, gap_cb, NULL);
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0)
        {
            conn_handle = ev->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, discover chrs");
            ble_gattc_disc_all_chrs(conn_handle, 1, 0xFFFF,
                                    disc_chr_cb, NULL);
        }
        else
        { /* retry scan */
            ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                         &((struct ble_gap_disc_params){.passive = 0}),
                         gap_cb, NULL);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        chr_val_hdl = 0;
        ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                     &((struct ble_gap_disc_params){.passive = 0}),
                     gap_cb, NULL);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* длина пришедших данных */
        int len = OS_MBUF_PKTLEN(ev->notify_rx.om);
        uint8_t buf[64];  // подберите размер >= максимальный expected
        if (len > sizeof(buf)-1) len = sizeof(buf)-1;

        /* копируем payload из mbuf */
        os_mbuf_copydata(ev->notify_rx.om, 0, len, buf);
        buf[len] = '\0';

        ESP_LOGI(TAG, "Notification: %s", buf);

        /* обновляем LED */
        bool on = strstr((char*)buf, "LED:ON") != NULL;
        led_strip_set_pixel(strip, 0, on ? 32 : 0, 0, 0);
        led_strip_refresh(strip);

        return 0;
    }

    default:
        return 0;
    }
}

/* ---------- on-sync: запускаем сканирование ----------------------- */
static void ble_on_sync(void)
{
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                 &((struct ble_gap_disc_params){.passive = 0}),
                 gap_cb, NULL);
}

/* ---------- NimBLE host task ------------------------------------- */
static void host_task(void *p)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---------- BLE init --------------------------------------------- */
static void ble_init(void)
{
    esp_bt_mem_release(ESP_BT_MODE_CLASSIC_BT);
    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(host_task);
}
#endif /* PROTO_BT */

/* ================================================================== */
/*                              main                                  */
/* ================================================================== */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

#if PROTO_BT
    // esp_log_level_set("NimBLE", ESP_LOG_NONE);   /* убираем шум NimBLE */
    ble_init();
#else
    wifi_init();
    xTaskCreatePinnedToCore(client, "tcp", 4096, NULL, 9, NULL, 0);
#endif

    /* LED-strip ----------------------------------------------------- */
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model = LED_MODEL_WS2812};
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000};
    led_strip_new_rmt_device(&cfg, &rmt, &strip);

    ESP_LOGI(TAG, "Client ready  (mode: %s)", PROTO_BT ? "BLE" : "Wi-Fi");
}
