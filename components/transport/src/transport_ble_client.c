#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_cpu.h"

#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#include "led_ctrl.h"

#define BLE_SVC_UUID16 0x1815
#define BLE_CHR_UUID16 0x2A56

#define TAG "CLIENT_TRANSPORT_BLE"

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
    led_ctrl_set(on);

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
        led_ctrl_set(on);

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

esp_err_t transport_init(){
    ble_init();
    return ESP_OK;
}

