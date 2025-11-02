#include "ble_server.h"

#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
// Optional: bonding store; not required for this project
// #include "store/config/ble_store_config.h"
// #include "host/ble_store.h"

#include "pwm_control.h"
#include "scheduler.h"
#include "command_codec.h"
#include "led_status.h"
#include "power_mgr.h"

static const char *TAG = "ble";

static int chr_power_mgmt_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return power_mgr_ble_access(conn_handle, attr_handle, ctxt);
}


// Custom 128-bit service UUID with 16-bit characteristics per specification
static const ble_uuid128_t UUID_SERVICE = BLE_UUID128_INIT(0x11,0x9b,0x5f,0x1b,0x1c,0x7a,0x3e,0x8e,0x61,0x47,0x72,0x6f,0x01,0x00,0x0b,0x5e);
static const ble_uuid16_t UUID_CH_STATE   = BLE_UUID16_INIT(0xFFF0);
static const ble_uuid16_t UUID_CH_CONTROL = BLE_UUID16_INIT(0xFFF1);
static const ble_uuid16_t UUID_CH_POWER_MGMT   = BLE_UUID16_INIT(0xFFF5);

static uint16_t s_state_val_handle;

// Read and write access callbacks
static int chr_state_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t states[6];  // 支持6个通道
        pwm_control_get_states(states);
        int rc = os_mbuf_append(ctxt->om, states, sizeof(states));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}


static int chr_control_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    uint8_t buffer[256];
    uint16_t copied = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buffer, sizeof(buffer), &copied);
    if (rc != 0) {
        ESP_LOGW(TAG, "Control write flatten rc=%d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (copied < 2) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    size_t offset = 0;
    while (offset < copied) {
        control_cmd_t cmd = {0};
        size_t consumed = 0;
        esp_err_t err = control_cmd_parse(buffer + offset, copied - offset, &consumed, &cmd);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Parse failed at offset=%zu err=%d", offset, err);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        err = control_cmd_validate(&cmd);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Validate failed mode=0x%02X ch=%u err=%d", cmd.mode, cmd.channel, err);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        err = scheduler_submit_command(&cmd, pdMS_TO_TICKS(200));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Scheduler busy mode=0x%02X err=%d", cmd.mode, err);
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        offset += consumed;
    }
    return 0;
}


static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SERVICE.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &UUID_CH_STATE.u,
                .access_cb = chr_state_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &s_state_val_handle,
            },
            {
                .uuid = &UUID_CH_CONTROL.u,
                .access_cb = chr_control_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
                    {
                .uuid = &UUID_CH_POWER_MGMT.u,
                .access_cb = chr_power_mgmt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        },
    },
    { 0 }
};

static uint8_t s_own_addr_type;

static int gap_event_cb(struct ble_gap_event *event, void *arg);

static void log_addr(uint8_t addr_type) {
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_id_copy_addr(addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "Own addr type=%d val=%02X:%02X:%02X:%02X:%02X:%02X",
                 addr_type,
                 addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    } else {
        ESP_LOGW(TAG, "ble_hs_id_copy_addr failed rc=%d for type=%d", rc, addr_type);
    }
}

static void start_advertise(void) {
    // Stop if already advertising; ignore error if not active
    int rc_stop = ble_gap_adv_stop();
    if (rc_stop != 0) {
        ESP_LOGI(TAG, "adv_stop rc=%d (ignore if not active)", rc_stop);
    }
    led_status_set_bluetooth_advertising(false);

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields rc=%d", rc);
        led_status_set_error(true);
        return;
    }

    struct ble_hs_adv_fields rsp = {0};
    const char *name = ble_svc_gap_device_name();
    size_t name_len = strlen(name);
    const size_t uuid128_field_size = 2 + 16; // len+type + 16 bytes
    const size_t name_field_overhead = 2;     // len+type
    // Max payload per packet is 31 bytes
    size_t max_short_name = 31;
    // We prefer to include UUID128; compute remaining space for name
    if (max_short_name > uuid128_field_size) {
        max_short_name -= uuid128_field_size;
    } else {
        max_short_name = 0;
    }
    if (max_short_name > name_field_overhead) {
        max_short_name -= name_field_overhead;
    } else {
        max_short_name = 0;
    }

    uint8_t short_name_buf[31] = {0};
    if (max_short_name >= 1) {
        size_t use_len = name_len < max_short_name ? name_len : max_short_name;
        memcpy(short_name_buf, name, use_len);
        rsp.name = short_name_buf;
        rsp.name_len = use_len;
        rsp.name_is_complete = (use_len == name_len) ? 1 : 0; // truncated => shortened name
        rsp.uuids128 = (ble_uuid128_t[]) { UUID_SERVICE };
        rsp.num_uuids128 = 1;
        rsp.uuids128_is_complete = 1;
    } else {
        // Not enough space for UUID + name; fall back to name only (complete)
        rsp.name = (uint8_t *)name;
        rsp.name_len = name_len;
        rsp.name_is_complete = 1;
    }

    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv rsp set fields rc=%d", rc);
        led_status_set_error(true);
        return;
    }

    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ESP_LOGI(TAG, "Starting advertising: own_addr_type=%d name_len=%u include_uuid128=%d", s_own_addr_type, (unsigned)rsp.name_len, rsp.num_uuids128);
    log_addr(s_own_addr_type);
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv start rc=%d", rc);
        led_status_set_bluetooth_advertising(false);
        led_status_set_error(true);
    } else {
        ESP_LOGI(TAG, "Advertising started");
        led_status_set_bluetooth_advertising(true);
        led_status_set_error(false);
    }
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "GAP connect status=%d", event->connect.status);
        if (event->connect.status == 0) {
            struct ble_gap_conn_desc d;
            if (ble_gap_conn_find(event->connect.conn_handle, &d) == 0) {
                ESP_LOGI(TAG, "Connected handle=%d peer_id_addr=%02X:%02X:%02X:%02X:%02X:%02X type=%d",
                         event->connect.conn_handle,
                         d.peer_id_addr.val[5], d.peer_id_addr.val[4], d.peer_id_addr.val[3],
                         d.peer_id_addr.val[2], d.peer_id_addr.val[1], d.peer_id_addr.val[0],
                         d.peer_id_addr.type);
            }
        }
        if (event->connect.status == 0) {
            led_status_set_bluetooth_connected(true);
            led_status_set_bluetooth_advertising(false);
            led_status_set_error(false);
            return 0;
        }
        // restart advertising on failed connect
        start_advertise();
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "GAP disconnect reason=%d; restart advertising", event->disconnect.reason);
        led_status_set_bluetooth_connected(false);
        power_mgr_ble_subscribe(BLE_HS_CONN_HANDLE_NONE, 0, false);
        start_advertise();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Subscribe event conn=%d attr=%d cur_notify=%d", 
                 event->subscribe.conn_handle, 
                 event->subscribe.attr_handle, 
                 event->subscribe.cur_notify);
        power_mgr_ble_subscribe(event->subscribe.conn_handle,
                                event->subscribe.attr_handle,
                                event->subscribe.cur_notify);
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete reason=%d, restarting", event->adv_complete.reason);
        led_status_set_bluetooth_advertising(false);
        start_advertise();
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated to %d", event->mtu.value);
        return 0;
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "Conn params updated");
        return 0;
    default:
        return 0;
    }
}

static void ble_on_sync(void) {
    ESP_LOGI(TAG, "on_sync: BLE host synced");
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGW(TAG, "infer addr type failed rc=%d, fallback to random", rc);
        s_own_addr_type = BLE_OWN_ADDR_RANDOM;
    }
    log_addr(s_own_addr_type);
    start_advertise();
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_server_init(void) {
    // NVS must already be initialized by caller
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init rc=%d", rc);
        // 设置LED错误状态
        led_status_set_error(true);
        return ESP_FAIL;
    }

    // Host config
    ble_hs_cfg.sync_cb = ble_on_sync;
    // No bonding store callbacks required for this project
    // ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // GAP/GATT init
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("PowerHub");

    // Add services
    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg rc=%d", rc);
        // 设置LED错误状态
        led_status_set_error(true);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs rc=%d", rc);
        // 设置LED错误状态
        led_status_set_error(true);
        return ESP_FAIL;
    }

    // ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);
    return ESP_OK;
}
