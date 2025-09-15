#include "ble_server.h"

#include <string.h>
#include "esp_log.h"
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
#include "storage.h"

static const char *TAG = "ble";

// Custom 128-bit UUIDs (base: 5e0bxxxx-6f72-4761-8e3e-7a1c1b5f9b11)
// Service UUID: 5e0b0001-6f72-4761-8e3e-7a1c1b5f9b11
// State (READ) Char: 5e0b0002-6f72-4761-8e3e-7a1c1b5f9b11
// Control (WRITE) Char: 5e0b0003-6f72-4761-8e3e-7a1c1b5f9b11
static const ble_uuid128_t UUID_SERVICE = BLE_UUID128_INIT(0x11,0x9b,0x5f,0x1b,0x1c,0x7a,0x3e,0x8e,0x61,0x47,0x72,0x6f,0x01,0x00,0x0b,0x5e);
static const ble_uuid128_t UUID_STATE   = BLE_UUID128_INIT(0x11,0x9b,0x5f,0x1b,0x1c,0x7a,0x3e,0x8e,0x61,0x47,0x72,0x6f,0x02,0x00,0x0b,0x5e);
static const ble_uuid128_t UUID_CONTROL = BLE_UUID128_INIT(0x11,0x9b,0x5f,0x1b,0x1c,0x7a,0x3e,0x8e,0x61,0x47,0x72,0x6f,0x03,0x00,0x0b,0x5e);

static uint16_t s_state_val_handle;

// Read and write access callbacks
static int chr_state_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t states[4];
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

    // Variable-length command stream; Big-Endian for multi-byte integers.
    // Command format(s):
    // 0x00 SET:   [0x00][ch][value]
    // 0x01 FADE:  [0x01][ch][value][dur_hi][dur_lo]
    // 0x02 BLINK: [0x02][ch][period_hi][period_lo]
    // 0x03 STROBE:[0x03][ch][count][total_hi][total_lo][pause_hi][pause_lo]
    //   count = 点亮次数 (number of ON pulses)
    //   period = total/count, ON=period/2, OFF=period/2

    uint8_t buf[128];
    uint16_t copied = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied);
    if (rc != 0 || copied < 2) {
        ESP_LOGW(TAG, "Invalid write len=%u rc=%d", (unsigned)copied, rc);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    bool need_persist = false;
    uint16_t idx = 0;
    while (idx < copied) {
        uint8_t mode = buf[idx++];
        if (idx >= copied) {
            ESP_LOGW(TAG, "Truncated after mode=0x%02X", mode);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint8_t ch = buf[idx++];
        if (ch >= PWM_CHANNEL_COUNT) {
            ESP_LOGW(TAG, "Invalid channel=%u", ch);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        esp_err_t err = ESP_OK;
        switch (mode) {
            case 0x00: { // SET
                if (idx + 1 > copied) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                uint8_t value = buf[idx++];
                ESP_LOGI(TAG, "SET ch=%u val=%u", ch, value);
                err = pwm_control_apply(ch, value, 0);
                need_persist = true;
                break;
            }
            case 0x01: { // FADE
                if (idx + 3 > copied) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                uint8_t value = buf[idx++];
                uint16_t dur = ((uint16_t)buf[idx] << 8) | (uint16_t)buf[idx+1];
                idx += 2;
                ESP_LOGI(TAG, "FADE ch=%u val=%u dur_ms=%u", ch, value, (unsigned)dur);
                err = pwm_control_apply(ch, value, dur);
                need_persist = true;
                break;
            }
            case 0x02: { // BLINK
                if (idx + 2 > copied) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                uint16_t period = ((uint16_t)buf[idx] << 8) | (uint16_t)buf[idx+1];
                idx += 2;
                ESP_LOGI(TAG, "BLINK ch=%u period_ms=%u", ch, (unsigned)period);
                err = pwm_control_start_blink(ch, period);
                break;
            }
            case 0x03: { // STROBE
                if (idx + 5 > copied) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                uint8_t count = buf[idx++];
                uint16_t total = ((uint16_t)buf[idx] << 8) | (uint16_t)buf[idx+1];
                idx += 2;
                uint16_t pause = ((uint16_t)buf[idx] << 8) | (uint16_t)buf[idx+1];
                idx += 2;
                ESP_LOGI(TAG, "STROBE ch=%u count=%u total_ms=%u pause_ms=%u", ch, count, (unsigned)total, (unsigned)pause);
                err = pwm_control_start_strobe(ch, count, total, pause);
                break;
            }
            default:
                ESP_LOGW(TAG, "Unknown mode=0x%02X", mode);
                return BLE_ATT_ERR_UNLIKELY;
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Op failed mode=0x%02X ch=%u err=%d", mode, ch, err);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    if (need_persist) {
        uint8_t states[4];
        pwm_control_get_states(states);
        esp_err_t nvs_rc = storage_write_states(states);
        if (nvs_rc != ESP_OK) {
            ESP_LOGW(TAG, "NVS persist failed rc=%d", nvs_rc);
        } else {
            ESP_LOGI(TAG, "State persisted: [%u,%u,%u,%u]",
                    states[0], states[1], states[2], states[3]);
        }
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SERVICE.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &UUID_STATE.u,
                .access_cb = chr_state_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &s_state_val_handle,
            },
            {
                .uuid = &UUID_CONTROL.u,
                .access_cb = chr_control_access,
                .flags = BLE_GATT_CHR_F_WRITE,
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

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields rc=%d", rc);
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
    } else {
        ESP_LOGI(TAG, "Advertising started");
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
            return 0;
        }
        // restart advertising on failed connect
        start_advertise();
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "GAP disconnect reason=%d; restart advertising", event->disconnect.reason);
        start_advertise();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete reason=%d, restarting", event->adv_complete.reason);
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
    if (rc != 0) return ESP_FAIL;
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) return ESP_FAIL;

    // ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);
    return ESP_OK;
}
