#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "can_common.h"
#include "ota.h"
#include "discovery.h"

static const char *TAG = "solstice";

// Waveshare ESP32-S3-RS485-CAN pin assignments — VE.Direct from IDC 2x10 ribbon header
#define VICTRON_TX_PIN   3   // MPPT TX — ribbon wire 9  (outer row, position 5)
#define VICTRON_RX_PIN   14  // MPPT RX — ribbon wire 10 (inner row, position 5)
#define SHUNT_RX_PIN     4   // Shunt RX — ribbon wire 11 (outer row, position 6), RX only
#define CAN_RX_PIN       16
#define CAN_TX_PIN       15

#define VICTRON_UART     UART_NUM_1
#define SHUNT_UART       UART_NUM_2
#define VICTRON_BUF_SIZE 1024
#define SHUNT_BUF_SIZE   1024

// CAN message identifiers
#define CAN_ID_BATT_DATA1        0x23  // ShuntBasicData1 (voltage, current, SOC)
#define CAN_ID_BATT_DATA2        0x24  // ShuntBasicData2 (wattage, TTG)
#define CAN_ID_SHUNT_EXT_LIVE    0x2B  // ShuntExtLive (temp, midpoint, alarm) — zero: no HEX GET
#define CAN_ID_SOLAR_DATA1       0x2C  // SolarMpptData1
#define CAN_ID_SOLAR_DATA2       0x2D  // SolarMpptData2
#define CAN_ID_LOAD_CONTROL      0x2E  // SolarLoadControl (RX)
#define CAN_ID_SHUNT_EXT_HIST    0x2F  // ShuntExtHistory — zero: no HEX GET

#define CAN_STATUS_PERIOD_MS     33
#define TX_PROBE_INTERVAL_MS     2000

// ---------------------------------------------------------------------------
// Parsed MPPT data (from UART1 TEXT protocol)
// ---------------------------------------------------------------------------
static volatile int panel_voltage_whole;
static volatile int panel_voltage_decimal;
static volatile uint8_t solar_wattage_msb;
static volatile uint8_t solar_wattage_lsb;
static volatile int battery_voltage_whole;
static volatile int battery_voltage_decimal;
static volatile int is_panel_current_negative;
static volatile int panel_current_whole;
static volatile int panel_current_decimal;
static volatile int solar_status;

// ---------------------------------------------------------------------------
// Parsed SmartShunt data — TEXT (from UART2, maps to 0x23/0x24)
// ---------------------------------------------------------------------------
static volatile int bmv_voltage_whole;
static volatile int bmv_voltage_decimal;
static volatile int bmv_is_current_negative;
static volatile int bmv_current_whole;
static volatile int bmv_current_decimal;
static volatile int bmv_soc_whole;
static volatile int bmv_soc_decimal;
static volatile uint8_t bmv_wattage_sign;
static volatile uint8_t bmv_wattage_msb;
static volatile uint8_t bmv_wattage_lsb;
static volatile uint8_t bmv_ttg_msb;
static volatile uint8_t bmv_ttg_lsb;

// ---------------------------------------------------------------------------
// Extended shunt data (0x2B / 0x2F) — stay zero: SmartShunt UART is RX-only,
// no HEX GET possible without a TX wire to the shunt.
// ---------------------------------------------------------------------------
static volatile int16_t  shunt_temperature_cc;
static volatile uint16_t shunt_midpoint_mv;
static volatile uint16_t shunt_alarm_reason;
static volatile uint16_t shunt_deepest_discharge_ah;
static volatile uint16_t shunt_cumulative_ah;
static volatile uint16_t shunt_charge_cycles;

// ---------------------------------------------------------------------------
// VE.Direct HEX SET — MPPT load output control (UART1 only)
// ---------------------------------------------------------------------------

static uint8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

static uint8_t hex_byte(const char *s)
{
    return (hex_nibble(s[0]) << 4) | hex_nibble(s[1]);
}

static void vedirect_set_load(uint8_t state)
{
    uint8_t cmd = 0x08;
    uint8_t reg_lo = 0xAB;
    uint8_t reg_hi = 0xED;
    uint8_t flags = 0x00;
    uint8_t val = state;
    uint8_t sum = cmd + reg_lo + reg_hi + flags + val;
    uint8_t checksum = (0x55 - sum) & 0xFF;

    char buf[16];
    snprintf(buf, sizeof(buf), ":8%02X%02X%02X%02X%02X\n",
             reg_lo, reg_hi, flags, val, checksum);
    uart_write_bytes(VICTRON_UART, buf, strlen(buf));
    ESP_LOGI(TAG, "VE.Direct HEX SET load=%d", state);
}

// ---------------------------------------------------------------------------
// TWAI (CAN) task
// ---------------------------------------------------------------------------

static void twai_task(void *arg)
{
    twai_reconfigure_alerts(CAN_COMMON_ALERTS, NULL);
    can_common_version_broadcast();

    typedef enum { TX_ACTIVE, TX_PROBING } tx_state_t;
    bool bus_off = false;
    tx_state_t tx_state = TX_ACTIVE;
    int tx_fail_count = 0;
    const int TX_FAIL_THRESHOLD = 3;
    int64_t last_tx_us = 0;
    const int64_t tx_period_us = CAN_STATUS_PERIOD_MS * 1000LL;
    const int64_t tx_probe_period_us = TX_PROBE_INTERVAL_MS * 1000LL;

    while (1) {
        uint32_t triggered;
        twai_read_alerts(&triggered, pdMS_TO_TICKS(CAN_STATUS_PERIOD_MS));

        if (triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            bus_off = true;
            twai_initiate_recovery();
        }
        if (triggered & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "TWAI bus recovered, restarting");
            twai_start();
            bus_off = false;
            tx_fail_count = 0;
            tx_state = TX_PROBING;
        }
        if (triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "TWAI error passive (no peers ACKing?)");
        }
        if (triggered & TWAI_ALERT_TX_FAILED) {
            if (tx_state == TX_ACTIVE) {
                tx_fail_count++;
                if (tx_fail_count >= TX_FAIL_THRESHOLD) {
                    tx_state = TX_PROBING;
                    ESP_LOGW(TAG, "TWAI no peers detected, entering slow probe");
                }
            }
        }
        if (triggered & TWAI_ALERT_TX_SUCCESS) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI probe ACK'd, peer detected, resuming normal TX");
            }
            tx_fail_count = 0;
        }

        if (triggered & TWAI_ALERT_RX_DATA) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI peer detected via RX, resuming normal TX");
            }
            twai_message_t msg;
            while (twai_receive(&msg, 0) == ESP_OK) {
                if (msg.rtr) continue;
                if (msg.identifier == CAN_ID_OTA_TRIGGER) {
                    ota_handle_trigger(msg.data, msg.data_length_code);
                } else if (msg.identifier == CAN_ID_WIFI_CONFIG) {
                    ota_handle_wifi_config(msg.data, msg.data_length_code);
                } else if (msg.identifier == CAN_ID_DISCOVERY_TRIGGER) {
                    discovery_handle_trigger();
                } else if (msg.identifier == CAN_ID_LOAD_CONTROL && msg.data_length_code >= 1) {
                    ESP_LOGI(TAG, "CAN load control received: %d", msg.data[0]);
                    vedirect_set_load(msg.data[0]);
                }
            }
        }

        int64_t now_us = esp_timer_get_time();
        int64_t effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : tx_period_us;
        if (!bus_off && (now_us - last_tx_us >= effective_period)) {
            last_tx_us = now_us;

            // 0x23 — SmartShunt basic: voltage, current, SOC
            twai_message_t m_batt1 = {
                .identifier = CAN_ID_BATT_DATA1,
                .data_length_code = 7,
                .data = {
                    bmv_voltage_whole,
                    bmv_voltage_decimal,
                    bmv_is_current_negative,
                    bmv_current_whole,
                    bmv_current_decimal,
                    bmv_soc_whole,
                    bmv_soc_decimal
                }
            };

            // 0x24 — SmartShunt basic: wattage, TTG
            twai_message_t m_batt2 = {
                .identifier = CAN_ID_BATT_DATA2,
                .data_length_code = 5,
                .data = {
                    bmv_wattage_sign,
                    bmv_wattage_msb,
                    bmv_wattage_lsb,
                    bmv_ttg_msb,
                    bmv_ttg_lsb
                }
            };

            // 0x2B — SmartShunt extended live: temp, midpoint, alarm (zero: no HEX GET)
            int16_t temp = shunt_temperature_cc;
            uint16_t midpt = shunt_midpoint_mv;
            uint16_t alarm = shunt_alarm_reason;
            twai_message_t m_shunt_live = {
                .identifier = CAN_ID_SHUNT_EXT_LIVE,
                .data_length_code = 6,
                .data = {
                    (temp >> 8) & 0xFF, temp & 0xFF,
                    (midpt >> 8) & 0xFF, midpt & 0xFF,
                    (alarm >> 8) & 0xFF, alarm & 0xFF
                }
            };

            // 0x2C — MPPT solar data 1
            twai_message_t m_solar1 = {
                .identifier = CAN_ID_SOLAR_DATA1,
                .data_length_code = 7,
                .data = {
                    panel_voltage_whole,
                    panel_voltage_decimal,
                    solar_wattage_msb,
                    solar_wattage_lsb,
                    battery_voltage_whole,
                    battery_voltage_decimal,
                    solar_status
                }
            };

            // 0x2D — MPPT solar data 2
            twai_message_t m_solar2 = {
                .identifier = CAN_ID_SOLAR_DATA2,
                .data_length_code = 3,
                .data = {
                    is_panel_current_negative,
                    panel_current_whole,
                    panel_current_decimal
                }
            };

            // 0x2F — SmartShunt extended history (zero: no HEX GET)
            uint16_t dd = shunt_deepest_discharge_ah;
            uint16_t cah = shunt_cumulative_ah;
            uint16_t cyc = shunt_charge_cycles;
            twai_message_t m_shunt_hist = {
                .identifier = CAN_ID_SHUNT_EXT_HIST,
                .data_length_code = 6,
                .data = {
                    (dd >> 8) & 0xFF, dd & 0xFF,
                    (cah >> 8) & 0xFF, cah & 0xFF,
                    (cyc >> 8) & 0xFF, cyc & 0xFF
                }
            };

            twai_transmit(&m_batt1, 0);
            twai_transmit(&m_batt2, 0);
            twai_transmit(&m_shunt_live, 0);
            twai_transmit(&m_solar1, 0);
            twai_transmit(&m_solar2, 0);
            twai_transmit(&m_shunt_hist, 0);
        }
    }
}

// ---------------------------------------------------------------------------
// MPPT VE.Direct TEXT parser (UART1)
// ---------------------------------------------------------------------------

static void process_vedirect_key_value(const char *key, const char *value)
{
    if (strcmp(key, "V") == 0) {
        float voltage = atoi(value) * 0.001f;
        battery_voltage_whole = (int)voltage;
        battery_voltage_decimal = (int)((voltage - battery_voltage_whole) * 100);
        ESP_LOGI(TAG, "MPPT V (battery): %d.%02d V", battery_voltage_whole, battery_voltage_decimal);
    } else if (strcmp(key, "VPV") == 0) {
        float voltage = atoi(value) * 0.001f;
        panel_voltage_whole = (int)voltage;
        panel_voltage_decimal = (int)((voltage - panel_voltage_whole) * 100);
        ESP_LOGI(TAG, "MPPT VPV (panel): %d.%02d V", panel_voltage_whole, panel_voltage_decimal);
    } else if (strcmp(key, "PPV") == 0) {
        int wattage = atoi(value);
        solar_wattage_lsb = wattage & 0xFF;
        solar_wattage_msb = (wattage >> 8) & 0xFF;
        ESP_LOGI(TAG, "MPPT PPV (solar power): %d W", wattage);
    } else if (strcmp(key, "I") == 0) {
        float current = atoi(value) * 0.001f;
        panel_current_whole = (int)current;
        panel_current_decimal = (int)((current - panel_current_whole) * 100);
        is_panel_current_negative = (current < 0) ? 1 : 0;
        ESP_LOGI(TAG, "MPPT I (panel current): %s%d.%02d A",
                 is_panel_current_negative ? "-" : "", panel_current_whole, panel_current_decimal);
    } else if (strcmp(key, "CS") == 0) {
        solar_status = atoi(value);
        ESP_LOGI(TAG, "MPPT CS (charge state): %d", solar_status);
    } else if (strcmp(key, "LOAD") == 0) {
        ESP_LOGI(TAG, "MPPT Load: %s", value);
    } else if (strcmp(key, "H19") == 0) {
        ESP_LOGI(TAG, "Yield Total: %.2f Kw", atoi(value) * 0.01f);
    } else if (strcmp(key, "H20") == 0) {
        ESP_LOGI(TAG, "Yield Today: %.2f Kw", atoi(value) * 0.01f);
    } else if (strcmp(key, "H21") == 0) {
        ESP_LOGI(TAG, "Maximum Power Today: %d W", atoi(value));
    } else if (strcmp(key, "H22") == 0) {
        ESP_LOGI(TAG, "Yield Yesterday: %.2f Kw", atoi(value) * 0.01f);
    } else if (strcmp(key, "H23") == 0) {
        ESP_LOGI(TAG, "Maximum Power Yesterday: %d W", atoi(value));
    } else if (strcmp(key, "ERR") == 0) {
        ESP_LOGW(TAG, "MPPT Error: %s", value);
    } else if (strcmp(key, "FW") == 0) {
        ESP_LOGI(TAG, "MPPT Firmware: %s", value);
    } else if (strcmp(key, "PID") == 0) {
        ESP_LOGI(TAG, "MPPT Product ID: %s", value);
    } else if (strcmp(key, "SER#") == 0) {
        ESP_LOGI(TAG, "MPPT Serial: %s", value);
    } else if (strcmp(key, "HSDS") == 0) {
        ESP_LOGI(TAG, "MPPT Day Number: %s", value);
    } else if (strcmp(key, "MPPT") == 0) {
        ESP_LOGI(TAG, "MPPT Tracker Mode: %s", value);
    }
}

static void parse_vedirect_line(const char *line)
{
    if (line[0] == ':') return;  // skip HEX frames (load control ACKs etc.)
    const char *tab = strchr(line, '\t');
    if (tab == NULL) return;

    size_t key_len = tab - line;
    char key[32];
    if (key_len >= sizeof(key)) return;
    memcpy(key, line, key_len);
    key[key_len] = '\0';
    process_vedirect_key_value(key, tab + 1);
}

// ---------------------------------------------------------------------------
// SmartShunt VE.Direct TEXT parser (UART2)
// ---------------------------------------------------------------------------

static void parse_bmv_field(const char *key, const char *value)
{
    if (strcmp(key, "V") == 0) {
        float v = atoi(value) * 0.001f;
        bmv_voltage_whole = (int)v;
        bmv_voltage_decimal = (int)((v - bmv_voltage_whole) * 100);
        ESP_LOGI(TAG, "BMV V (battery): %d.%02d V", bmv_voltage_whole, bmv_voltage_decimal);
    } else if (strcmp(key, "I") == 0) {
        float current = atoi(value) * 0.001f;
        if (current < 0) {
            bmv_is_current_negative = 1;
            current = -current;
        } else {
            bmv_is_current_negative = 0;
        }
        bmv_current_whole = (int)current;
        bmv_current_decimal = (int)((current - bmv_current_whole) * 100);
        ESP_LOGI(TAG, "BMV I (current): %s%d.%02d A",
                 bmv_is_current_negative ? "-" : "", bmv_current_whole, bmv_current_decimal);
    } else if (strcmp(key, "P") == 0) {
        int watts = atoi(value);
        bmv_wattage_sign = (watts < 0) ? 0xFF : 0x00;
        unsigned int abs_watts = abs(watts);
        bmv_wattage_msb = (abs_watts >> 8) & 0xFF;
        bmv_wattage_lsb = abs_watts & 0xFF;
        ESP_LOGI(TAG, "BMV P (power): %d W", watts);
    } else if (strcmp(key, "SOC") == 0) {
        float soc = atoi(value) * 0.1f;
        bmv_soc_whole = (int)soc;
        bmv_soc_decimal = (int)((soc - bmv_soc_whole) * 100);
        ESP_LOGI(TAG, "BMV SOC: %d.%02d%%", bmv_soc_whole, bmv_soc_decimal);
    } else if (strcmp(key, "TTG") == 0) {
        int ttg = atoi(value);
        bmv_ttg_msb = (ttg >> 8) & 0xFF;
        bmv_ttg_lsb = ttg & 0xFF;
        ESP_LOGI(TAG, "BMV TTG: %d min", ttg);
    } else if (strcmp(key, "CE") == 0) {
        ESP_LOGI(TAG, "BMV Consumed Ah: %.3f", atoi(value) * 0.001f);
    } else if (strcmp(key, "Alarm") == 0) {
        if (strcmp(value, "OFF") != 0)
            ESP_LOGW(TAG, "BMV Alarm: %s", value);
    } else if (strcmp(key, "AR") == 0) {
        if (strcmp(value, "0") != 0)
            ESP_LOGW(TAG, "BMV Alarm Reason: %s", value);
    } else if (strcmp(key, "BMV") == 0) {
        ESP_LOGI(TAG, "BMV Model: %s", value);
    } else if (strcmp(key, "FW") == 0) {
        ESP_LOGI(TAG, "BMV Firmware: %s", value);
    } else if (strcmp(key, "PID") == 0) {
        ESP_LOGI(TAG, "BMV Product ID: %s", value);
    }
}

static void parse_bmv_line(const char *line)
{
    if (line[0] == ':') return;  // skip any unsolicited HEX frames
    const char *tab = strchr(line, '\t');
    if (tab == NULL) return;

    size_t key_len = tab - line;
    char key[32];
    if (key_len >= sizeof(key)) return;
    memcpy(key, line, key_len);
    key[key_len] = '\0';
    parse_bmv_field(key, tab + 1);
}

// ---------------------------------------------------------------------------
// SmartShunt task — reads UART2 TEXT only (RX-only, no HEX GET)
// ---------------------------------------------------------------------------

static void bmv_task(void *arg)
{
    char line_buf[256];
    int line_pos = 0;
    uint8_t uart_byte;
    uint32_t rx_byte_count = 0;
    int64_t last_rx_report_us = 0;

    while (1) {
        int len = uart_read_bytes(SHUNT_UART, &uart_byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            rx_byte_count++;
            int64_t now_report = esp_timer_get_time();
            if ((now_report - last_rx_report_us) >= 5000000LL) {
                ESP_LOGI(TAG, "SmartShunt RX: %"PRIu32" bytes received so far", rx_byte_count);
                last_rx_report_us = now_report;
            }
            if (uart_byte == '\n' || uart_byte == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    parse_bmv_line(line_buf);
                    line_pos = 0;
                }
            } else if (line_pos < (int)(sizeof(line_buf) - 1)) {
                line_buf[line_pos++] = (char)uart_byte;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Main application
// ---------------------------------------------------------------------------

void app_main(void)
{
    ota_init();
    discovery_init();

    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // MPPT on UART1 — full duplex (TX for load control, RX for TEXT data)
    uart_driver_install(VICTRON_UART, VICTRON_BUF_SIZE, VICTRON_BUF_SIZE, 0, NULL, 0);
    uart_param_config(VICTRON_UART, &uart_config);
    uart_set_pin(VICTRON_UART, VICTRON_TX_PIN, VICTRON_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "MPPT UART ready: TX=GPIO%d RX=GPIO%d @ 19200", VICTRON_TX_PIN, VICTRON_RX_PIN);

    // SmartShunt on UART2 — RX only (no TX wire in ribbon cable)
    uart_driver_install(SHUNT_UART, SHUNT_BUF_SIZE, SHUNT_BUF_SIZE, 0, NULL, 0);
    uart_param_config(SHUNT_UART, &uart_config);
    uart_set_pin(SHUNT_UART, UART_PIN_NO_CHANGE, SHUNT_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "SmartShunt UART ready: RX=GPIO%d @ 19200 (RX only)", SHUNT_RX_PIN);

    ESP_LOGI(TAG, "=== TrailCurrent Solstice ===");
    ESP_LOGI(TAG, "Hostname: %s", ota_get_hostname());

    ESP_ERROR_CHECK(can_common_init(CAN_TX_PIN, CAN_RX_PIN));
    xTaskCreatePinnedToCore(twai_task, "twai", 4096, NULL, 5, NULL, 1);
    xTaskCreate(bmv_task, "bmv", 4096, NULL, 4, NULL);

    // Main task: read MPPT VE.Direct (UART1)
    char line_buf[256];
    int line_pos = 0;
    uint8_t uart_byte;
    uint32_t rx_byte_count = 0;
    int64_t last_rx_report_us = 0;

    while (1) {
        int len = uart_read_bytes(VICTRON_UART, &uart_byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            rx_byte_count++;
            int64_t now_report = esp_timer_get_time();
            if ((now_report - last_rx_report_us) >= 5000000LL) {
                ESP_LOGI(TAG, "MPPT RX: %"PRIu32" bytes received so far", rx_byte_count);
                last_rx_report_us = now_report;
            }
            if (uart_byte == '\n' || uart_byte == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    parse_vedirect_line(line_buf);
                    line_pos = 0;
                }
            } else if (line_pos < (int)(sizeof(line_buf) - 1)) {
                line_buf[line_pos++] = (char)uart_byte;
            }
        }
    }
}
