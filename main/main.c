#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ota.h"

static const char *TAG = "solstice";

// Waveshare ESP32-S3-RS485-CAN pin assignments
#define VICTRON_RX_PIN   1
#define VICTRON_TX_PIN   2
#define CAN_RX_PIN       16
#define CAN_TX_PIN       15
#define VICTRON_UART     UART_NUM_1
#define VICTRON_BUF_SIZE 1024

// CAN message identifiers
#define CAN_ID_SOLAR_DATA1       0x2C
#define CAN_ID_SOLAR_DATA2       0x2D
#define CAN_ID_LOAD_CONTROL      0x2E

// CAN transmit period in milliseconds
#define CAN_STATUS_PERIOD_MS     33

// Parsed Victron MPPT data
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

// VE.Direct HEX protocol: send a SET command for register 0xEDAB (load output control)
// Value: 0x00=OFF, 0x01=ON, 0x04=Default (use MPPT built-in algorithm)
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
// TWAI (CAN) task -- runs independently so bus errors never stall UART
// ---------------------------------------------------------------------------

static void twai_task(void *arg)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver");
        vTaskDelete(NULL);
        return;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver");
        vTaskDelete(NULL);
        return;
    }

    uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS |
                      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                      TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                      TWAI_ALERT_ERR_ACTIVE;
    twai_reconfigure_alerts(alerts, NULL);
    ESP_LOGI(TAG, "TWAI driver started (NORMAL mode)");

    bool bus_off = false;
    int64_t last_tx_us = 0;
    const int64_t tx_period_us = CAN_STATUS_PERIOD_MS * 1000LL;

    while (1) {
        uint32_t triggered;
        twai_read_alerts(&triggered, pdMS_TO_TICKS(CAN_STATUS_PERIOD_MS));

        if (triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            bus_off = true;
            twai_initiate_recovery();
            continue;
        }

        if (triggered & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "TWAI bus recovered, restarting");
            twai_start();
            bus_off = false;
        }

        if (triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "TWAI error passive (no peers ACKing?)");
        }

        // Drain received messages and dispatch
        if (triggered & TWAI_ALERT_RX_DATA) {
            twai_message_t msg;
            while (twai_receive(&msg, 0) == ESP_OK) {
                if (msg.rtr) continue;

                if (msg.identifier == CAN_ID_OTA_TRIGGER) {
                    ota_handle_trigger(msg.data, msg.data_length_code);
                } else if (msg.identifier == CAN_ID_WIFI_CONFIG) {
                    ota_handle_wifi_config(msg.data, msg.data_length_code);
                } else if (msg.identifier == CAN_ID_LOAD_CONTROL && msg.data_length_code >= 1) {
                    ESP_LOGI(TAG, "CAN load control received: %d", msg.data[0]);
                    vedirect_set_load(msg.data[0]);
                }
            }
        }

        // Periodic transmit (skip if bus is down)
        int64_t now_us = esp_timer_get_time();
        if (!bus_off && (now_us - last_tx_us >= tx_period_us)) {
            last_tx_us = now_us;

            twai_message_t m1 = {
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

            twai_message_t m2 = {
                .identifier = CAN_ID_SOLAR_DATA2,
                .data_length_code = 3,
                .data = {
                    is_panel_current_negative,
                    panel_current_whole,
                    panel_current_decimal
                }
            };

            twai_transmit(&m1, 0);
            twai_transmit(&m2, 0);
        }
    }
}

// ---------------------------------------------------------------------------
// VE.Direct text protocol parser
// ---------------------------------------------------------------------------

static void process_vedirect_key_value(const char *key, const char *value)
{
    if (strcmp(key, "V") == 0) {
        float voltage = atoi(value) * 0.001f;
        battery_voltage_whole = (int)voltage;
        battery_voltage_decimal = (int)((voltage - battery_voltage_whole) * 100);
    } else if (strcmp(key, "VPV") == 0) {
        float voltage = atoi(value) * 0.001f;
        panel_voltage_whole = (int)voltage;
        panel_voltage_decimal = (int)((voltage - panel_voltage_whole) * 100);
    } else if (strcmp(key, "PPV") == 0) {
        int wattage = atoi(value);
        solar_wattage_lsb = wattage & 0xFF;
        solar_wattage_msb = (wattage >> 8) & 0xFF;
    } else if (strcmp(key, "I") == 0) {
        float current = atoi(value) * 0.001f;
        panel_current_whole = (int)current;
        panel_current_decimal = (int)((current - panel_current_whole) * 100);
        is_panel_current_negative = (current < 0) ? 1 : 0;
    } else if (strcmp(key, "LOAD") == 0) {
        ESP_LOGI(TAG, "Panel Load: %s", value);
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
        ESP_LOGW(TAG, "Error: %s", value);
    } else if (strcmp(key, "CS") == 0) {
        solar_status = atoi(value);
    } else if (strcmp(key, "FW") == 0) {
        ESP_LOGI(TAG, "Firmware Version: %s", value);
    } else if (strcmp(key, "PID") == 0) {
        ESP_LOGI(TAG, "Product ID: %s", value);
    } else if (strcmp(key, "SER#") == 0) {
        ESP_LOGI(TAG, "Serial Number: %s", value);
    } else if (strcmp(key, "HSDS") == 0) {
        ESP_LOGI(TAG, "Day Number: %s", value);
    } else if (strcmp(key, "MPPT") == 0) {
        ESP_LOGI(TAG, "Tracker Mode: %s", value);
    }
}

static void parse_vedirect_line(const char *line)
{
    const char *tab = strchr(line, '\t');
    if (tab == NULL) {
        return;
    }

    size_t key_len = tab - line;
    char key[32];
    if (key_len >= sizeof(key)) {
        return;
    }
    memcpy(key, line, key_len);
    key[key_len] = '\0';

    const char *value = tab + 1;
    process_vedirect_key_value(key, value);
}

// ---------------------------------------------------------------------------
// Main application
// ---------------------------------------------------------------------------

void app_main(void)
{
    ota_init();

    // Configure UART for Victron VE.Direct
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(VICTRON_UART, VICTRON_BUF_SIZE, VICTRON_BUF_SIZE, 0, NULL, 0);
    uart_param_config(VICTRON_UART, &uart_config);
    uart_set_pin(VICTRON_UART, VICTRON_TX_PIN, VICTRON_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "=== TrailCurrent Solstice ===");
    ESP_LOGI(TAG, "Hostname: %s", ota_get_hostname());

    // CAN runs in its own task so bus errors never block UART
    xTaskCreate(twai_task, "twai", 4096, NULL, 5, NULL);

    // Main task: read VE.Direct serial data from MPPT
    char line_buf[256];
    int line_pos = 0;
    uint8_t uart_byte;

    while (1) {
        int len = uart_read_bytes(VICTRON_UART, &uart_byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
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
