#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "can_helper.h"

static const char *TAG = "solstice";

#define VICTRON_RX_PIN 1
#define VICTRON_TX_PIN 2
#define VICTRON_UART   UART_NUM_1
#define VICTRON_BUF_SIZE 1024

#define CAN_SEND_MESSAGE_SOLAR01_IDENTIFIER 0x2C
#define CAN_SEND_MESSAGE_SOLAR02_IDENTIFIER 0x2D
#define CAN_RECV_LOAD_CONTROL_IDENTIFIER    0x2E

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

static const uint32_t can_status_period_ms = 33;

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

static void on_can_rx(twai_message_t *message)
{
    if (message->identifier == CAN_RECV_LOAD_CONTROL_IDENTIFIER && message->data_length_code >= 1) {
        uint8_t load_state = message->data[0];
        ESP_LOGI(TAG, "CAN load control received: %d", load_state);
        vedirect_set_load(load_state);
    }
}

static void send_mppt_message(void)
{
    twai_message_t message = {0};
    message.identifier = CAN_SEND_MESSAGE_SOLAR01_IDENTIFIER;
    message.data_length_code = 7;
    message.data[0] = panel_voltage_whole;
    message.data[1] = panel_voltage_decimal;
    message.data[2] = solar_wattage_msb;
    message.data[3] = solar_wattage_lsb;
    message.data[4] = battery_voltage_whole;
    message.data[5] = battery_voltage_decimal;
    message.data[6] = solar_status;

    if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
        ESP_LOGD(TAG, "Message 1 queued for transmission");
    } else {
        ESP_LOGW(TAG, "Failed to queue message 1 for transmission");
    }
}

static void send_mppt_message2(void)
{
    twai_message_t message = {0};
    message.identifier = CAN_SEND_MESSAGE_SOLAR02_IDENTIFIER;
    message.data_length_code = 3;
    message.data[0] = is_panel_current_negative;
    message.data[1] = panel_current_whole;
    message.data[2] = panel_current_decimal;

    if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
        ESP_LOGD(TAG, "Message 2 queued for transmission");
    } else {
        ESP_LOGW(TAG, "Failed to queue message 2 for transmission");
    }
}

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
    // VE.Direct TEXT protocol: lines are "KEY\tVALUE"
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

void app_main(void)
{
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

    can_helper_setup(on_can_rx);

    // Line buffer for VE.Direct serial data
    char line_buf[256];
    int line_pos = 0;

    uint64_t last_can_time_us = 0;

    while (1) {
        uint64_t now_us = esp_timer_get_time();
        if ((now_us - last_can_time_us) >= (can_status_period_ms * 1000)) {
            // Read all available VE.Direct data
            uint8_t byte;
            while (uart_read_bytes(VICTRON_UART, &byte, 1, 0) > 0) {
                if (byte == '\n' || byte == '\r') {
                    if (line_pos > 0) {
                        line_buf[line_pos] = '\0';
                        ESP_LOGD(TAG, "VE.Direct: %s", line_buf);
                        parse_vedirect_line(line_buf);
                        line_pos = 0;
                    }
                } else if (line_pos < (int)(sizeof(line_buf) - 1)) {
                    line_buf[line_pos++] = (char)byte;
                }
            }

            can_helper_loop();
            send_mppt_message();
            send_mppt_message2();
            last_can_time_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
