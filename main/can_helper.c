#include "can_helper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "can_helper";

static bool driver_installed = false;
static can_rx_callback_t rx_callback = NULL;

void can_helper_setup(can_rx_callback_t callback)
{
    rx_callback = callback;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG, "Driver installed");
    } else {
        ESP_LOGE(TAG, "Failed to install driver");
        return;
    }

    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "Driver started");
    } else {
        ESP_LOGE(TAG, "Failed to start driver");
        return;
    }

    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        ESP_LOGI(TAG, "CAN alerts reconfigured");
    } else {
        ESP_LOGE(TAG, "Failed to reconfigure alerts");
        return;
    }

    driver_installed = true;
}

void can_helper_loop(void)
{
    if (!driver_installed) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(CAN_POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
        ESP_LOGW(TAG, "Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
        ESP_LOGW(TAG, "Alert: Bus error. Count: %lu", twaistatus.bus_error_count);
    }

    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
        twai_message_t message;
        while (twai_receive(&message, 0) == ESP_OK) {
            if (rx_callback != NULL) {
                rx_callback(&message);
            }
        }
    }
}
