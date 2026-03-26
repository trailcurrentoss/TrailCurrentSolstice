#pragma once

#include "driver/twai.h"

#define CAN_RX_PIN 16
#define CAN_TX_PIN 15
#define CAN_POLLING_RATE_MS 100

typedef void (*can_rx_callback_t)(twai_message_t *message);

void can_helper_setup(can_rx_callback_t callback);
void can_helper_loop(void);
