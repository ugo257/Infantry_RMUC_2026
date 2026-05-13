#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <stdint.h>
#include "can.h"

typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    uint32_t tx_mailbox;
    uint8_t rx_data[8];
    uint8_t tx_data[8];
} CAN_Handler;

#endif // CAN_HANDLER_H
