#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "bmw_shifter.h"

#ifdef __cplusplus
extern "C" {
#endif

// Serial protocol message types
typedef enum {
    SERIAL_MSG_CAN_RX = 0,           // CAN message received
    SERIAL_MSG_SHIFTER_STATE,        // Shifter state update
    SERIAL_MSG_SET_BACKLIGHT,        // Set backlight level (from app)
    SERIAL_MSG_SET_GEAR_INDICATION   // Set gear indication (from app)
} serial_msg_type_t;

// Serial message structure for CAN RX
typedef struct {
    uint16_t can_id;
    uint8_t data[8];
    uint8_t dlc;
    uint32_t timestamp_ms;
} serial_can_rx_msg_t;

// Serial message structure for shifter state
typedef struct {
    bmw_gear_t gear;
    uint8_t lever_pos;
    bool park_pressed;
    uint8_t manual_gear;
} serial_shifter_state_msg_t;

// Serial message structure for set backlight
typedef struct {
    uint8_t level;
} serial_set_backlight_msg_t;

// Serial message structure for set gear indication
typedef struct {
    bmw_gear_t gear;
} serial_set_gear_indication_msg_t;

// Function declarations
void serial_send_can_rx(uint16_t can_id, const uint8_t *data, uint8_t dlc);
void serial_send_shifter_state(const bmw_shifter_state_t *state);
bool serial_process_received_data(const char *json_str, uint8_t *backlight_level, bmw_gear_t *gear_indication, 
                                  int *hid_button, int *hid_action);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_PROTOCOL_H

