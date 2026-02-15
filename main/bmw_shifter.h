#ifndef BMW_SHIFTER_H
#define BMW_SHIFTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// CAN ID definitions (from can_ids.lua)
#define CAN_ID_GEAR_LEVER_POSITION      0x197  // 407 - Position from shifter
#define CAN_ID_GEAR_LEVER_HEARTBEAT    0x55E  // 1374 - Heartbeat from shifter
#define CAN_ID_DISPLAY_GEAR            0x3FD  // 1021 - Display gear indication (to shifter)
#define CAN_ID_BACKLIGHT               0x202  // 514 - Backlight control (to shifter)

// Lever position values (from gear-lever.lua)
#define LEVER_POS_CENTER_MIDDLE        0x0E  // Centre middle
#define LEVER_POS_UP_1                 0x1E  // Pushed up (1 notch)
#define LEVER_POS_UP_2                 0x2E  // Pushed up (2 notches)
#define LEVER_POS_DOWN_1               0x3E  // Pushed down (1 notch)
#define LEVER_POS_DOWN_2               0x4E  // Pushed down (2 notches)
#define LEVER_POS_CENTER_SIDE          0x7E  // Centre side (manual mode)
#define LEVER_POS_SIDE_UP              0x5E  // Pushed side and up
#define LEVER_POS_SIDE_DOWN            0x6E  // Pushed side and down

// Park button values
#define PARK_BUTTON_NORMAL             0xC0  // Normal state
#define PARK_BUTTON_PRESSED            0xD5  // Button pressed

// Gear enumeration
typedef enum {
    GEAR_P = 0,  // Park
    GEAR_R = 1,  // Reverse
    GEAR_N = 2,  // Neutral
    GEAR_D = 3,  // Drive
    GEAR_M = 4   // Manual
} bmw_gear_t;

// Gear indication values (for display)
#define GEAR_IND_P                     0x20  // Park indication
#define GEAR_IND_R                     0x40  // Reverse indication
#define GEAR_IND_N                     0x60  // Neutral indication
#define GEAR_IND_D                     0x81  // Drive/Manual indication
#define GEAR_IND_FLASH                 0x08  // Flash bit (add to base value)

// Backlight range
#define BACKLIGHT_MIN                  0
#define BACKLIGHT_MAX                  254
#define BACKLIGHT_DEFAULT              254

// Message timing (in milliseconds)
#define TIMING_GEAR_DISPLAY_MS         100   // Display gear message interval
#define TIMING_BACKLIGHT_MS            1000  // Backlight message interval
#define TIMING_HEARTBEAT_MS            640   // Heartbeat message interval
#define TIMING_GEAR_LEVER_RX_MS        30    // Expected gear lever position message interval

// Shifter state structure
typedef struct {
    uint8_t lever_position;      // Current lever position (0x0E, 0x1E, etc.)
    uint8_t park_button;          // Park button state (0xC0 or 0xD5)
    bmw_gear_t current_gear;      // Calculated current gear (P/R/N/D/M)
    uint8_t manual_gear;          // Manual gear number (for M mode)
    uint8_t prev_lever_position;  // Previous lever position for state machine
} bmw_shifter_state_t;

// CAN message structure for gear lever position (ID 0x197)
// Byte 0: CRC (calculated)
// Byte 1: Counter (lower 4 bits) | other bits
// Byte 2: Lever position (0x0E, 0x1E, etc.)
// Byte 3: Park button (0xC0 or 0xD5)
typedef struct {
    uint8_t crc;
    uint8_t counter_and_flags;
    uint8_t lever_position;
    uint8_t park_button;
} __attribute__((packed)) gear_lever_position_msg_t;

// CAN message structure for gear display (ID 0x3FD)
// Byte 0: CRC (calculated)
// Byte 1: Counter (lower 4 bits) | other bits
// Byte 2: Gear indication (0x20=P, 0x40=R, 0x60=N, 0x81=D/M)
// Byte 3: 0x0C (constant)
// Byte 4: 0xFF (constant)
typedef struct {
    uint8_t crc;
    uint8_t counter_and_flags;
    uint8_t gear_indication;
    uint8_t reserved1;
    uint8_t reserved2;
} __attribute__((packed)) gear_display_msg_t;

// CAN message structure for backlight (ID 0x202)
// Byte 0: Backlight level (0-254)
// Byte 1: 0x00 (constant)
typedef struct {
    uint8_t backlight_level;
    uint8_t reserved;
} __attribute__((packed)) backlight_msg_t;

// CAN message structure for heartbeat (ID 0x55E)
// Byte 0-3: 0x00
// Byte 4: Bus identifier (0x01 for PT-CAN, 0x02 for PT-CAN2)
// Byte 5-6: 0x00
// Byte 7: 0x5E
typedef struct {
    uint8_t reserved[4];
    uint8_t bus_id;
    uint8_t reserved2[2];
    uint8_t magic;
} __attribute__((packed)) heartbeat_msg_t;

// Function declarations
void bmw_update_pkt(uint16_t can_id, uint8_t *data, uint8_t data_len);
uint8_t bmw_get_gear_indication(bmw_gear_t gear);
void bmw_shifter_init(bmw_shifter_state_t *state);
void bmw_process_lever_position(bmw_shifter_state_t *state, uint8_t lever_pos, uint8_t park_button);
void bmw_lever_up(bmw_shifter_state_t *state);
void bmw_lever_down(bmw_shifter_state_t *state);

#ifdef __cplusplus
}
#endif

#endif // BMW_SHIFTER_H

