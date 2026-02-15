#include "bmw_shifter.h"
#include <string.h>

// CRC table from egs_utils.lua
static const uint8_t crc_table[256] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

// CRC start values for different CAN IDs (from egs_utils.lua)
static uint8_t get_crc_start_value(uint16_t can_id) {
    switch (can_id) {
        case 0x08F: return 0x75;
        case 0x0A0: return 0xBC;
        case 0x0A5: return 0x16;
        case 0x0A6: return 0xC2;
        case 0x0A7: return 0x8E;
        case 0x0B0: return 0x4C;
        case 0x0C2: return 0xD8;
        case 0x0D9: return 0x3E;
        case 0x0EF: return 0x98;
        case 0x12F: return 0x60;
        case 0x145: return 0x48;
        case 0x163: return 0xA0;
        case 0x173: return 0x13;
        case 0x197: return 0x62;  // Gear lever position
        case 0x199: return 0x8F;
        case 0x19A: return 0x17;
        case 0x19F: return 0xEF;
        case 0x1A1: return 0x77;
        case 0x1AF: return 0xB5;
        case 0x1E1: return 0x78;
        case 0x1FC: return 0x66;
        case 0x207: return 0x51;
        case 0x254: return 0xB8;
        case 0x297: return 0xDF;
        case 0x2C5: return 0xFC;
        case 0x2E0: return 0x5B;
        case 0x2ED: return 0x1D;
        case 0x302: return 0xC3;
        case 0x30B: return 0xBE;
        case 0x3A7: return 0x05;
        case 0x3F9: return 0x38;
        case 0x3FD: return 0xD7;  // Display gear indication
        default: return 0x00;
    }
}

// Packet counters for each CAN ID (0-14, cyclic)
static uint8_t pkt_counters[0x400] = {0};  // Support up to 0x3FF CAN IDs

/**
 * Update packet CRC and counter
 * Based on egs_utils.lua update_pkt() function
 * 
 * @param can_id CAN ID
 * @param data Pointer to data array (first byte will be CRC, second byte contains counter)
 * @param data_len Total length of data array
 */
void bmw_update_pkt(uint16_t can_id, uint8_t *data, uint8_t data_len) {
    uint8_t crc_start = get_crc_start_value(can_id);
    if (crc_start == 0x00 && can_id != 0x202) {  // 0x202 doesn't use CRC
        // Unsupported ID or no CRC needed
        return;
    }
    
    // Update counter (lower 4 bits of byte 1)
    uint8_t counter = pkt_counters[can_id];
    data[1] = (data[1] & 0xF0) | counter;
    pkt_counters[can_id] = (counter + 1) % 15;
    
    // Calculate CRC (skip if ID 0x202 - backlight doesn't use CRC)
    if (can_id == 0x202) {
        return;
    }
    
    uint8_t crc = crc_start;
    // Process bytes from index 1 to data_len-1 (CRC is in byte 0)
    for (int i = 1; i < data_len; i++) {
        crc = crc_table[crc ^ data[i]];
    }
    data[0] = crc;
}

/**
 * Get gear indication byte based on current gear
 * Based on gear-lever.lua GetIndication() function
 */
uint8_t bmw_get_gear_indication(bmw_gear_t gear) {
    switch (gear) {
        case GEAR_P: return GEAR_IND_P;
        case GEAR_R: return GEAR_IND_R;
        case GEAR_N: return GEAR_IND_N;
        case GEAR_D:
        case GEAR_M: return GEAR_IND_D;
        default: return 0x00;
    }
}

/**
 * Initialize shifter state
 */
void bmw_shifter_init(bmw_shifter_state_t *state) {
    memset(state, 0, sizeof(bmw_shifter_state_t));
    state->lever_position = LEVER_POS_CENTER_MIDDLE;
    state->park_button = PARK_BUTTON_NORMAL;
    state->current_gear = GEAR_P;
    state->prev_lever_position = LEVER_POS_CENTER_MIDDLE;
}

/**
 * Process lever position change and update gear state
 * Based on gear-lever.lua LeverPos() function
 */
void bmw_process_lever_position(bmw_shifter_state_t *state, uint8_t lever_pos, uint8_t park_button) {
    // Check park button first
    if (park_button == PARK_BUTTON_PRESSED) {
        state->current_gear = GEAR_P;
        state->lever_position = lever_pos;
        state->park_button = park_button;
        state->prev_lever_position = lever_pos;
        return;
    }
    
    uint8_t prev = state->prev_lever_position;
    
    // Handle lever up movements
    if (lever_pos == LEVER_POS_UP_1 && prev == LEVER_POS_CENTER_MIDDLE) {
        bmw_lever_up(state);
    } else if (lever_pos == LEVER_POS_UP_2 && prev == LEVER_POS_UP_1) {
        bmw_lever_up(state);
    }
    // Handle lever down movements
    else if (lever_pos == LEVER_POS_DOWN_1 && prev == LEVER_POS_CENTER_MIDDLE) {
        bmw_lever_down(state);
    } else if (lever_pos == LEVER_POS_DOWN_2 && prev == LEVER_POS_DOWN_1) {
        bmw_lever_down(state);
    }
    // Handle side movement (manual mode)
    else if (lever_pos == LEVER_POS_CENTER_SIDE && prev == LEVER_POS_CENTER_MIDDLE && state->current_gear == GEAR_D) {
        state->current_gear = GEAR_M;
    } else if (lever_pos == LEVER_POS_CENTER_MIDDLE && prev == LEVER_POS_CENTER_SIDE && state->current_gear == GEAR_M) {
        state->current_gear = GEAR_D;
    }
    // Handle manual gear changes
    else if (prev == LEVER_POS_CENTER_SIDE && state->current_gear == GEAR_M) {
        if (lever_pos == LEVER_POS_SIDE_UP) {
            if (state->manual_gear > 0) {
                state->manual_gear--;
            }
        } else if (lever_pos == LEVER_POS_SIDE_DOWN) {
            state->manual_gear++;
        }
    }
    // If lever returned to center, maintain current gear but update position
    else if (lever_pos == LEVER_POS_CENTER_MIDDLE && prev != LEVER_POS_CENTER_MIDDLE) {
        // Lever returned to center - maintain current gear
        // This handles the case when lever is released back to center
    }
    
    state->lever_position = lever_pos;
    state->park_button = park_button;
    
    // Only update prev_lever_position if lever actually moved
    if (lever_pos != prev) {
        state->prev_lever_position = lever_pos;
    }
}

/**
 * Handle lever up movement
 * Based on gear-lever.lua leverUp() function
 */
void bmw_lever_up(bmw_shifter_state_t *state) {
    if (state->current_gear == GEAR_P) {
        state->current_gear = GEAR_N;  // P -> N
    } else if (state->current_gear == GEAR_D) {
        state->current_gear = GEAR_N;  // D -> N
    } else if (state->current_gear == GEAR_N) {
        state->current_gear = GEAR_R;  // N -> R
    }
}

/**
 * Handle lever down movement
 * Based on gear-lever.lua leverDown() function
 */
void bmw_lever_down(bmw_shifter_state_t *state) {
    if (state->current_gear == GEAR_P) {
        state->current_gear = GEAR_D;  // P -> D
    } else if (state->current_gear == GEAR_N) {
        state->current_gear = GEAR_D;  // N -> D
    } else if (state->current_gear == GEAR_R) {
        state->current_gear = GEAR_N;  // R -> N
    }
}

