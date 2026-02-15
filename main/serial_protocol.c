#include "serial_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Simple JSON serialization (without external library)
void serial_send_can_rx(uint16_t can_id, const uint8_t *data, uint8_t dlc) {
    char json[256];
    int len = snprintf(json, sizeof(json),
        "{\"type\":\"can_rx\",\"id\":%u,\"data\":[",
        can_id);
    
    for (int i = 0; i < dlc && i < 8; i++) {
        len += snprintf(json + len, sizeof(json) - len, "%s0x%02X", 
                       i > 0 ? "," : "", data[i]);
    }
    
    len += snprintf(json + len, sizeof(json) - len, "],\"dlc\":%u}\n", dlc);
    
    printf("%s", json);
    fflush(stdout);
}

void serial_send_shifter_state(const bmw_shifter_state_t *state) {
    const char *gear_str;
    switch (state->current_gear) {
        case GEAR_P: gear_str = "P"; break;
        case GEAR_R: gear_str = "R"; break;
        case GEAR_N: gear_str = "N"; break;
        case GEAR_D: gear_str = "D"; break;
        case GEAR_M: gear_str = "M"; break;
        default: gear_str = "Unknown"; break;
    }
    
    printf("{\"type\":\"shifter_state\",\"gear\":\"%s\",\"lever_pos\":0x%02X,\"park\":%s,\"manual\":%u}\n",
           gear_str,
           state->lever_position,
           state->park_button == PARK_BUTTON_PRESSED ? "true" : "false",
           state->manual_gear);
    fflush(stdout);
}

// Simple JSON parser (basic implementation)
bool serial_process_received_data(const char *json_str, uint8_t *backlight_level, bmw_gear_t *gear_indication,
                                  int *hid_button, int *hid_action) {
    if (backlight_level == NULL || gear_indication == NULL || hid_button == NULL || hid_action == NULL) {
        return false;
    }
    
    // Simple string matching for JSON parsing (without external library)
    if (strstr(json_str, "\"type\":\"set_backlight\"") != NULL) {
        // Parse backlight level
        const char *level_str = strstr(json_str, "\"level\":");
        if (level_str != NULL) {
            int level = atoi(level_str + 8);
            if (level >= BACKLIGHT_MIN && level <= BACKLIGHT_MAX) {
                *backlight_level = (uint8_t)level;
                return true;
            }
        }
    } else if (strstr(json_str, "\"type\":\"set_gear_indication\"") != NULL) {
        // Parse gear indication
        const char *gear_str = strstr(json_str, "\"gear\":\"");
        if (gear_str != NULL) {
            char gear_char = gear_str[9];
            switch (gear_char) {
                case 'P': *gear_indication = GEAR_P; break;
                case 'R': *gear_indication = GEAR_R; break;
                case 'N': *gear_indication = GEAR_N; break;
                case 'D': *gear_indication = GEAR_D; break;
                case 'M': *gear_indication = GEAR_M; break;
                default: return false;
            }
            return true;
        }
    } else if (strstr(json_str, "\"type\":\"hid_button\"") != NULL) {
        // Parse HID button command
        const char *button_str = strstr(json_str, "\"button\":\"");
        const char *action_str = strstr(json_str, "\"action\":\"");
        
        if (button_str != NULL && action_str != NULL) {
            char button_char = button_str[10];
            
            // Parse button
            int button = -1;
            switch (button_char) {
                case 'P': button = 0; break; // HID_BUTTON_P
                case 'N': button = 1; break; // HID_BUTTON_N
                case 'R': button = 2; break; // HID_BUTTON_R
                case 'D': button = 3; break; // HID_BUTTON_D
                case 'M': button = 4; break; // HID_BUTTON_M
                case '+': button = 5; break; // HID_BUTTON_PLUS
                case '-': button = 6; break; // HID_BUTTON_MINUS
                case 'U': 
                case 'u': button = 7; break; // HID_BUTTON_UNLOCK
                default: return false;
            }
            
            // Parse action
            int action = -1;
            if (strncmp(action_str + 10, "press", 5) == 0) {
                action = 0; // HID_ACTION_PRESS
            } else if (strncmp(action_str + 10, "release", 7) == 0) {
                action = 1; // HID_ACTION_RELEASE
            } else {
                return false;
            }
            
            *hid_button = button;
            *hid_action = action;
            return true;
        }
    }
    return false;
}

