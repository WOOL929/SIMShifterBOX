#ifndef USB_HID_H
#define USB_HID_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// HID button definitions for Gamepad
typedef enum {
    HID_BUTTON_P = 0,      // Park (Button 0)
    HID_BUTTON_N = 1,      // Neutral (Button 1)
    HID_BUTTON_R = 2,      // Reverse (Button 2)
    HID_BUTTON_D = 3,      // Drive (Button 3)
    HID_BUTTON_M = 4,      // Manual (Button 4)
    HID_BUTTON_PLUS = 5,   // Shift Up (+) (Button 5)
    HID_BUTTON_MINUS = 6,  // Shift Down (-) (Button 6)
    HID_BUTTON_UNLOCK = 7  // Unlock (Button 7)
} hid_button_t;

// HID action types
typedef enum {
    HID_ACTION_PRESS = 0,
    HID_ACTION_RELEASE = 1
} hid_action_t;

// Function declarations
esp_err_t usb_hid_init(void);
bool usb_hid_is_ready(void);
esp_err_t usb_hid_send_button(hid_button_t button, hid_action_t action);
esp_err_t usb_hid_send_gamepad_report(void);
esp_err_t usb_hid_send_key(uint8_t keycode, bool press); // Deprecated, use usb_hid_send_button
void usb_hid_task(void);

#ifdef __cplusplus
}
#endif

#endif // USB_HID_H

