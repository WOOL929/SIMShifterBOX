#include "usb_hid.h"
#include "esp_log.h"
#include "esp_err.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "class/hid/hid_device.h"
#include <string.h>

static const char *TAG = "USB_HID";

// HID ready flag
static bool hid_ready = false;

// HID Gamepad Report Descriptor
// Custom descriptor with 32 buttons support (instead of standard 16)
// Report ID must be 1-255 (Windows requirement)
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(1))
};

// Note: Standard TUD_HID_REPORT_DESC_GAMEPAD uses uint16_t for buttons (16 buttons)
// We'll override the report structure to use uint32_t for 32 buttons
// The descriptor format is compatible, we just send more button bits

// String descriptors
const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "BMW Shifter",         // 1: Manufacturer
    "Shifter Gamepad",     // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Gamepad Interface",   // 4: HID
};

// Configuration descriptor
#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, CFG_TUD_HID_EP_BUFSIZE, 10),
};

// TinyUSB HID callbacks
// Invoked when received GET HID REPORT DESCRIPTOR request
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void) instance;
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, 
                                hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    return 0;
}

// Invoked when received SET_REPORT control request or received data on OUT endpoint
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, 
                           hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;
    // Gamepad doesn't need to handle SET_REPORT
}

// tud_mount_cb and tud_umount_cb are already defined in espressif__esp_tinyusb component
// Do not redefine them here to avoid multiple definition errors

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    ESP_LOGI(TAG, "USB suspended");
}

void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB resumed");
}

bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate)
{
    (void) instance;
    (void) idle_rate;
    return true;
}

// Custom gamepad report structure
// Standard gamepad supports 32 buttons via uint32_t
// Format: 6 axes (int8_t) + hat (uint8_t) + buttons (uint32_t) = 11 bytes
// Report ID is passed separately to tud_hid_n_report
typedef struct {
    int8_t x, y, z, rz, rx, ry;  // 6 analog axes (signed)
    uint8_t hat;            // Hat switch (0-7, 8 = center)
    uint32_t buttons;       // 32 buttons as bitfield (bit 0 = button 1, bit 1 = button 2, etc.)
} __attribute__((packed)) custom_gamepad_report_t;

static custom_gamepad_report_t gamepad_report = {0};

esp_err_t usb_hid_init(void)
{
    ESP_LOGI(TAG, "Initializing USB HID Gamepad...");
    
    // Configure TinyUSB using default config
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    
    // Set custom descriptors
    tusb_cfg.descriptor.device = NULL; // Use default device descriptor
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = hid_configuration_descriptor;
#endif // TUD_OPT_HIGH_SPEED
    
    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TinyUSB: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize gamepad report
    memset(&gamepad_report, 0, sizeof(custom_gamepad_report_t));
    gamepad_report.hat = 8; // Center position
    hid_ready = false;
    
    ESP_LOGI(TAG, "USB HID Gamepad initialization started, waiting for host connection...");
    return ESP_OK;
}

bool usb_hid_is_ready(void)
{
    return tud_mounted() && tud_hid_n_ready(0);
}

// Map button to gamepad button number (1-32)
// Button numbers: N=1, R=2, D=3, M=4, P=5, +=30, -=31
static uint8_t button_to_gamepad_number(hid_button_t button)
{
    switch (button) {
        case HID_BUTTON_N: return 1;      // Neutral -> Button 1
        case HID_BUTTON_R: return 2;      // Reverse -> Button 2
        case HID_BUTTON_D: return 3;      // Drive -> Button 3
        case HID_BUTTON_M: return 4;      // Manual -> Button 4
        case HID_BUTTON_P: return 5;      // Park -> Button 5
        case HID_BUTTON_PLUS: return 30;  // Shift Up -> Button 30
        case HID_BUTTON_MINUS: return 31; // Shift Down -> Button 31
        case HID_BUTTON_UNLOCK: return 32; // Unlock -> Button 32
        default: return 0; // Invalid
    }
}

esp_err_t usb_hid_send_gamepad_report(void)
{
    if (!usb_hid_is_ready()) {
        ESP_LOGW(TAG, "USB HID not ready");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send gamepad report using tud_hid_n_report with custom structure
    // Report ID 1 matches the descriptor
    // This allows us to send 32 buttons (uint32_t) instead of 16 (uint16_t)
    if (!tud_hid_n_report(0, 1, &gamepad_report, sizeof(custom_gamepad_report_t))) {
        ESP_LOGE(TAG, "Failed to send gamepad report");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t usb_hid_send_button(hid_button_t button, hid_action_t action)
{
    if (!usb_hid_is_ready()) {
        ESP_LOGW(TAG, "USB HID not ready");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t button_num = button_to_gamepad_number(button);
    if (button_num == 0 || button_num > 32) {
        ESP_LOGE(TAG, "Invalid button: %d", button);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Button numbers are 1-32, but bitfield uses 0-31 (button 1 = bit 0, button 2 = bit 1, etc.)
    uint8_t button_bit = button_num - 1;
    
    // Update gamepad report button state
    if (action == HID_ACTION_PRESS) {
        gamepad_report.buttons |= (1UL << button_bit);
        ESP_LOGI(TAG, "HID: Button %d pressed (bit %d)", button_num, button_bit);
    } else {
        gamepad_report.buttons &= ~(1UL << button_bit);
        ESP_LOGI(TAG, "HID: Button %d released (bit %d)", button_num, button_bit);
    }
    
    // Send updated report
    return usb_hid_send_gamepad_report();
}

esp_err_t usb_hid_send_key(uint8_t keycode, bool press)
{
    // This function is kept for compatibility but maps to gamepad buttons
    // Map keycodes to buttons if needed, or use usb_hid_send_button directly
    ESP_LOGW(TAG, "usb_hid_send_key is deprecated, use usb_hid_send_button instead");
    return ESP_OK;
}

// USB HID task (must be called periodically)
void usb_hid_task(void)
{
    tud_task(); // TinyUSB device task - must be called frequently
}

