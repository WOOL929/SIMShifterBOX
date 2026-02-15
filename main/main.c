#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "bmw_shifter.h"
#include "serial_protocol.h"
#include "usb_hid.h"

static const char *TAG = "BMW_SHIFTER";

// Global state
static bmw_shifter_state_t shifter_state;
static bmw_shifter_state_t prev_shifter_state;  // Previous state for change detection
static uint8_t backlight_level = BACKLIGHT_DEFAULT;
static bool shifter_connected = false;
static bool shifter_state_initialized = false;  // Track if we've seen first state update
static uint32_t last_heartbeat_time = 0;  // Last time we received message from shifter
static uint8_t current_gear_indication = 0;  // Current gear indication value (0x20=P, 0x40=R, 0x60=N, 0x80=D, 0x81=M/S)

// Button press timing - track when buttons were pressed for 80ms release
static uint32_t button_press_time = 0;  // Time when button was pressed (0 = no button pressed, UINT32_MAX = hold button)
static hid_button_t pressed_button = HID_BUTTON_P;  // Currently pressed button
static bool button_is_pressed = false;  // Flag indicating if button is currently pressed
static bool button_should_hold = false;  // Flag indicating if button should be held (for R button)

// CAN message buffers
static gear_display_msg_t gear_display_msg = {0, 0, 0, 0x0C, 0xFF};
static backlight_msg_t backlight_msg = {BACKLIGHT_DEFAULT, 0x00};
static heartbeat_msg_t heartbeat_msg = {{0, 0, 0, 0}, 0x02, {0, 0}, 0x5E};

// Timer handles
static TimerHandle_t timer_gear_display = NULL;
static TimerHandle_t timer_backlight = NULL;
static TimerHandle_t timer_heartbeat = NULL;

// Update HID buttons based on gear indication value
// This function is called periodically from hid_update_task
static void update_hid_buttons_from_gear_indication(void) {
    if (!usb_hid_is_ready()) {
        return;  // USB HID not ready, skip
    }
    
    static uint8_t prev_gear_indication = 0;
    
    // Check if gear indication changed
    if (current_gear_indication == prev_gear_indication) {
        // If R is active and button should be held, ensure it's still pressed
        if (current_gear_indication == 0x40 && button_is_pressed && pressed_button == HID_BUTTON_R) {
            // R is still active, keep button pressed (already pressed, no action needed)
            return;
        }
        return;  // No change, don't press buttons again
    }
    
    // If button is currently pressed, release it first (unless it's R and we're switching to R)
    if (button_is_pressed && !(current_gear_indication == 0x40 && pressed_button == HID_BUTTON_R)) {
        usb_hid_send_button(pressed_button, HID_ACTION_RELEASE);
        button_is_pressed = false;
        button_press_time = 0;
        button_should_hold = false;
    }
    
    // Press button based on gear indication
    switch (current_gear_indication) {
        case 0x20:  // P - don't press any buttons
            ESP_LOGI(TAG, "HID: Gear indication P (0x20) - no buttons");
            break;
            
        case 0x40:  // R - press button 2 and hold while 0x40 is active
            usb_hid_send_button(HID_BUTTON_R, HID_ACTION_PRESS);
            pressed_button = HID_BUTTON_R;
            button_is_pressed = true;
            button_press_time = UINT32_MAX;  // Special value to indicate hold
            button_should_hold = true;
            ESP_LOGI(TAG, "HID: Gear indication R (0x40) - button 2 pressed and held");
            break;
            
        case 0x60:  // N - press button 1 for 80ms
            usb_hid_send_button(HID_BUTTON_N, HID_ACTION_PRESS);
            pressed_button = HID_BUTTON_N;
            button_is_pressed = true;
            button_press_time = xTaskGetTickCount();
            button_should_hold = false;
            ESP_LOGI(TAG, "HID: Gear indication N (0x60) - button 1 pressed for 80ms");
            break;
            
        case 0x80:  // D - press button 3 for 80ms
            usb_hid_send_button(HID_BUTTON_D, HID_ACTION_PRESS);
            pressed_button = HID_BUTTON_D;
            button_is_pressed = true;
            button_press_time = xTaskGetTickCount();
            button_should_hold = false;
            ESP_LOGI(TAG, "HID: Gear indication D (0x80) - button 3 pressed for 80ms");
            break;
            
        case 0x81:  // M/S (lever moved to side) - don't press any buttons
            ESP_LOGI(TAG, "HID: Gear indication M/S (0x81) - no buttons");
            break;
            
        default:
            ESP_LOGW(TAG, "HID: Unknown gear indication 0x%02X", current_gear_indication);
            break;
    }
    
    prev_gear_indication = current_gear_indication;
}

// Update +/- buttons based on current lever position (called periodically)
static void update_hid_plus_minus_buttons(void) {
    if (!usb_hid_is_ready()) {
        return;  // USB HID not ready, skip
    }
    
    static uint8_t last_lever_pos = 0;
    static bool last_was_m_mode = false;
    
    // Check if we're in M mode
    bool is_m_mode = (shifter_state.current_gear == GEAR_M);
    
    // If lever position changed or mode changed, update buttons
    if (shifter_state.lever_position != last_lever_pos || is_m_mode != last_was_m_mode) {
        // If button is currently pressed, release it first
        if (button_is_pressed && (pressed_button == HID_BUTTON_PLUS || pressed_button == HID_BUTTON_MINUS)) {
            usb_hid_send_button(pressed_button, HID_ACTION_RELEASE);
            button_is_pressed = false;
            button_press_time = 0;
        }
        
        // Press buttons based on current lever position in M mode (will be released after 80ms)
        if (is_m_mode) {
            if (shifter_state.lever_position == LEVER_POS_SIDE_UP) {
                // Lever moved up in M mode - press + button (button 30) for 80ms
                usb_hid_send_button(HID_BUTTON_PLUS, HID_ACTION_PRESS);
                pressed_button = HID_BUTTON_PLUS;
                button_is_pressed = true;
                button_press_time = xTaskGetTickCount();
                ESP_LOGI(TAG, "HID: Lever up in M mode - button 30 pressed for 80ms");
            } else if (shifter_state.lever_position == LEVER_POS_SIDE_DOWN) {
                // Lever moved down in M mode - press - button (button 31) for 80ms
                usb_hid_send_button(HID_BUTTON_MINUS, HID_ACTION_PRESS);
                pressed_button = HID_BUTTON_MINUS;
                button_is_pressed = true;
                button_press_time = xTaskGetTickCount();
                ESP_LOGI(TAG, "HID: Lever down in M mode - button 31 pressed for 80ms");
            }
            // If lever is in center position, buttons are not pressed
        }
        
        last_lever_pos = shifter_state.lever_position;
        last_was_m_mode = is_m_mode;
    }
}

// Update HID buttons based on shifter state changes (for initialization)
static void update_hid_buttons_from_shifter(void) {
    // Skip if this is the first state update (no previous state to compare)
    if (!shifter_state_initialized) {
        shifter_state_initialized = true;
        memcpy(&prev_shifter_state, &shifter_state, sizeof(bmw_shifter_state_t));
        return;
    }
    
    // Update previous state (this function is called from can_rx_task to track state changes)
    memcpy(&prev_shifter_state, &shifter_state, sizeof(bmw_shifter_state_t));
}

// HID update task - periodically updates HID buttons based on current state
void hid_update_task(void *pvParameters) {
    const uint32_t BUTTON_PRESS_DURATION_MS = 80;  // Button press duration
    
    while (1) {
        if (usb_hid_is_ready() && shifter_state_initialized) {
            // Check if button needs to be released (80ms after press, but not if it should be held)
            if (button_is_pressed && button_press_time != 0 && button_press_time != UINT32_MAX && !button_should_hold) {
                uint32_t now = xTaskGetTickCount();
                if ((now - button_press_time) >= pdMS_TO_TICKS(BUTTON_PRESS_DURATION_MS)) {
                    // Release button after 80ms (only if not R button)
                    usb_hid_send_button(pressed_button, HID_ACTION_RELEASE);
                    button_is_pressed = false;
                    button_press_time = 0;
                    button_should_hold = false;
                    ESP_LOGI(TAG, "HID: Button released after 80ms");
                }
            }
            
            // Update HID buttons based on gear indication and lever position
            update_hid_buttons_from_gear_indication();
            update_hid_plus_minus_buttons();
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Update every 10ms for precise timing
    }
}

// Timer callbacks
void timer_gear_display_callback(TimerHandle_t xTimer) {
    // Get gear indication based on current gear and lever position
    uint8_t gear_ind = bmw_get_gear_indication(shifter_state.current_gear);
    
    // If in M mode and lever is moved to side, use 0x81 (M/S)
    if (shifter_state.current_gear == GEAR_M && 
        shifter_state.lever_position == LEVER_POS_CENTER_SIDE) {
        gear_ind = 0x81;  // M/S mode
    }
    
    gear_display_msg.gear_indication = gear_ind;
    current_gear_indication = gear_ind;  // Store current indication for HID logic
    
    bmw_update_pkt(CAN_ID_DISPLAY_GEAR, (uint8_t*)&gear_display_msg, sizeof(gear_display_msg));
    
    twai_message_t msg;
    msg.identifier = CAN_ID_DISPLAY_GEAR;
    msg.flags = 0;
    msg.data_length_code = sizeof(gear_display_msg);
    memcpy(msg.data, &gear_display_msg, sizeof(gear_display_msg));
    
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send gear display: %s", esp_err_to_name(ret));
    }
    
    // Note: HID button updates are handled in can_rx_task to avoid stack overflow in timer callback
}

void timer_backlight_callback(TimerHandle_t xTimer) {
    backlight_msg.backlight_level = backlight_level;
    
    twai_message_t msg;
    msg.identifier = CAN_ID_BACKLIGHT;
    msg.flags = 0;
    msg.data_length_code = sizeof(backlight_msg);
    memcpy(msg.data, &backlight_msg, sizeof(backlight_msg));
    
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send backlight: %s", esp_err_to_name(ret));
    }
}

void timer_heartbeat_callback(TimerHandle_t xTimer) {
    twai_message_t msg;
    msg.identifier = CAN_ID_GEAR_LEVER_HEARTBEAT;
    msg.flags = 0;
    msg.data_length_code = sizeof(heartbeat_msg);
    memcpy(msg.data, &heartbeat_msg, sizeof(heartbeat_msg));
    
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send heartbeat: %s", esp_err_to_name(ret));
    }
}

// CAN receive task
void can_rx_task(void *pvParameters) {
    twai_message_t rx_msg;
    static uint32_t last_can_log_time = 0;
    static uint32_t last_state_send_time = 0;
    const uint32_t CAN_LOG_INTERVAL_MS = 500;  // Log CAN messages every 500ms max
    const uint32_t STATE_SEND_INTERVAL_MS = 100;  // Send state updates every 100ms max
    
    while (1) {
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
        
        if (ret == ESP_OK) {
            uint32_t now = xTaskGetTickCount();
            
            // Send CAN message to serial port only for important IDs or with throttling
            bool should_log = false;
            if (rx_msg.identifier == CAN_ID_GEAR_LEVER_POSITION) {
                // Always log gear lever position messages
                should_log = true;
            } else if ((now - last_can_log_time) > pdMS_TO_TICKS(CAN_LOG_INTERVAL_MS)) {
                // Log other messages with throttling
                should_log = true;
                last_can_log_time = now;
            }
            
            if (should_log) {
                serial_send_can_rx(rx_msg.identifier, rx_msg.data, rx_msg.data_length_code);
            }
            
            // Process gear lever position message (ID 0x197)
            if (rx_msg.identifier == CAN_ID_GEAR_LEVER_POSITION && rx_msg.data_length_code >= 4) {
                uint8_t lever_pos = rx_msg.data[2];
                uint8_t park_button = rx_msg.data[3];
                
                // Update shifter state
                bmw_process_lever_position(&shifter_state, lever_pos, park_button);
                
                // Update HID buttons based on state changes (track state only, HID updates in separate task)
                update_hid_buttons_from_shifter();
                
                // Send updated state to serial port with throttling
                if ((now - last_state_send_time) > pdMS_TO_TICKS(STATE_SEND_INTERVAL_MS)) {
                    serial_send_shifter_state(&shifter_state);
                    last_state_send_time = now;
                }
                
                shifter_connected = true;
                last_heartbeat_time = now;
                ESP_LOGI(TAG, "Gear lever: pos=0x%02X park=%s gear=%d",
                         lever_pos,
                         park_button == PARK_BUTTON_PRESSED ? "pressed" : "normal",
                         shifter_state.current_gear);
            }
            // Process heartbeat from shifter (ID 0x55E)
            else if (rx_msg.identifier == CAN_ID_GEAR_LEVER_HEARTBEAT) {
                shifter_connected = true;
                last_heartbeat_time = now;
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Timeout is normal, continue
        } else {
            ESP_LOGE(TAG, "CAN receive error: %s", esp_err_to_name(ret));
        }
    }
}

// Serial receive task (for commands from app)
void serial_rx_task(void *pvParameters) {
    uint8_t buffer[256];
    int len;
    
    while (1) {
        len = uart_read_bytes(UART_NUM_0, buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            buffer[len] = '\0';
            
            // Process JSON command
            uint8_t new_backlight = backlight_level;
            bmw_gear_t new_gear = shifter_state.current_gear;
            int hid_button = -1;
            int hid_action = -1;
            
            if (serial_process_received_data((char*)buffer, &new_backlight, &new_gear, &hid_button, &hid_action)) {
                if (new_backlight != backlight_level) {
                    backlight_level = new_backlight;
                    ESP_LOGI(TAG, "Backlight level set to %u", backlight_level);
                }
                
                // Process HID button command
                if (hid_button >= 0 && hid_action >= 0) {
                    esp_err_t ret = usb_hid_send_button((hid_button_t)hid_button, (hid_action_t)hid_action);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "HID button %d %s", hid_button, 
                                hid_action == 0 ? "pressed" : "released");
                    } else {
                        ESP_LOGW(TAG, "Failed to send HID button: %s", esp_err_to_name(ret));
                    }
                }
            }
        }
    }
}

// USB HID task (must be called periodically)
void usb_hid_task_wrapper(void *pvParameters) {
    while (1) {
        usb_hid_task();
        vTaskDelay(pdMS_TO_TICKS(1)); // TinyUSB task should be called frequently
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Инициализация BMW Shifter Controller...");
    
    // Initialize shifter state
    bmw_shifter_init(&shifter_state);
    bmw_shifter_init(&prev_shifter_state);  // Initialize previous state
    shifter_state_initialized = false;  // Mark as not initialized until first update
    
    // Initialize USB HID
    ESP_LOGI(TAG, "Инициализация USB HID...");
    ESP_ERROR_CHECK(usb_hid_init());
    
    // Configure UART for serial communication
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    
    // Configure TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    ESP_LOGI(TAG, "Установка TWAI драйвера...");
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    
    ESP_LOGI(TAG, "TWAI драйвер запущен. TX GPIO: %d, RX GPIO: %d", g_config.tx_io, g_config.rx_io);
    
    // Initialize CAN message structures
    gear_display_msg.counter_and_flags = 0x00;
    gear_display_msg.gear_indication = GEAR_IND_P;
    current_gear_indication = GEAR_IND_P;  // Initialize gear indication
    
    // Create timers for periodic CAN messages
    timer_gear_display = xTimerCreate("GearDisplay", 
                                      pdMS_TO_TICKS(TIMING_GEAR_DISPLAY_MS),
                                      pdTRUE, NULL, timer_gear_display_callback);
    timer_backlight = xTimerCreate("Backlight",
                                   pdMS_TO_TICKS(TIMING_BACKLIGHT_MS),
                                   pdTRUE, NULL, timer_backlight_callback);
    timer_heartbeat = xTimerCreate("Heartbeat",
                                   pdMS_TO_TICKS(TIMING_HEARTBEAT_MS),
                                   pdTRUE, NULL, timer_heartbeat_callback);
    
    xTimerStart(timer_gear_display, 0);
    xTimerStart(timer_backlight, 0);
    xTimerStart(timer_heartbeat, 0);
    
    // Create tasks
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 5, NULL);
    xTaskCreate(serial_rx_task, "serial_rx", 2048, NULL, 5, NULL);
    xTaskCreate(usb_hid_task_wrapper, "usb_hid", 4096, NULL, 5, NULL);
    xTaskCreate(hid_update_task, "hid_update", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Система инициализирована. Ожидание сообщений от шифтера...");
    ESP_LOGI(TAG, "USB HID устройство готово. Подключите второй USB порт к компьютеру.");
    
    // Main loop - just keep alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Check shifter connection status
        static bool was_connected = false;
        uint32_t now = xTaskGetTickCount();
        if (shifter_connected && (now - last_heartbeat_time > pdMS_TO_TICKS(2000))) {
            ESP_LOGW(TAG, "Шифтер не отвечает более 2 секунд");
            shifter_connected = false;
        }
        
        // If connection lost, release all HID buttons
        if (was_connected && !shifter_connected && usb_hid_is_ready()) {
            ESP_LOGI(TAG, "HID: Releasing all buttons due to connection loss");
            // Release all gear buttons
            usb_hid_send_button(HID_BUTTON_P, HID_ACTION_RELEASE);
            usb_hid_send_button(HID_BUTTON_R, HID_ACTION_RELEASE);
            usb_hid_send_button(HID_BUTTON_N, HID_ACTION_RELEASE);
            usb_hid_send_button(HID_BUTTON_D, HID_ACTION_RELEASE);
            usb_hid_send_button(HID_BUTTON_M, HID_ACTION_RELEASE);
            usb_hid_send_button(HID_BUTTON_PLUS, HID_ACTION_RELEASE);
            usb_hid_send_button(HID_BUTTON_MINUS, HID_ACTION_RELEASE);
            // Reset state initialization flag and gear indication
            shifter_state_initialized = false;
            current_gear_indication = 0;  // Reset to trigger update on reconnect
            // Reset button press state
            button_is_pressed = false;
            button_press_time = 0;
            button_should_hold = false;
        }
        was_connected = shifter_connected;
    }
}
