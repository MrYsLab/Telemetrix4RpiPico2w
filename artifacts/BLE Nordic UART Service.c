/*
 * BLE Nordic UART Service (NUS) - GATT Builder Version
 * For Raspberry Pi Pico 2W
 * 
 * IMPORTANT: Tools->IP/Bluetooth Stack = "IPv4 + Bluetooth"
 */

extern "C" {
  #include "btstack.h"
  #include "ble/gatt-service/battery_service_server.h"
}

#include <Arduino.h>

// Nordic UART Service UUIDs
static const uint8_t nus_service_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

static const uint8_t nus_rx_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

static const uint8_t nus_tx_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

// Connection state
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static bool notifications_enabled = false;
static uint16_t tx_char_value_handle = 0;
static uint16_t tx_char_cccd_handle = 0;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Advertising data - includes name for better scanner compatibility
static const uint8_t adv_data[] = {
    0x02, 0x01, 0x06,  // Flags
    0x07, 0x09, 'p', 'i', 'c', 'o', '2', 'w',  // Complete Local Name
    0x11, 0x07,        // 128-bit Service UUID
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
};

static const uint8_t scan_resp_data[] = {
    0x07, 0x09, 'p', 'i', 'c', 'o', '2', 'w'
};

// Simple ATT database - minimal version for testing
static const uint8_t profile_data[] = 
{
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18, 
    
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a, 
    
    // 0x0003 VALUE CHARACTERISTIC-GAP_DEVICE_NAME - READ -'Pico2W'
    0x0e, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x50, 0x69, 0x63, 0x6f, 0x32, 0x57, 

    // Nordic UART Service
    // 0x0004 PRIMARY_SERVICE-6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    0x18, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E, 

    // RX Characteristic (Write Without Response & Write)
    // 0x0005 CHARACTERISTIC-6E400002-B5A3-F393-E0A9-E50E24DCCA9E - WRITE_WITHOUT_RESPONSE | WRITE
    0x1b, 0x00, 0x02, 0x00, 0x05, 0x00, 0x03, 0x28, 0x0c, 0x06, 0x00, 
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E, 
    
    // 0x0006 VALUE CHARACTERISTIC-6E400002 - WRITE_WITHOUT_RESPONSE | WRITE | DYNAMIC
    0x16, 0x00, 0x0c, 0x03, 0x06, 0x00, 
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E, 

    // TX Characteristic (Notify)
    // 0x0007 CHARACTERISTIC-6E400003-B5A3-F393-E0A9-E50E24DCCA9E - NOTIFY
    0x1b, 0x00, 0x02, 0x00, 0x07, 0x00, 0x03, 0x28, 0x10, 0x08, 0x00, 
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E, 
    
    // 0x0008 VALUE CHARACTERISTIC-6E400003 - NOTIFY | DYNAMIC
    0x16, 0x00, 0x10, 0x03, 0x08, 0x00, 
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E, 
    
    // 0x0009 CLIENT_CHARACTERISTIC_CONFIGURATION
    0x0a, 0x00, 0x0e, 0x01, 0x09, 0x00, 0x02, 0x29, 0x00, 0x00, 
};

// ATT Write Callback
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, 
                              uint16_t transaction_mode, uint16_t offset, 
                              uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    
    Serial.print("Write to handle 0x");
    Serial.print(att_handle, HEX);
    Serial.print(": ");
    
    if (att_handle == 0x0006) {  // RX characteristic
        Serial.print("RX Data: ");
        for (int i = 0; i < buffer_size; i++) {
            Serial.print((char)buffer[i]);
        }
        Serial.println();

        // Echo back if notifications enabled
        if (connection_handle != HCI_CON_HANDLE_INVALID && notifications_enabled) {
            att_server_notify(connection_handle, 0x0008, buffer, buffer_size);
            Serial.println("Echoed back to phone");
        }
    }
    else if (att_handle == 0x0009) {  // CCCD
        notifications_enabled = (buffer_size >= 2 && (buffer[0] & 0x01));
        Serial.println(notifications_enabled ? "NOTIFICATIONS ENABLED!" : "Notifications disabled");
    }
    else {
        Serial.println("Unknown handle");
    }
    
    return 0;
}

// ATT Read Callback
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, 
                                  uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);
    UNUSED(offset);
    UNUSED(buffer);
    UNUSED(buffer_size);
    
    Serial.print("Read from handle 0x");
    Serial.println(att_handle, HEX);
    
    return 0;
}

// Packet handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                Serial.println("\u2713 BTstack ready - starting advertising");
                gap_advertisements_enable(1);
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = HCI_CON_HANDLE_INVALID;
            notifications_enabled = false;
            Serial.println("\n========== DISCONNECTED ==========");
            gap_advertisements_enable(1);
            break;

        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    Serial.println("\n========== CONNECTED ==========");
                    Serial.println("Device connected!");
                    Serial.println("In nRF Connect:");
                    Serial.println("1. You should see services now");
                    Serial.println("2. Find Nordic UART Service");
                    Serial.println("3. Enable notify on TX (handle 0x0008)");
                    Serial.println("4. Write to RX (handle 0x0006)");
                    Serial.println("===============================\n");
                    break;
            }
            break;
    }
}

void setup() {
    Serial.begin(115200);
    
    for (int i = 0; i < 30 && !Serial; i++) {
        delay(100);
    }

    Serial.println("\n\n=============================================");
    Serial.println("Nordic UART Service - Debug Version");
    Serial.println("Raspberry Pi Pico 2W");
    Serial.println("=============================================\n");

    // Initialize L2CAP
    l2cap_init();
    
    // Initialize LE Security Manager (required for some operations)
    sm_init();

    // Initialize ATT server
    Serial.println("Initializing ATT server...");
    att_server_init(profile_data, att_read_callback, att_write_callback);
    Serial.println("\u2713 ATT server initialized");

    // Register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(packet_handler);

    // Setup advertising
    bd_addr_t null_addr = {0, 0, 0, 0, 0, 0};
    
    gap_advertisements_set_params(
        0x0030, 0x0030, 0x00, (bd_addr_type_t)0, 
        null_addr, 0x07, 0x00
    );
    
    gap_advertisements_set_data(sizeof(adv_data), (uint8_t*)adv_data);
    gap_scan_response_set_data(sizeof(scan_resp_data), (uint8_t*)scan_resp_data);

    Serial.println("Powering on Bluetooth...");
    hci_power_control(HCI_POWER_ON);

    Serial.println("\n\u2713 Setup complete");
    Serial.println("\u2713 Advertising as 'pico2w'");
    Serial.println("\u2713 Connect with nRF Connect\n");
    Serial.println("Handle Map:");
    Serial.println("  0x0006 = RX (write here)");
    Serial.println("  0x0008 = TX (read/notify here)");
    Serial.println("  0x0009 = CCCD (enable notifications)\n");
}

void loop() {
    static unsigned long lastPrint = 0;
    
    // Status update every 10 seconds
    if (millis() - lastPrint > 10000) {
        if (con_handle == HCI_CON_HANDLE_INVALID) {
            Serial.println("[Advertising...]");
        } else {
            Serial.print("[Connected - Notifications: ");
            Serial.print(notifications_enabled ? "ON" : "OFF");
            Serial.println("]");
        }
        lastPrint = millis();
    }
    
    // Send serial input to phone
    if (Serial.available() > 0 && con_handle != HCI_CON_HANDLE_INVALID && notifications_enabled) {
        uint8_t data[128];
        int len = 0;

        while (Serial.available() > 0 && len < 128) {
            data[len++] = Serial.read();
            delay(1);
        }

        if (len > 0) {
            att_server_notify(con_handle, 0x0008, data, len);
            Serial.print("Sent to phone: ");
            Serial.write(data, len);
            Serial.println();
        }
    }

    delay(10);
}
