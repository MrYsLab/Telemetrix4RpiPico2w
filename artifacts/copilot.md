

# Project: Telemtrix4RpiPico2w-BLE

## Description

This project implements a Telemetrix server within an Arduino .ino sketch, called 
**Telemetrix4RpiPico2w-BLE.ino**. 
```aiignore
/*
 * BLE Nordic UART Service for Telemetrix4RpiPico2w
 * Based on working example - User can set device name
 *
 * IMPORTANT: Tools->IP/Bluetooth Stack = "IPv4 + Bluetooth"
 */

// ============================================================
// USER CONFIGURABLE - Change this to set your device name
// ============================================================
#define BLE_DEVICE_NAME "Tmx4Pico"
// ============================================================

extern "C" {
  #include "btstack.h"
}

#include <Arduino.h>

// Nordic UART Service UUIDs (little-endian)
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const uint8_t nus_service_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6e
};

// RX Characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write from client)
static const uint8_t nus_rx_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6e
};

// TX Characteristic: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Notify to client)
static const uint8_t nus_tx_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6e
};

// Connection state
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static bool notifications_enabled = false;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Advertising data - dynamically built in setup()
static uint8_t adv_data[128];
static uint8_t adv_data_len = 0;

static uint8_t scan_resp_data[128];
static uint8_t scan_resp_data_len = 0;

// ATT Database - dynamically updated with device name
static uint8_t profile_data[] = {
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18,

    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a,

    // 0x0003 VALUE-GAP_DEVICE_NAME
    // NOTE: Size will be updated in setup() based on actual device name length
    0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a,
    // Device name will be inserted here (up to 20 bytes reserved)
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

    // 0x0004 PRIMARY_SERVICE-GATT_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 0x01, 0x18,

    // 0x0005 PRIMARY_SERVICE-Nordic UART Service
    0x18, 0x00, 0x02, 0x00, 0x05, 0x00, 0x00, 0x28,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,

    // 0x0006 CHARACTERISTIC-NUS RX - WRITE_WITHOUT_RESPONSE | WRITE
    0x1b, 0x00, 0x02, 0x00, 0x06, 0x00, 0x03, 0x28, 0x0c, 0x07, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,

    // 0x0007 VALUE-NUS RX - WRITE_WITHOUT_RESPONSE | WRITE | DYNAMIC
    0x16, 0x00, 0x0c, 0x03, 0x07, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,

    // 0x0008 CHARACTERISTIC-NUS TX - NOTIFY
    0x1b, 0x00, 0x02, 0x00, 0x08, 0x00, 0x03, 0x28, 0x10, 0x09, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,

    // 0x0009 VALUE-NUS TX - NOTIFY | DYNAMIC
    0x16, 0x00, 0x10, 0x03, 0x09, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,

    // 0x000a CLIENT_CHARACTERISTIC_CONFIGURATION
    0x0a, 0x00, 0x0e, 0x01, 0x0a, 0x00, 0x02, 0x29, 0x00, 0x00,

    // END
    0x00, 0x00,
};

// ATT Write Callback
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle,
                              uint16_t transaction_mode, uint16_t offset,
                              uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);

    Serial.print("ATT Write, handle 0x");
    Serial.print(att_handle, HEX);
    Serial.print(", size ");
    Serial.print(buffer_size);
    Serial.print(": ");

    if (att_handle == 0x0007) {  // RX characteristic
        Serial.print("RX Data: ");
        for (int i = 0; i < buffer_size; i++) {
            Serial.print((char)buffer[i]);
        }
        Serial.println();

        // Echo back if notifications enabled
        if (connection_handle != HCI_CON_HANDLE_INVALID && notifications_enabled) {
            att_server_notify(connection_handle, 0x0009, buffer, buffer_size);
            Serial.println("  -> Echoed back via TX characteristic");
        }
    }
    else if (att_handle == 0x000a) {  // CCCD for TX characteristic
        notifications_enabled = (buffer_size >= 2 && (buffer[0] & 0x01));
        Serial.println(notifications_enabled ? "TX Notifications ENABLED" : "TX Notifications DISABLED");
    }
    else {
        Serial.println("(unknown handle)");
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

    Serial.print("ATT Read, handle 0x");
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
                bd_addr_t local_addr;
                gap_local_bd_addr(local_addr);
                Serial.println("\n*** BTstack WORKING ***");
                Serial.print("Local BT Address: ");
                Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                    local_addr[0], local_addr[1], local_addr[2],
                    local_addr[3], local_addr[4], local_addr[5]);
                Serial.println("*** Starting Advertising ***");
                gap_advertisements_enable(1);
                Serial.println("*** ADVERTISING NOW! ***\n");
                Serial.print("Look for '");
                Serial.print(BLE_DEVICE_NAME);
                Serial.println("' in nRF Connect or BLE scanner");
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = HCI_CON_HANDLE_INVALID;
            notifications_enabled = false;
            Serial.println("\n=== DISCONNECTED ===");
            Serial.println("Restarting advertising...\n");
            gap_advertisements_enable(1);
            break;

        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    Serial.println("\n=== CONNECTED ===");
                    Serial.print("Connection handle: 0x");
                    Serial.println(con_handle, HEX);
                    Serial.println("\nNordic UART Service ready:");
                    Serial.println("  RX (write):  handle 0x0007");
                    Serial.println("  TX (notify): handle 0x0009");
                    Serial.println("  CCCD:        handle 0x000a");
                    Serial.println("\nEnable notifications on TX, then write to RX!\n");
                    break;
            }
            break;
    }
}

void setup() {
    Serial.begin(115200);

    // Wait for serial connection (up to 3 seconds)
    for (int i = 0; i < 30 && !Serial; i++) {
        delay(100);
    }

    Serial.println("\n\n=============================================");
    Serial.println("  Telemetrix4RpiPico2w - BLE Transport");
    Serial.println("  Nordic UART Service (NUS)");
    Serial.println("=============================================\n");

    // Update ATT database with device name
    // The device name field starts at offset 23 in profile_data
    // First, update the length byte at offset 21 (total length of this ATT entry)
    uint8_t name_len = strlen(BLE_DEVICE_NAME);
    profile_data[21] = 8 + name_len;  // 8 bytes header + name length

    // Copy device name into profile_data at offset 23
    memcpy(&profile_data[23], BLE_DEVICE_NAME, name_len);

    Serial.print("Device Name: ");
    Serial.println(BLE_DEVICE_NAME);
    Serial.print("Name Length: ");
    Serial.println(name_len);

    // Build advertising data - Service UUID only (more reliable)
    uint8_t pos = 0;

    // Flags
    adv_data[pos++] = 0x02;
    adv_data[pos++] = 0x01;
    adv_data[pos++] = 0x06;

    // 128-bit Service UUID (CRITICAL - must be in advertising data)
    adv_data[pos++] = 0x11;  // Length: 17 bytes
    adv_data[pos++] = 0x07;  // Type: Complete 128-bit UUID
    memcpy(&adv_data[pos], nus_service_uuid, 16);
    pos += 16;

    adv_data_len = pos;

    // Build scan response data - Device name goes here
    pos = 0;
    scan_resp_data[pos++] = 1 + name_len;
    scan_resp_data[pos++] = 0x09;  // Type: Complete Local Name
    memcpy(&scan_resp_data[pos], BLE_DEVICE_NAME, name_len);
    pos += name_len;
    scan_resp_data_len = pos;

    // Initialize L2CAP
    l2cap_init();
    Serial.println("L2CAP initialized");

    // Initialize LE Security Manager
    sm_init();
    Serial.println("Security Manager initialized");

    // Initialize ATT server with our profile
    att_server_init(profile_data, att_read_callback, att_write_callback);
    Serial.println("ATT server initialized with NUS profile");

    // Register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Also register packet handler with ATT server
    att_server_register_packet_handler(packet_handler);
    Serial.println("Event handlers registered");

    // Setup advertising parameters
    bd_addr_t null_addr = {0, 0, 0, 0, 0, 0};

    gap_advertisements_set_params(
        0x0030,              // min interval (30ms)
        0x0030,              // max interval (30ms)
        0x00,                // ADV_IND - connectable undirected
        (bd_addr_type_t)0,   // own address type
        null_addr,           // direct address (not used)
        0x07,                // all channels
        0x00                 // filter policy
    );

    // Set advertising data
    gap_advertisements_set_data(adv_data_len, adv_data);
    gap_scan_response_set_data(scan_resp_data_len, scan_resp_data);
    Serial.println("Advertising data configured");

    // Print advertising data for debugging
    Serial.println("\nAdvertising data (HEX):");
    for(int i = 0; i < adv_data_len; i++) {
        Serial.printf("%02X ", adv_data[i]);
        if ((i + 1) % 16 == 0) Serial.println();
    }
    Serial.println("\n");

    // Power on Bluetooth
    Serial.println("Powering on Bluetooth...");
    hci_power_control(HCI_POWER_ON);

    Serial.println("\nSetup complete - waiting for BTstack to initialize...");
}

void loop() {
    static unsigned long lastStatusPrint = 0;

    // Print status every 10 seconds
    if (millis() - lastStatusPrint > 10000) {
        if (con_handle == HCI_CON_HANDLE_INVALID) {
            Serial.println("[Status: Advertising - waiting for connection...]");
        } else {
            Serial.print("[Status: Connected - Notifications ");
            Serial.print(notifications_enabled ? "ENABLED" : "DISABLED");
            Serial.println("]");
        }
        lastStatusPrint = millis();
    }

    // Send serial input to connected device via BLE
    if (Serial.available() > 0 && con_handle != HCI_CON_HANDLE_INVALID && notifications_enabled) {
        uint8_t data[128];
        int len = 0;

        // Read available serial data
        while (Serial.available() > 0 && len < 128) {
            data[len++] = Serial.read();
            delay(1);  // Small delay to allow more data to arrive
        }

        if (len > 0) {
            // Send notification to connected device
            att_server_notify(con_handle, 0x0009, data, len);
            Serial.print("Sent to BLE device: ");
            Serial.write(data, len);
            Serial.println();
        }
    }

    delay(10);
}
```

It
is compiled with the Arduino IDE and runs on a Raspberry Pi Pico 2W. It uses the 
arduino-pico library and arduino-pico's version of BTStack.
This server uses the Nordic BLE UART Service as its transport.

This server does not work. 

I have another arduino sketch, called BLE Nordic UART Service.ino that does seem to work.
```aiignore
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

```

To test I am using this Python file with the device ID set to match
the device ID in each of the sketches.
```
"""
BLE Nordic UART Service Diagnostic Script
This will show you exactly what's being discovered
"""
import asyncio
from bleak import BleakClient, BleakScanner

# Your device name
# DEVICE_NAME = "Tmx4Pico"
DEVICE_NAME = "Tmx4Pico"


# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Server TX, Client RX (notifications)
UART_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Server RX, Client TX (write)

async def main():
    print(f"Scanning for device '{DEVICE_NAME}'...")

    # Find the device
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

    if not device:
        print(f"ERROR: Device '{DEVICE_NAME}' not found!")
        return

    print(f"Found device: {device.name} ({device.address})")
    print(f"\nConnecting...")

    async with BleakClient(device, timeout=15.0) as client:
        print(f"Connected: {client.is_connected}")

        # Wait a moment for service discovery to complete
        await asyncio.sleep(1)

        # Services are auto-discovered on connection
        print("\n=== DISCOVERING SERVICES ===")
        # Just access .services to trigger discovery if needed

        # List all services
        print(f"\n=== ALL SERVICES ({len(client.services.services)} found) ===")
        for service in client.services:
            print(f"\nService: {service.uuid}")
            print(f"  Handle: {service.handle}")
            print(f"  Description: {service.description}")

            # List all characteristics for this service
            print(f"  Characteristics ({len(service.characteristics)} found):")
            for char in service.characteristics:
                print(f"    UUID: {char.uuid}")
                print(f"    Handle: {char.handle}")
                print(f"    Properties: {char.properties}")

                # List descriptors
                if char.descriptors:
                    print(f"    Descriptors:")
                    for desc in char.descriptors:
                        print(f"      UUID: {desc.uuid}, Handle: {desc.handle}")

        # Check specifically for Nordic UART Service
        print(f"\n=== CHECKING FOR NORDIC UART SERVICE ===")

        # Try to find the service
        nus_service = None
        for service in client.services:
            if service.uuid.lower() == NUS_SERVICE_UUID.lower():
                nus_service = service
                print(f"✓ Found Nordic UART Service!")
                break

        if not nus_service:
            print(f"✗ Nordic UART Service NOT FOUND")
            print(f"  Looking for: {NUS_SERVICE_UUID}")
            return

        # Check for TX characteristic (notify)
        print(f"\n=== CHECKING FOR TX CHARACTERISTIC (notify) ===")
        tx_char = None
        for char in nus_service.characteristics:
            print(f"  Checking: {char.uuid}")
            if char.uuid.lower() == UART_TX_UUID.lower():
                tx_char = char
                print(f"  ✓ Found TX characteristic!")
                print(f"    Properties: {char.properties}")
                break

        if not tx_char:
            print(f"  ✗ TX Characteristic NOT FOUND")
            print(f"    Looking for: {UART_TX_UUID}")

        # Check for RX characteristic (write)
        print(f"\n=== CHECKING FOR RX CHARACTERISTIC (write) ===")
        rx_char = None
        for char in nus_service.characteristics:
            print(f"  Checking: {char.uuid}")
            if char.uuid.lower() == UART_RX_UUID.lower():
                rx_char = char
                print(f"  ✓ Found RX characteristic!")
                print(f"    Properties: {char.properties}")
                break

        if not rx_char:
            print(f"  ✗ RX Characteristic NOT FOUND")
            print(f"    Looking for: {UART_RX_UUID}")

        # Try using get_characteristic method
        print(f"\n=== TESTING get_characteristic() METHOD ===")
        test_char = client.services.get_characteristic(UART_TX_UUID)
        print(f"get_characteristic('{UART_TX_UUID}'): {test_char}")

        test_char2 = client.services.get_characteristic(UART_RX_UUID)
        print(f"get_characteristic('{UART_RX_UUID}'): {test_char2}")

        # If we found the characteristics, try to use them
        if tx_char and rx_char:
            print(f"\n=== ATTEMPTING TO USE CHARACTERISTICS ===")

            def notification_handler(sender, data):
                print(f"Received: {data}")

            try:
                print("Enabling notifications on TX characteristic...")
                await client.start_notify(tx_char, notification_handler)
                print("✓ Notifications enabled!")

                print("Writing test data to RX characteristic...")
                test_data = b"Hello from Python!"
                await client.write_gatt_char(rx_char, test_data)
                print("✓ Data written!")

                # Wait for response
                await asyncio.sleep(2)

                print("Disabling notifications...")
                await client.stop_notify(tx_char)
                print("✓ Test complete!")

            except Exception as e:
                print(f"✗ Error during test: {e}")

        print(f"\n=== DIAGNOSTIC COMPLETE ===")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(e)

```

The output of this test for the working sketch is:
```aiignore
/home/afy/PycharmProjects/telemetrix-rpi-pico-w/.venv3/bin/python /home/afy/PycharmProjects/telemetrix-rpi-pico-2w/archive/claude_debug.py
Scanning for device 'pico2w'...
Found device: pico2w (2C:CF:67:DF:E8:EE)

Connecting...
Connected: True

=== DISCOVERING SERVICES ===

=== ALL SERVICES (2 found) ===

Service: 00001800-0000-1000-8000-00805f9b34fb
  Handle: 1
  Description: Generic Access Profile
  Characteristics (1 found):
    UUID: 00002a00-0000-1000-8000-00805f9b34fb
    Handle: 2
    Properties: ['read']

Service: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
  Handle: 4
  Description: Nordic UART Service
  Characteristics (2 found):
    UUID: 6e400003-b5a3-f393-e0a9-e50e24dcca9e
    Handle: 7
    Properties: ['notify']
    Descriptors:
      UUID: 00002902-0000-1000-8000-00805f9b34fb, Handle: 9
    UUID: 6e400002-b5a3-f393-e0a9-e50e24dcca9e
    Handle: 5
    Properties: ['write-without-response', 'write']

=== CHECKING FOR NORDIC UART SERVICE ===
✓ Found Nordic UART Service!

=== CHECKING FOR TX CHARACTERISTIC (notify) ===
  Checking: 6e400003-b5a3-f393-e0a9-e50e24dcca9e
  ✓ Found TX characteristic!
    Properties: ['notify']

=== CHECKING FOR RX CHARACTERISTIC (write) ===
  Checking: 6e400003-b5a3-f393-e0a9-e50e24dcca9e
  Checking: 6e400002-b5a3-f393-e0a9-e50e24dcca9e
  ✓ Found RX characteristic!
    Properties: ['write-without-response', 'write']

=== TESTING get_characteristic() METHOD ===
get_characteristic('6e400003-b5a3-f393-e0a9-e50e24dcca9e'): 6e400003-b5a3-f393-e0a9-e50e24dcca9e (Handle: 7): Nordic UART TX
get_characteristic('6e400002-b5a3-f393-e0a9-e50e24dcca9e'): 6e400002-b5a3-f393-e0a9-e50e24dcca9e (Handle: 5): Nordic UART RX

=== ATTEMPTING TO USE CHARACTERISTICS ===
Enabling notifications on TX characteristic...
✓ Notifications enabled!
Writing test data to RX characteristic...
✓ Data written!
Disabling notifications...
✓ Test complete!

=== DIAGNOSTIC COMPLETE ===


Process finished with exit code 0

```
The output in the Arduino serial monitor is:
```aiignore

========== CONNECTED ==========
Device connected!
In nRF Connect:
1. You should see services now
2. Find Nordic UART Service
3. Enable notify on TX (handle 0x0008)
4. Write to RX (handle 0x0006)
===============================


========== CONNECTED ==========
Device connected!
In nRF Connect:
1. You should see services now
2. Find Nordic UART Service
3. Enable notify on TX (handle 0x0008)
4. Write to RX (handle 0x0006)
===============================

[Connected - Notifications: OFF]
Write to handle 0x9: NOTIFICATIONS ENABLED!
Write to handle 0x6: RX Data: Hello from Python!
Echoed back to phone
Write to handle 0x9: Notifications disabled
Write to handle 0x0: Unknown handle

========== DISCONNECTED ==========

========== DISCONNECTED ==========
[Advertising...]

```

Please compare the two sketches, and repair Telemetrix4RpiPico2w-BLE so that
it correctly implements The Nordic UART service.



## Code Style
*   Code in C++ for the Arduino IDE.

## Dependencies
*   [BLE Nordic UART Service.c](https://github.com/MrYsLab/Telemetrix4RpiPico2w/blob/ble/artifacts/BLE%20Nordic%20UART%20Service.c)
*   [arduino-pico](https://github.com/earlephilhower/arduino-pico)

## Functional Requirements





