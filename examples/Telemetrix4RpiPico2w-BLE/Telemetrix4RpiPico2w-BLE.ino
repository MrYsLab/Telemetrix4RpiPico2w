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