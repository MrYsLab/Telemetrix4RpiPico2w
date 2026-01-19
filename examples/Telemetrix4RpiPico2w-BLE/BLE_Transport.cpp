/*
 * BLE_Transport.cpp - BLE Nordic UART Service transport implementation
 * Based on working BLE NUS implementation
 */

#include "BLE_Transport.h"

// Nordic UART Service UUIDs (little-endian format)
static const uint8_t nus_service_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// ATT Database for Nordic UART Service
static const uint8_t profile_data[] =
{
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18,

    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a,

    // 0x0003 VALUE CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x12, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a,
    'T', 'e', 'l', 'e', 'm', 'e', 't', 'r', 'i', 'x',

    // 0x0004 PRIMARY_SERVICE - Nordic UART Service
    0x18, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,

    // 0x0005 CHARACTERISTIC - RX (Write Without Response & Write)
    0x1b, 0x00, 0x02, 0x00, 0x05, 0x00, 0x03, 0x28, 0x0c, 0x06, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,

    // 0x0006 VALUE - RX Characteristic (WRITE_WITHOUT_RESPONSE | WRITE | DYNAMIC)
    0x16, 0x00, 0x0c, 0x03, 0x06, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,

    // 0x0007 CHARACTERISTIC - TX (Notify)
    0x1b, 0x00, 0x02, 0x00, 0x07, 0x00, 0x03, 0x28, 0x10, 0x08, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,

    // 0x0008 VALUE - TX Characteristic (NOTIFY | DYNAMIC)
    0x16, 0x00, 0x10, 0x03, 0x08, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,

    // 0x0009 CLIENT_CHARACTERISTIC_CONFIGURATION
    0x0a, 0x00, 0x0e, 0x01, 0x09, 0x00, 0x02, 0x29, 0x00, 0x00,
};

// Static member initialization
uint8_t BLE_Transport::rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t BLE_Transport::rx_head = 0;
volatile uint16_t BLE_Transport::rx_tail = 0;
hci_con_handle_t BLE_Transport::con_handle = HCI_CON_HANDLE_INVALID;
bool BLE_Transport::notifications_enabled = false;
btstack_packet_callback_registration_t BLE_Transport::hci_event_callback_registration;

BLE_Transport::BLE_Transport() {
    rx_head = 0;
    rx_tail = 0;
    con_handle = HCI_CON_HANDLE_INVALID;
    notifications_enabled = false;
}

void BLE_Transport::begin(const char* deviceName) {
    // Initialize L2CAP
    l2cap_init();

    // Initialize LE Security Manager
    sm_init();

    // Initialize ATT server with our profile
    att_server_init(profile_data, att_read_callback, att_write_callback);

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

    // Advertising data with device name
    uint8_t name_len = strlen(deviceName);
    if (name_len > 20) name_len = 20; // Limit name length

    uint8_t adv_data[31];
    uint8_t pos = 0;

    // Flags
    adv_data[pos++] = 0x02;
    adv_data[pos++] = 0x01;
    adv_data[pos++] = 0x06;

    // Complete Local Name
    adv_data[pos++] = name_len + 1;
    adv_data[pos++] = 0x09;
    memcpy(&adv_data[pos], deviceName, name_len);
    pos += name_len;

    // 128-bit Service UUID
    adv_data[pos++] = 0x11;
    adv_data[pos++] = 0x07;
    memcpy(&adv_data[pos], nus_service_uuid, 16);
    pos += 16;

    gap_advertisements_set_data(pos, adv_data);

    // Scan response with device name
    uint8_t scan_resp[31];
    scan_resp[0] = name_len + 1;
    scan_resp[1] = 0x09;
    memcpy(&scan_resp[2], deviceName, name_len);
    gap_scan_response_set_data(name_len + 2, scan_resp);

    // Power on Bluetooth
    hci_power_control(HCI_POWER_ON);
}

void BLE_Transport::end() {
    gap_advertisements_enable(0);
    hci_power_control(HCI_POWER_OFF);
}

int BLE_Transport::available() {
    return (RX_BUFFER_SIZE + rx_head - rx_tail) % RX_BUFFER_SIZE;
}

int BLE_Transport::read() {
    if (rx_head == rx_tail) {
        return -1;
    }

    uint8_t c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return c;
}

int BLE_Transport::peek() {
    if (rx_head == rx_tail) {
        return -1;
    }
    return rx_buffer[rx_tail];
}

size_t BLE_Transport::write(uint8_t byte) {
    return write(&byte, 1);
}

size_t BLE_Transport::write(const uint8_t *buffer, size_t size) {
    if (con_handle == HCI_CON_HANDLE_INVALID || !notifications_enabled) {
        return 0;
    }

    // Split data into chunks if necessary (BLE MTU limits)
    size_t sent = 0;
    const size_t max_chunk = 20; // Safe size for BLE notifications

    while (sent < size) {
        size_t chunk_size = size - sent;
        if (chunk_size > max_chunk) {
            chunk_size = max_chunk;
        }

        // Send via TX characteristic (handle 0x0008)
        att_server_notify(con_handle, 0x0008, (uint8_t*)(buffer + sent), chunk_size);
        sent += chunk_size;

        // Small delay between chunks to avoid overwhelming BLE stack
        if (sent < size) {
            delay(5);
        }
    }

    return sent;
}

void BLE_Transport::flush() {
    // No buffering, so nothing to flush
}

// ATT Write Callback - handles incoming data from BLE client
int BLE_Transport::att_write_callback(hci_con_handle_t connection_handle,
                                      uint16_t att_handle,
                                      uint16_t transaction_mode,
                                      uint16_t offset,
                                      uint8_t *buffer,
                                      uint16_t buffer_size) {
    UNUSED(connection_handle);
    UNUSED(transaction_mode);
    UNUSED(offset);

    if (att_handle == 0x0006) {  // RX characteristic - data from client
        // Add data to circular buffer
        for (uint16_t i = 0; i < buffer_size; i++) {
            uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
            if (next_head != rx_tail) {  // Buffer not full
                rx_buffer[rx_head] = buffer[i];
                rx_head = next_head;
            } else {
                // Buffer overflow - data lost
                break;
            }
        }
    }
    else if (att_handle == 0x0009) {  // CCCD - notification enable/disable
        notifications_enabled = (buffer_size >= 2 && (buffer[0] & 0x01));
    }

    return 0;
}

// ATT Read Callback
uint16_t BLE_Transport::att_read_callback(hci_con_handle_t connection_handle,
                                         uint16_t att_handle,
                                         uint16_t offset,
                                         uint8_t *buffer,
                                         uint16_t buffer_size) {
    UNUSED(connection_handle);
    UNUSED(att_handle);
    UNUSED(offset);
    UNUSED(buffer);
    UNUSED(buffer_size);
    return 0;
}

// Packet Handler - handles BLE connection events
void BLE_Transport::packet_handler(uint8_t packet_type, uint16_t channel,
                                   uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                // Start advertising when Bluetooth is ready
                gap_advertisements_enable(1);
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = HCI_CON_HANDLE_INVALID;
            notifications_enabled = false;
            // Restart advertising after disconnect
            gap_advertisements_enable(1);
            break;

        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    break;
            }
            break;
    }
}