/*
 * BLE_Transport.cpp
 * BLE Nordic UART Service implementation using BTstack
 */

#include "BLE_Transport.h"

extern "C" {
    #include "btstack.h"
    #include "pico/cyw43_arch.h"
    #include "ble/gatt-service/nordic_spp_service_server.h"
}

// Global instance
BLE_Transport bleTransport;

// BTstack callback functions
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static void nordic_can_send_now(void * context);

// Nordic UART Service UUIDs (128-bit, reversed for BTstack)
const uint8_t nordic_spp_service_uuid[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
const uint8_t nordic_spp_service_tx_uuid[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};
const uint8_t nordic_spp_service_rx_uuid[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};

// ATT database definition
// This creates the Nordic UART Service structure
static const uint8_t att_db_storage[] = {
    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18,
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME-READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a,
    // 0x0003 VALUE-GAP_DEVICE_NAME-READ-'Tmx4Pico2W'
    0x16, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x54, 0x6d, 0x78, 0x34, 0x50, 0x69, 0x63, 0x6f, 0x32, 0x57,

    // 0x0004 PRIMARY_SERVICE-GATT_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 0x01, 0x18,

    // 0x0005 PRIMARY_SERVICE-6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    0x18, 0x00, 0x02, 0x00, 0x05, 0x00, 0x00, 0x28, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,

    // 0x0006 CHARACTERISTIC-6E400003-B5A3-F393-E0A9-E50E24DCCA9E-NOTIFY
    0x1b, 0x00, 0x02, 0x00, 0x06, 0x00, 0x03, 0x28, 0x10, 0x07, 0x00, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
    // 0x0007 VALUE-6E400003-B5A3-F393-E0A9-E50E24DCCA9E-NOTIFY-''
    0x16, 0x00, 0x10, 0x03, 0x07, 0x00, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
    // 0x0008 CLIENT_CHARACTERISTIC_CONFIGURATION
    0x0a, 0x00, 0x0a, 0x01, 0x08, 0x00, 0x02, 0x29, 0x00, 0x00,

    // 0x0009 CHARACTERISTIC-6E400002-B5A3-F393-E0A9-E50E24DCCA9E-WRITE_WITHOUT_RESPONSE | WRITE
    0x1b, 0x00, 0x02, 0x00, 0x09, 0x00, 0x03, 0x28, 0x0c, 0x0a, 0x00, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,
    // 0x000a VALUE-6E400002-B5A3-F393-E0A9-E50E24DCCA9E-WRITE_WITHOUT_RESPONSE | WRITE-''
    0x16, 0x00, 0x0c, 0x03, 0x0a, 0x00, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,

    // END
    0x00, 0x00,
};

// ATT handles
#define ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE 0x0007
#define ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_CLIENT_CONFIGURATION_HANDLE 0x0008
#define ATT_CHARACTERISTIC_6E400002_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE 0x000a

BLE_Transport::BLE_Transport()
    : rxHead(0), rxTail(0), connected(false), connection_handle(0),
      tx_notifications_enabled(false), att_tx_value_handle(ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE),
      att_rx_value_handle(ATT_CHARACTERISTIC_6E400002_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE),
      txLength(0), txPending(false) {
}

void BLE_Transport::begin(const char* deviceName) {
    // Clear buffers
    rxHead = 0;
    rxTail = 0;
    txLength = 0;
    txPending = false;

    // Initialize CYW43 architecture
    if (cyw43_arch_init()) {
        Serial.println("Failed to initialize CYW43");
        return;
    }

    // Initialize BTstack
    l2cap_init();
    sm_init();

    // Setup ATT server with our database
    att_server_init((uint8_t*)att_db_storage, att_read_callback, att_write_callback);

    // Setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);

    // Set local name
    gap_set_local_name(deviceName);
    gap_discoverable_control(1);

    // Setup advertising data with Nordic UART Service UUID
    uint8_t adv_data[] = {
        // Flags general discoverable, BR/EDR not supported
        0x02, 0x01, 0x06,
        // Name
        0x0B, 0x09, 'T', 'm', 'x', '4', 'P', 'i', 'c', 'o', '2', 'W',
        // 128-bit Service UUID
        0x11, 0x07,
        0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
        0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
    };
    gap_advertisements_set_data(sizeof(adv_data), adv_data);

    // Register for HCI events
    hci_event_callback_registration_t hci_callback;
    hci_callback.callback = &packet_handler;
    hci_add_event_handler(&hci_callback);

    // Register for ATT server events
    att_server_register_packet_handler(&packet_handler);

    // Turn on Bluetooth!
    hci_power_control(HCI_POWER_ON);

    Serial.println("BLE initialized");
}

void BLE_Transport::update() {
    // BTstack runs in the background via interrupts
    // This is here for compatibility
}

int BLE_Transport::available() {
    if (rxHead >= rxTail) {
        return rxHead - rxTail;
    } else {
        return BUFFER_SIZE - rxTail + rxHead;
    }
}

int BLE_Transport::read() {
    if (rxHead == rxTail) {
        return -1;
    }
    uint8_t data = rxBuffer[rxTail];
    rxTail = (rxTail + 1) % BUFFER_SIZE;
    return data;
}

int BLE_Transport::peek() {
    if (rxHead == rxTail) {
        return -1;
    }
    return rxBuffer[rxTail];
}

size_t BLE_Transport::write(uint8_t byte) {
    return write(&byte, 1);
}

size_t BLE_Transport::write(const uint8_t* buffer, size_t size) {
    if (!connected || !tx_notifications_enabled) {
        return 0;
    }

    size_t written = 0;
    const size_t max_mtu = 20; // Conservative MTU size

    while (written < size) {
        size_t chunk = min((size_t)max_mtu, size - written);

        // Wait for previous transmission to complete
        while (txPending) {
            delay(1);
        }

        // Copy data to TX buffer
        memcpy(txBuffer, buffer + written, chunk);
        txLength = chunk;
        txPending = true;

        // Request permission to send
        att_server_request_can_send_now_event(connection_handle);

        // Wait for transmission
        uint32_t start = millis();
        while (txPending && (millis() - start < 100)) {
            delay(1);
        }

        if (txPending) {
            // Timeout
            txPending = false;
            return written;
        }

        written += chunk;
    }

    return written;
}

size_t BLE_Transport::print(const char* str) {
    return write((const uint8_t*)str, strlen(str));
}

size_t BLE_Transport::println(const char* str) {
    size_t n = print(str);
    n += write('\r');
    n += write('\n');
    return n;
}

void BLE_Transport::handleRxData(const uint8_t* data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        uint16_t nextHead = (rxHead + 1) % BUFFER_SIZE;
        if (nextHead != rxTail) {
            rxBuffer[rxHead] = data[i];
            rxHead = nextHead;
        }
    }
}

void BLE_Transport::handleConnection(uint16_t conn_handle) {
    connected = true;
    connection_handle = conn_handle;
    Serial.println("BLE connected");
}

void BLE_Transport::handleDisconnection() {
    connected = false;
    connection_handle = 0;
    tx_notifications_enabled = false;
    txPending = false;
    rxHead = 0;
    rxTail = 0;
    Serial.println("BLE disconnected");
}

void BLE_Transport::handleTxNotificationsEnabled(bool enabled) {
    tx_notifications_enabled = enabled;
}

void BLE_Transport::handleCanSendNow() {
    if (txPending && txLength > 0) {
        att_server_notify(connection_handle, att_tx_value_handle, txBuffer, txLength);
        txLength = 0;
        txPending = false;
    }
}

// C callback functions
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);
    UNUSED(offset);
    UNUSED(buffer);
    UNUSED(buffer_size);

    // No readable characteristics in Nordic UART Service
    return 0;
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);

    if (att_handle == ATT_CHARACTERISTIC_6E400002_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE) {
        // RX characteristic write - data received from client
        bleTransport.handleRxData(buffer, buffer_size);
        return 0;
    }

    if (att_handle == ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_CLIENT_CONFIGURATION_HANDLE) {
        // Client Characteristic Configuration Descriptor
        bool enabled = (buffer[0] == 0x01);
        bleTransport.handleTxNotificationsEnabled(enabled);
        return 0;
    }

    return 0;
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            bleTransport.handleDisconnection();
            break;

        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE: {
                    uint16_t conn_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    bleTransport.handleConnection(conn_handle);
                    break;
                }
            }
            break;

        case ATT_EVENT_CAN_SEND_NOW:
            bleTransport.handleCanSendNow();
            break;
    }
}