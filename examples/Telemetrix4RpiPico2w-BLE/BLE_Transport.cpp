/*
 * BLE_Transport.cpp
 * Telemetrix4RpiPico2w - BLE Nordic UART Service Transport
 */

#include "BLE_Transport.h"

// Static instance pointer for callbacks
BLE_Transport* BLE_Transport::instance = nullptr;

// Forward declarations for GATT callbacks
static uint16_t att_read_callback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t offset, uint8_t* buffer, uint16_t bufferSize);
static int att_write_callback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t transactionMode, uint16_t offset, uint8_t* buffer, uint16_t bufferSize);

// ATT database - using raw bytes instead of macros
static uint8_t att_db_storage[] = {
    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18,
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME-READ
    0x00, 0x02, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a,
    // 0x0003 VALUE-GAP_DEVICE_NAME-READ-'Tmx4Pico2W'
    0x00, 0x03, 0x02, 0x00, 0x0b, 0x00, 0x00, 0x2a,
    'T', 'm', 'x', '4', 'P', 'i', 'c', 'o', '2', 'W', 0x00,

    // 0x0004 PRIMARY_SERVICE-NUS_SERVICE_UUID
    0x00, 0x04, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x6E, 0x40,

    // 0x0005 CHARACTERISTIC-NUS_RX-WRITE | WRITE_WITHOUT_RESPONSE
    0x00, 0x05, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x0C, 0x06, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x6E, 0x40,

    // 0x0006 VALUE-NUS_RX-WRITE | WRITE_WITHOUT_RESPONSE | DYNAMIC
    0x00, 0x06, 0x0a, 0x00, 0x00, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x6E, 0x40,

    // 0x0007 CHARACTERISTIC-NUS_TX-NOTIFY
    0x00, 0x07, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x10, 0x08, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x6E, 0x40,

    // 0x0008 VALUE-NUS_TX-NOTIFY | DYNAMIC
    0x00, 0x08, 0x0a, 0x00, 0x00, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x6E, 0x40,

    // 0x0009 CLIENT_CHARACTERISTIC_CONFIGURATION
    0x00, 0x09, 0x0a, 0x00, 0x02, 0x00, 0x02, 0x29, 0x00, 0x00,

    // END
    0x00, 0x00,
};

static uint8_t* att_db = att_db_storage;

BLE_Transport::BLE_Transport() {
    instance = this;
    conHandle = HCI_CON_HANDLE_INVALID;
    rxCharValueHandle = 0x06;
    txCharValueHandle = 0x08;
    txCharCCCDHandle = 0x09;
    connected = false;
    txNotifyEnabled = false;
    rxHead = 0;
    rxTail = 0;
    txHead = 0;
    txTail = 0;
    mtuSize = 23;
    strncpy(deviceName, DEFAULT_DEVICE_NAME, sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';
}

bool BLE_Transport::begin(const char* name) {
    if (name && name[0] != '\0') {
        setDeviceName(name);
    }

    // Initialize L2CAP
    l2cap_init();

    // Initialize ATT server with callbacks
    att_server_init(att_db, att_read_callback, att_write_callback);
    att_server_register_packet_handler(packetHandler);

    // Setup advertising data
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr = {0};

    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);

    // Build advertising data
    uint8_t adv_data[31];
    uint8_t adv_data_len = 0;

    // Flags
    adv_data[adv_data_len++] = 2;
    adv_data[adv_data_len++] = 0x01;
    adv_data[adv_data_len++] = 0x06;

    // Complete 128-bit Service UUIDs
    adv_data[adv_data_len++] = 17;
    adv_data[adv_data_len++] = 0x07;
    uint8_t nus_service_uuid[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                   0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x6E, 0x40};
    memcpy(&adv_data[adv_data_len], nus_service_uuid, 16);
    adv_data_len += 16;

    // Device name
    uint8_t name_len = strlen(deviceName);
    if (adv_data_len + 2 + name_len <= 31) {
        adv_data[adv_data_len++] = name_len + 1;
        adv_data[adv_data_len++] = 0x09;
        memcpy(&adv_data[adv_data_len], deviceName, name_len);
        adv_data_len += name_len;
    }

    gap_advertisements_set_data(adv_data_len, adv_data);
    gap_advertisements_enable(1);

    return true;
}

bool BLE_Transport::isConnected() {
    return connected && txNotifyEnabled;
}

int BLE_Transport::available() {
    if (rxHead >= rxTail) {
        return rxHead - rxTail;
    } else {
        return RX_BUFFER_SIZE - rxTail + rxHead;
    }
}

int BLE_Transport::read() {
    if (!available()) {
        return -1;
    }

    uint8_t data = rxBuffer[rxTail];
    rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
    return data;
}

size_t BLE_Transport::read(uint8_t* buffer, size_t size) {
    size_t count = 0;
    while (count < size && available() > 0) {
        buffer[count++] = read();
    }
    return count;
}

size_t BLE_Transport::write(uint8_t data) {
    if (!isConnected()) {
        return 0;
    }

    txBuffer[txHead] = data;
    txHead = (txHead + 1) % TX_BUFFER_SIZE;

    sendNotification();
    return 1;
}

size_t BLE_Transport::write(const uint8_t* buffer, size_t size) {
    if (!isConnected()) {
        return 0;
    }

    size_t written = 0;
    for (size_t i = 0; i < size; i++) {
        if (txBufferAvailable() == 0) {
            sendNotification();
            delay(10);
        }

        if (txBufferAvailable() > 0) {
            txBuffer[txHead] = buffer[i];
            txHead = (txHead + 1) % TX_BUFFER_SIZE;
            written++;
        } else {
            break;
        }
    }

    sendNotification();
    return written;
}

void BLE_Transport::flush() {
    if (isConnected() && txHead != txTail) {
        sendNotification();
    }
}

void BLE_Transport::setDeviceName(const char* name) {
    if (name) {
        strncpy(deviceName, name, sizeof(deviceName) - 1);
        deviceName[sizeof(deviceName) - 1] = '\0';
    }
}

const char* BLE_Transport::getDeviceName() {
    return deviceName;
}

// Static callback handlers
void BLE_Transport::packetHandler(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (!instance) return;

    if (packetType == HCI_EVENT_PACKET) {
        instance->handleHCIEvent(packet, size);
    }
}

uint16_t BLE_Transport::attReadCallback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t offset, uint8_t* buffer, uint16_t bufferSize) {
    UNUSED(conHandle);
    UNUSED(attHandle);
    UNUSED(offset);
    UNUSED(buffer);
    UNUSED(bufferSize);
    return 0;
}

int BLE_Transport::attWriteCallback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t transactionMode, uint16_t offset, uint8_t* buffer, uint16_t bufferSize) {
    UNUSED(transactionMode);
    UNUSED(offset);

    if (!instance) return 0;

    // Handle writes to RX characteristic
    if (attHandle == instance->rxCharValueHandle) {
        instance->addToRxBuffer(buffer, bufferSize);
        return 0;
    }

    // Handle CCCD writes
    if (attHandle == instance->txCharCCCDHandle) {
        instance->txNotifyEnabled = (buffer[0] == 0x01);
        return 0;
    }

    return 0;
}

void BLE_Transport::handleHCIEvent(uint8_t* packet, uint16_t size) {
    UNUSED(size);

    uint8_t event = hci_event_packet_get_type(packet);

    switch (event) {
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                conHandle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                connected = true;
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            connected = false;
            txNotifyEnabled = false;
            conHandle = HCI_CON_HANDLE_INVALID;
            rxHead = rxTail = 0;
            txHead = txTail = 0;
            gap_advertisements_enable(1);
            break;

        case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
            mtuSize = att_event_mtu_exchange_complete_get_MTU(packet);
            break;

        case ATT_EVENT_CAN_SEND_NOW:
            sendNotification();
            break;
    }
}

void BLE_Transport::handleATTEvent(uint8_t* packet, uint16_t size) {
    UNUSED(packet);
    UNUSED(size);
}

void BLE_Transport::sendNotification() {
    if (!isConnected() || txHead == txTail) {
        return;
    }

    uint16_t available = (txHead >= txTail) ? (txHead - txTail) : (TX_BUFFER_SIZE - txTail + txHead);
    uint16_t maxSend = mtuSize - 3;
    uint16_t toSend = (available < maxSend) ? available : maxSend;

    uint8_t notifyBuffer[maxSend];
    for (uint16_t i = 0; i < toSend; i++) {
        notifyBuffer[i] = txBuffer[txTail];
        txTail = (txTail + 1) % TX_BUFFER_SIZE;
    }

    att_server_notify(conHandle, txCharValueHandle, notifyBuffer, toSend);
}

uint16_t BLE_Transport::rxBufferAvailable() {
    if (rxHead >= rxTail) {
        return RX_BUFFER_SIZE - (rxHead - rxTail) - 1;
    } else {
        return rxTail - rxHead - 1;
    }
}

uint16_t BLE_Transport::txBufferAvailable() {
    if (txHead >= txTail) {
        return TX_BUFFER_SIZE - (txHead - txTail) - 1;
    } else {
        return txTail - txHead - 1;
    }
}

void BLE_Transport::addToRxBuffer(uint8_t data) {
    if (rxBufferAvailable() > 0) {
        rxBuffer[rxHead] = data;
        rxHead = (rxHead + 1) % RX_BUFFER_SIZE;
    }
}

void BLE_Transport::addToRxBuffer(const uint8_t* data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        addToRxBuffer(data[i]);
    }
}

// Global callback functions that forward to class instance
static uint16_t att_read_callback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t offset, uint8_t* buffer, uint16_t bufferSize) {
    if (BLE_Transport::instance) {
        return BLE_Transport::instance->attReadCallback(conHandle, attHandle, offset, buffer, bufferSize);
    }
    return 0;
}

static int att_write_callback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t transactionMode, uint16_t offset, uint8_t* buffer, uint16_t bufferSize) {
    if (BLE_Transport::instance) {
        return BLE_Transport::instance->attWriteCallback(conHandle, attHandle, transactionMode, offset, buffer, bufferSize);
    }
    return 0;
}