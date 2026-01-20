/*
 * BLE_Transport.h
 * Telemetrix4RpiPico2w - BLE Nordic UART Service Transport
 *
 * This class implements BLE Nordic UART Service (NUS) transport for Telemetrix
 * using the BTStack library provided by arduino-pico
 */

#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <Arduino.h>
#include <btstack.h>

// Default device name for advertising
#define DEFAULT_DEVICE_NAME "Tmx4Pico2W"

// Maximum MTU size for BLE
#define MAX_MTU_SIZE 512
#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 512

class BLE_Transport {
public:
    BLE_Transport();

    // Initialize BLE transport with optional custom device name
    bool begin(const char* deviceName = DEFAULT_DEVICE_NAME);

    // Check if a client is connected
    bool isConnected();

    // Read data from BLE (non-blocking)
    int available();
    int read();
    size_t read(uint8_t* buffer, size_t size);

    // Write data to BLE
    size_t write(uint8_t data);
    size_t write(const uint8_t* buffer, size_t size);

    // Flush any pending data
    void flush();

    // Set the device name for advertising
    void setDeviceName(const char* name);

    // Get current device name
    const char* getDeviceName();

    // Static instance pointer for callbacks
    static BLE_Transport* instance;

    // Public callback methods (called from global C callbacks)
    uint16_t attReadCallback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t offset, uint8_t* buffer, uint16_t bufferSize);
    int attWriteCallback(hci_con_handle_t conHandle, uint16_t attHandle, uint16_t transactionMode, uint16_t offset, uint8_t* buffer, uint16_t bufferSize);

private:
    // BLE connection handle
    hci_con_handle_t conHandle;

    // Characteristic value handles
    uint16_t rxCharValueHandle;
    uint16_t txCharValueHandle;
    uint16_t txCharCCCDHandle;

    // Connection state
    bool connected;
    bool txNotifyEnabled;

    // Device name
    char deviceName[32];

    // Buffers for RX and TX
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint16_t rxHead;
    uint16_t rxTail;

    uint8_t txBuffer[TX_BUFFER_SIZE];
    uint16_t txHead;
    uint16_t txTail;

    // MTU size
    uint16_t mtuSize;

    // Static callback for BTStack
    static void packetHandler(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size);

    // Helper methods
    void handleHCIEvent(uint8_t* packet, uint16_t size);
    void handleATTEvent(uint8_t* packet, uint16_t size);
    void sendNotification();
    uint16_t rxBufferAvailable();
    uint16_t txBufferAvailable();
    void addToRxBuffer(uint8_t data);
    void addToRxBuffer(const uint8_t* data, uint16_t len);
};

#endif // BLE_TRANSPORT_H