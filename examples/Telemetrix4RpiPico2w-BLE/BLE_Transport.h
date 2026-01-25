/*
 * BLE_Transport.h
 * BLE Nordic UART Service wrapper for Telemetrix4RpiPico2w
 *
 * This file provides drop-in replacements for Serial functions
 * to enable BLE transport with minimal changes to existing code
 */
extern "C" {
  #include "btstack.h"


#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <Arduino.h>
#include <BTstackLib.h>

class BLE_Transport {
private:
    // Circular buffer for received data
    static const uint16_t BUFFER_SIZE = 512;
    uint8_t rxBuffer[BUFFER_SIZE];
    volatile uint16_t rxHead;
    volatile uint16_t rxTail;

    // BLE characteristics
    BLECharacteristic* txChar;
    BLECharacteristic* rxChar;

    // Connection state
    volatile bool connected;

    // MTU size (default 20 bytes for BLE)
    uint16_t mtuSize;

public:
    BLE_Transport() : rxHead(0), rxTail(0), connected(false), mtuSize(20) {
        txChar = nullptr;
        rxChar = nullptr;
    }

    /**
     * Initialize BLE with Nordic UART Service
     * @param deviceName BLE advertised name
     */
    void begin(const char* deviceName = "Telemetrix4Pico2W") {
        // Clear buffer
        rxHead = 0;
        rxTail = 0;

        // Initialize BTstack
        BTstack.setup();
        BTstack.setBLEDeviceName(deviceName);

        // Create Nordic UART Service
        static BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

        // TX Characteristic (for sending data to client)
        static BLECharacteristic txCharacteristic(
            "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
            ATT_PROPERTY_NOTIFY | ATT_PROPERTY_READ,
            nullptr, 0
        );
        txChar = &txCharacteristic;

        // RX Characteristic (for receiving data from client)
        static BLECharacteristic rxCharacteristic(
            "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
            ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE,
            nullptr, 0
        );
        rxChar = &rxCharacteristic;

        // Set up RX callback using lambda
        rxChar->setValueCallback([](BLECharacteristic* characteristic,
                                     uint8_t* data, uint16_t size) {
            // Access the singleton instance
            extern BLE_Transport bleTransport;
            bleTransport.onRxData(data, size);
        });

        // Set up connection callback
        BTstack.setBLEConnectionCallback([](BLEStatus status,
                                            BLEConnection* connection) {
            extern BLE_Transport bleTransport;
            bleTransport.onConnectionChange(status, connection);
        });

        // Add characteristics to service
        uartService.addCharacteristic(txChar);
        uartService.addCharacteristic(rxChar);

        // Add service to BTstack
        BTstack.addGATTService(&uartService);

        // Start advertising
        BTstack.startAdvertising();
    }

    /**
     * Update BLE stack (call in loop())
     */
    void update() {
        BTstack.loop();
    }

    /**
     * Check if BLE is connected
     */
    bool isConnected() const {
        return connected;
    }

    /**
     * Check how many bytes are available to read
     */
    int available() {
        int count = 0;
        if (rxHead >= rxTail) {
            count = rxHead - rxTail;
        } else {
            count = BUFFER_SIZE - rxTail + rxHead;
        }
        return count;
    }

    /**
     * Read one byte from buffer
     * @return byte value or -1 if no data available
     */
    int read() {
        if (rxHead == rxTail) {
            return -1;
        }

        uint8_t data = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % BUFFER_SIZE;
        return data;
    }

    /**
     * Peek at next byte without removing it
     */
    int peek() {
        if (rxHead == rxTail) {
            return -1;
        }
        return rxBuffer[rxTail];
    }

    /**
     * Write a single byte
     */
    size_t write(uint8_t byte) {
        return write(&byte, 1);
    }

    /**
     * Write a buffer of bytes
     */
    size_t write(const uint8_t* buffer, size_t size) {
        if (!connected || txChar == nullptr) {
            return 0;
        }

        size_t written = 0;

        // Send in chunks based on MTU size
        while (written < size) {
            size_t chunkSize = min((size_t)mtuSize, size - written);

            // Send notification
            txChar->setValue(buffer + written, chunkSize);
            txChar->notify();

            written += chunkSize;

            // Small delay between chunks to prevent overflow
            if (written < size) {
                delay(5);
            }
        }

        return written;
    }

    /**
     * Write a null-terminated string
     */
    size_t print(const char* str) {
        return write((const uint8_t*)str, strlen(str));
    }

    /**
     * Write a string with newline
     */
    size_t println(const char* str) {
        size_t n = print(str);
        n += write('\r');
        n += write('\n');
        return n;
    }

    /**
     * Flush - no-op for BLE
     */
    void flush() {
        // BLE handles this automatically
    }

    /**
     * Callback when data is received
     */
    void onRxData(uint8_t* data, uint16_t size) {
        for (uint16_t i = 0; i < size; i++) {
            uint16_t nextHead = (rxHead + 1) % BUFFER_SIZE;
            if (nextHead != rxTail) {
                rxBuffer[rxHead] = data[i];
                rxHead = nextHead;
            }
            // Note: silently drop data if buffer is full
        }
    }

    /**
     * Callback when connection state changes
     */
    void onConnectionChange(BLEStatus status, BLEConnection* connection) {
        if (status == BLE_STATUS_OK) {
            connected = connection->isConnected();

            // Clear buffer on disconnect
            if (!connected) {
                rxHead = 0;
                rxTail = 0;
            }
        }
    }
};

// Global instance
extern BLE_Transport bleTransport;

#endif // BLE_TRANSPORT_H
