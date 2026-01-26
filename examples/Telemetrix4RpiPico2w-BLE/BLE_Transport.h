/*
 * BLE_Transport.h
 * BLE Nordic UART Service wrapper for Telemetrix4RpiPico2w
 *
 * This implementation uses BTstack's low-level C API to create a GATT server
 * with Nordic UART Service for BLE communication
 */

#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <Arduino.h>

class BLE_Transport {
private:
    // Circular buffer for received data
    static const uint16_t BUFFER_SIZE = 512;
    uint8_t rxBuffer[BUFFER_SIZE];
    volatile uint16_t rxHead;
    volatile uint16_t rxTail;

    // Connection state
    volatile bool connected;
    volatile uint16_t connection_handle;

    // Notification state
    volatile bool tx_notifications_enabled;

    // ATT handles (set during initialization)
    uint16_t att_tx_value_handle;
    uint16_t att_rx_value_handle;

    // Pending TX data
    static const uint16_t TX_BUFFER_SIZE = 244;
    uint8_t txBuffer[TX_BUFFER_SIZE];
    volatile uint16_t txLength;
    volatile bool txPending;

public:
    BLE_Transport();

    /**
     * Initialize BLE with Nordic UART Service
     * @param deviceName BLE advertised name
     */
    void begin(const char* deviceName = "Tmx4Pico2W");

    /**
     * Update BLE stack (call in loop())
     */
    void update();

    /**
     * Check if BLE is connected
     */
    bool isConnected() const {
        return connected;
    }

    /**
     * Check how many bytes are available to read
     */
    int available();

    /**
     * Read one byte from buffer
     * @return byte value or -1 if no data available
     */
    int read();

    /**
     * Peek at next byte without removing it
     */
    int peek();

    /**
     * Write a single byte
     */
    size_t write(uint8_t byte);

    /**
     * Write a buffer of bytes
     */
    size_t write(const uint8_t* buffer, size_t size);

    /**
     * Write a null-terminated string
     */
    size_t print(const char* str);

    /**
     * Write a string with newline
     */
    size_t println(const char* str);

    /**
     * Flush - no-op for BLE
     */
    void flush() {}

    // Public callbacks (called from C code)
    void handleRxData(const uint8_t* data, uint16_t size);
    void handleConnection(uint16_t conn_handle);
    void handleDisconnection();
    void handleTxNotificationsEnabled(bool enabled);
    void handleCanSendNow();

    // Get ATT handles
    uint16_t getTxValueHandle() const { return att_tx_value_handle; }
    uint16_t getRxValueHandle() const { return att_rx_value_handle; }
    void setTxValueHandle(uint16_t handle) { att_tx_value_handle = handle; }
    void setRxValueHandle(uint16_t handle) { att_rx_value_handle = handle; }
};

// Global instance
extern BLE_Transport bleTransport;

#endif // BLE_TRANSPORT_H