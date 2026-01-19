/*
 * BLE_Transport.h - BLE Nordic UART Service transport for Telemetrix4RpiPico2w
 * Based on working BLE NUS implementation
 *
 * IMPORTANT: Tools->IP/Bluetooth Stack = "IPv4 + Bluetooth"
 */

#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <Arduino.h>

extern "C" {
    #include "btstack.h"
}

#define RX_BUFFER_SIZE 512

class BLE_Transport {
private:
    // Circular buffer for received data
    static uint8_t rx_buffer[RX_BUFFER_SIZE];
    static volatile uint16_t rx_head;
    static volatile uint16_t rx_tail;

    // Connection state
    static hci_con_handle_t con_handle;
    static bool notifications_enabled;
    static btstack_packet_callback_registration_t hci_event_callback_registration;

    // Callbacks
    static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle,
                                  uint16_t transaction_mode, uint16_t offset,
                                  uint8_t *buffer, uint16_t buffer_size);
    static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle,
                                      uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
    static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

public:
    BLE_Transport();

    void begin(const char* deviceName = "Telemetrix4Pico");
    void end();

    // Stream-like interface for compatibility with Telemetrix
    int available();
    int read();
    int peek();

    size_t write(uint8_t byte);
    size_t write(const uint8_t *buffer, size_t size);
    void flush();

    bool isConnected() { return con_handle != HCI_CON_HANDLE_INVALID; }
};

#endif // BLE_TRANSPORT_H