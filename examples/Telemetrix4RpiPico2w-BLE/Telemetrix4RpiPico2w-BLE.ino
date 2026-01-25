/*
  Copyright (c) 2025 Alan Yorinks All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
  Version 3 as published by the Free Software Foundation; either
  or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
  along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// ============================================================
// USER CONFIGURABLE - Change this to set your device name
// ============================================================
#define BLE_DEVICE_NAME "Tmx4Pico2W"
// ============================================================

extern "C" {
  #include "btstack.h"
}

#include <Arduino.h>
#include <Servo.h>
#include <NanoConnectHcSr04.h>
#include <Wire.h>
#include <dhtnew.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <NeoPixelConnect.h>

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*         BLE Transport Layer - Nordic UART Service                */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Nordic UART Service UUIDs (little-endian)
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

// BLE Connection state
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static bool notifications_enabled = false;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Advertising data
static uint8_t adv_data[128];
static uint8_t adv_data_len = 0;
static uint8_t scan_resp_data[128];
static uint8_t scan_resp_data_len = 0;

// ATT Database
static uint8_t profile_data[256];

// BLE receive buffer for command assembly
#define BLE_RX_BUFFER_SIZE 256
static uint8_t ble_rx_buffer[BLE_RX_BUFFER_SIZE];
static uint16_t ble_rx_write_pos = 0;
static uint16_t ble_rx_read_pos = 0;

// BLE Transport Class
class BLETransport {
public:
    void begin(const char* deviceName) {
        // Device name is already set via BLE_DEVICE_NAME
    }

    int available() {
        return (ble_rx_write_pos - ble_rx_read_pos + BLE_RX_BUFFER_SIZE) % BLE_RX_BUFFER_SIZE;
    }

    int read() {
        if (!available()) return -1;
        uint8_t data = ble_rx_buffer[ble_rx_read_pos];
        ble_rx_read_pos = (ble_rx_read_pos + 1) % BLE_RX_BUFFER_SIZE;
        return data;
    }

    void write(uint8_t* data, size_t len) {
        if (con_handle != HCI_CON_HANDLE_INVALID && notifications_enabled) {
            att_server_notify(con_handle, 0x0008, data, len);
        }
    }

    void write(uint8_t data) {
        write(&data, 1);
    }

    void flush() {
        ble_rx_read_pos = ble_rx_write_pos;
    }
};

BLETransport bleTransport;

// ATT Write Callback - receives data from BLE client
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle,
                              uint16_t transaction_mode, uint16_t offset,
                              uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(connection_handle);

    if (att_handle == 0x0006) {  // RX characteristic - incoming commands
        // Add received data to buffer
        for (int i = 0; i < buffer_size; i++) {
            ble_rx_buffer[ble_rx_write_pos] = buffer[i];
            ble_rx_write_pos = (ble_rx_write_pos + 1) % BLE_RX_BUFFER_SIZE;
        }
    }
    else if (att_handle == 0x0009) {  // CCCD
        notifications_enabled = (buffer_size >= 2 && (buffer[0] & 0x01));
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
    UNUSED(att_handle);
    return 0;
}

// BLE Packet handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                gap_advertisements_enable(1);
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = HCI_CON_HANDLE_INVALID;
            notifications_enabled = false;
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

void init_ble_transport() {
    uint8_t name_len = strlen(BLE_DEVICE_NAME);
    uint8_t pos = 0;

    // Build ATT database
    profile_data[pos++] = 1;  // ATT DB Version

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    profile_data[pos++] = 0x0a; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x01; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x00; profile_data[pos++] = 0x28;
    profile_data[pos++] = 0x00; profile_data[pos++] = 0x18;

    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME
    profile_data[pos++] = 0x0d; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x03; profile_data[pos++] = 0x28;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x03; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x00; profile_data[pos++] = 0x2a;

    // 0x0003 VALUE-GAP_DEVICE_NAME
    uint8_t name_entry_len = 8 + name_len;
    profile_data[pos++] = name_entry_len; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x03; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x00; profile_data[pos++] = 0x2a;
    memcpy(&profile_data[pos], BLE_DEVICE_NAME, name_len);
    pos += name_len;

    // 0x0004 PRIMARY_SERVICE-Nordic UART Service
    profile_data[pos++] = 0x18; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x04; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x00; profile_data[pos++] = 0x28;
    memcpy(&profile_data[pos], nus_service_uuid, 16);
    pos += 16;

    // 0x0005 CHARACTERISTIC-NUS RX
    profile_data[pos++] = 0x1b; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x05; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x03; profile_data[pos++] = 0x28;
    profile_data[pos++] = 0x0c; profile_data[pos++] = 0x06; profile_data[pos++] = 0x00;
    memcpy(&profile_data[pos], nus_rx_uuid, 16);
    pos += 16;

    // 0x0006 VALUE-NUS RX
    profile_data[pos++] = 0x16; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x0c; profile_data[pos++] = 0x03;
    profile_data[pos++] = 0x06; profile_data[pos++] = 0x00;
    memcpy(&profile_data[pos], nus_rx_uuid, 16);
    pos += 16;

    // 0x0007 CHARACTERISTIC-NUS TX
    profile_data[pos++] = 0x1b; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x07; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x03; profile_data[pos++] = 0x28;
    profile_data[pos++] = 0x10; profile_data[pos++] = 0x08; profile_data[pos++] = 0x00;
    memcpy(&profile_data[pos], nus_tx_uuid, 16);
    pos += 16;

    // 0x0008 VALUE-NUS TX
    profile_data[pos++] = 0x16; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x10; profile_data[pos++] = 0x03;
    profile_data[pos++] = 0x08; profile_data[pos++] = 0x00;
    memcpy(&profile_data[pos], nus_tx_uuid, 16);
    pos += 16;

    // 0x0009 CLIENT_CHARACTERISTIC_CONFIGURATION
    profile_data[pos++] = 0x0a; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x0e; profile_data[pos++] = 0x01;
    profile_data[pos++] = 0x09; profile_data[pos++] = 0x00;
    profile_data[pos++] = 0x02; profile_data[pos++] = 0x29;
    profile_data[pos++] = 0x00; profile_data[pos++] = 0x00;

    // Build advertising data
    pos = 0;
    adv_data[pos++] = 0x02; adv_data[pos++] = 0x01; adv_data[pos++] = 0x06;
    adv_data[pos++] = 1 + name_len; adv_data[pos++] = 0x09;
    memcpy(&adv_data[pos], BLE_DEVICE_NAME, name_len);
    pos += name_len;
    adv_data[pos++] = 0x11; adv_data[pos++] = 0x07;
    memcpy(&adv_data[pos], nus_service_uuid, 16);
    pos += 16;
    adv_data_len = pos;

    // Scan response
    pos = 0;
    scan_resp_data[pos++] = 1 + name_len; scan_resp_data[pos++] = 0x09;
    memcpy(&scan_resp_data[pos], BLE_DEVICE_NAME, name_len);
    pos += name_len;
    scan_resp_data_len = pos;

    // Initialize BTStack
    l2cap_init();
    sm_init();
    att_server_init(profile_data, att_read_callback, att_write_callback);

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(packet_handler);

    bd_addr_t null_addr = {0, 0, 0, 0, 0, 0};
    gap_advertisements_set_params(0x0030, 0x0030, 0x00, (bd_addr_type_t)0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, adv_data);
    gap_scan_response_set_data(scan_resp_data_len, scan_resp_data);

    hci_power_control(HCI_POWER_ON);
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*         Client Command Related Defines and Support               */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Commands - maintaining sequential numbering
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define ANALOG_WRITE 3
#define MODIFY_REPORTING 4
#define GET_FIRMWARE_VERSION 5
#define SERVO_ATTACH 6
#define SERVO_WRITE 7
#define SERVO_DETACH 8
#define I2C_BEGIN 9
#define I2C_READ 10
#define I2C_WRITE 11
#define SONAR_NEW 12
#define DHT_NEW 13
#define STOP_ALL_REPORTS 14
#define SET_ANALOG_SCANNING_INTERVAL 15
#define ENABLE_ALL_REPORTS 16
#define RESET 17
#define SPI_INIT 18
#define SPI_WRITE_BLOCKING 19
#define SPI_READ_BLOCKING 20
#define SPI_SET_FORMAT 21
#define SPI_CS_CONTROL 22
#define ONE_WIRE_INIT 23
#define ONE_WIRE_RESET 24
#define ONE_WIRE_SELECT 25
#define ONE_WIRE_SKIP 26
#define ONE_WIRE_WRITE 27
#define ONE_WIRE_READ 28
#define ONE_WIRE_RESET_SEARCH 29
#define ONE_WIRE_SEARCH 30
#define ONE_WIRE_CRC8 31
#define SET_PIN_MODE_STEPPER 32
#define STEPPER_MOVE_TO 33
#define STEPPER_MOVE 34
#define STEPPER_RUN 35
#define STEPPER_RUN_SPEED 36
#define STEPPER_SET_MAX_SPEED 37
#define STEPPER_SET_ACCELERATION 38
#define STEPPER_SET_SPEED 39
#define STEPPER_SET_CURRENT_POSITION 40
#define STEPPER_RUN_SPEED_TO_POSITION 41
#define STEPPER_STOP 42
#define STEPPER_DISABLE_OUTPUTS 43
#define STEPPER_ENABLE_OUTPUTS 44
#define STEPPER_SET_MINIMUM_PULSE_WIDTH 45
#define STEPPER_SET_ENABLE_PIN 46
#define STEPPER_SET_3_PINS_INVERTED 47
#define STEPPER_SET_4_PINS_INVERTED 48
#define STEPPER_IS_RUNNING 49
#define STEPPER_GET_CURRENT_POSITION 50
#define STEPPER_GET_DISTANCE_TO_GO 51
#define STEPPER_GET_TARGET_POSITION 52
#define RESET_BOARD 53
#define INIT_NEOPIXELS 54
#define SHOW_NEOPIXELS 55
#define SET_NEOPIXEL 56
#define CLEAR_NEOPIXELS 57
#define FILL_NEOPIXELS 58
#define PWM_FREQ 59
#define PWM_RANGE 60
#define GET_CPU_TEMP 61
#define RETRIEVE_PICO_UNIQUE_ID 62

/* Command Forward References*/
extern void serial_loopback();
extern void set_pin_mode();
extern void digital_write();
extern void analog_write();
extern void modify_reporting();
extern void get_firmware_version();
extern void servo_attach();
extern void servo_write();
extern void servo_detach();
extern void i2c_begin();
extern void i2c_read();
extern void i2c_write();
extern void sonar_new();
extern void dht_new();
extern void stop_all_reports();
extern void set_analog_scanning_interval();
extern void enable_all_reports();
extern void reset_data();
extern void init_pin_structures();
extern void init_spi();
extern void write_blocking_spi();
extern void read_blocking_spi();
extern void set_format_spi();
extern void spi_cs_control();
extern void onewire_init();
extern void onewire_reset();
extern void onewire_select();
extern void onewire_skip();
extern void onewire_write();
extern void onewire_read();
extern void onewire_reset_search();
extern void onewire_search();
extern void onewire_crc8();
extern void set_pin_mode_stepper();
extern void stepper_move_to();
extern void stepper_move();
extern void stepper_run();
extern void stepper_run_speed();
extern void stepper_set_max_speed();
extern void stepper_set_acceleration();
extern void stepper_set_speed();
extern void stepper_get_distance_to_go();
extern void stepper_get_target_position();
extern void stepper_get_current_position();
extern void stepper_set_current_position();
extern void stepper_run_speed_to_position();
extern void stepper_stop();
extern void stepper_disable_outputs();
extern void stepper_enable_outputs();
extern void stepper_set_minimum_pulse_width();
extern void stepper_set_3_pins_inverted();
extern void stepper_set_4_pins_inverted();
extern void stepper_set_enable_pin();
extern void stepper_is_running();
extern void reset_board();
extern void init_neo_pixels();
extern void show_neo_pixels();
extern void set_neo_pixel();
extern void clear_all_neo_pixels();
extern void fill_neo_pixels();
extern void set_pwm_freq();
extern void set_pwm_range();
extern void get_cpu_temp();
extern void get_pico_unique_id() {
  byte unique_id_report_report_message[10] = { 9, UNIQUE_ID_REPORT, 0, 0, 0, 0, 0, 0, 0, 0 };
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);

  unique_id_report_report_message[2] = (board_id.id[0]);
  unique_id_report_report_message[3] = (board_id.id[1]);
  unique_id_report_report_message[4] = (board_id.id[2]);
  unique_id_report_report_message[5] = (board_id.id[3]);
  unique_id_report_report_message[6] = (board_id.id[4]);
  unique_id_report_report_message[7] = (board_id.id[5]);
  unique_id_report_report_message[8] = (board_id.id[6]);
  unique_id_report_report_message[9] = (board_id.id[7]);

  bleTransport.write(unique_id_report_report_message, 10);
}

void run_steppers() {
  boolean running;
  long target_position;

  for (int i = 0; i < MAX_NUMBER_OF_STEPPERS; i++) {
    if (stepper_run_modes[i] == STEPPER_STOP) {
      continue;
    } else {
      steppers[i]->enableOutputs();
      switch (stepper_run_modes[i]) {
        case STEPPER_RUN:
          steppers[i]->run();
          running = steppers[i]->isRunning();
          if (!running) {
            byte report_message[3] = { 2, STEPPER_RUN_COMPLETE_REPORT, (byte)i };
            bleTransport.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }
          break;
        case STEPPER_RUN_SPEED:
          steppers[i]->runSpeed();
          break;
        case STEPPER_RUN_SPEED_TO_POSITION:
          running = steppers[i]->runSpeedToPosition();
          target_position = steppers[i]->targetPosition();
          if (target_position == steppers[i]->currentPosition()) {
            byte report_message[3] = { 2, STEPPER_RUN_COMPLETE_REPORT, (byte)i };
            bleTransport.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }
          break;
        default:
          break;
      }
    }
  }
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                       Setup And Loop                             */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void setup() {
  // Initialize BLE transport
  init_ble_transport();
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);

  // LED blink pattern
  for (int blink = 0; blink < 2; blink++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  for (int i = 0; i < MAX_NUMBER_OF_STEPPERS; i++) {
    stepper_run_modes[i] = STEPPER_STOP;
  }

  analogWriteRange(20000);
  init_pin_structures();
}

void loop() {
  if (!rebooting) {
    get_next_command();

    if (!stop_reports) {
      scan_digital_inputs();
      scan_analog_inputs();
      scan_sonars();
      scan_dhts();
      scan_cpu_temp();
      run_steppers();
    }
  }
}id();

struct command_descriptor {
  void (*command_func)(void);
};

command_descriptor command_table[] = {
  { &serial_loopback },
  { &set_pin_mode },
  { &digital_write },
  { &analog_write },
  { &modify_reporting },
  { &get_firmware_version },
  { &servo_attach },
  { &servo_write },
  { &servo_detach },
  { &i2c_begin },
  { &i2c_read },
  { &i2c_write },
  { &sonar_new },
  { &dht_new },
  { &stop_all_reports },
  { &set_analog_scanning_interval },
  { &enable_all_reports },
  { &reset_data },
  { &init_spi },
  { &write_blocking_spi },
  { &read_blocking_spi },
  { &set_format_spi },
  { &spi_cs_control },
  { &onewire_init },
  { &onewire_reset },
  { &onewire_select },
  { &onewire_skip },
  { &onewire_write },
  { &onewire_read },
  { &onewire_reset_search },
  { &onewire_search },
  { &onewire_crc8 },
  { &set_pin_mode_stepper },
  { &stepper_move_to },
  { &stepper_move },
  { &stepper_run },
  { &stepper_run_speed },
  { &stepper_set_max_speed },
  { &stepper_set_acceleration },
  { &stepper_set_speed },
  { &stepper_set_current_position },
  { &stepper_run_speed_to_position },
  { &stepper_stop },
  { &stepper_disable_outputs },
  { &stepper_enable_outputs },
  { &stepper_set_minimum_pulse_width },
  { &stepper_set_enable_pin },
  { &stepper_set_3_pins_inverted },
  { &stepper_set_4_pins_inverted },
  { &stepper_is_running },
  { &stepper_get_current_position },
  { &stepper_get_distance_to_go },
  { &stepper_get_target_position },
  { &reset_board },
  { &init_neo_pixels },
  { &show_neo_pixels },
  { &set_neo_pixel },
  { &clear_all_neo_pixels },
  { &fill_neo_pixels },
  { &set_pwm_freq },
  { &set_pwm_range },
  { &get_cpu_temp },
  { &get_pico_unique_id },
};

#define MAX_COMMAND_LENGTH 30
byte command_buffer[MAX_COMMAND_LENGTH];

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                 Reporting Defines and Support                    */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT 3
#define FIRMWARE_REPORT 5
#define I_AM_HERE 6
#define SERVO_UNAVAILABLE 7
#define I2C_TOO_FEW_BYTES_RCVD 8
#define I2C_TOO_MANY_BYTES_RCVD 9
#define I2C_READ_REPORT 10
#define SONAR_DISTANCE 11
#define DHT_REPORT 12
#define SPI_REPORT 13
#define ONE_WIRE_REPORT 14
#define STEPPER_DISTANCE_TO_GO 15
#define STEPPER_TARGET_POSITION 16
#define STEPPER_CURRENT_POSITION 17
#define STEPPER_RUNNING_REPORT 18
#define STEPPER_RUN_COMPLETE_REPORT 19
#define CPU_TEMP_REPORT 20
#define UNIQUE_ID_REPORT 62
#define DEBUG_PRINT 99

byte i2c_report_message[64];
TwoWire *current_i2c_port = NULL;
SPIClassRP2040 *current_spi_port = NULL;

uint32_t clock0;
BitOrder bitOrder0;
SPIMode dataMode0;
uint8_t chipSelect0;

uint32_t clock1;
BitOrder bitOrder1;
SPIMode dataMode1;
uint8_t chipSelect1;

byte spi_report_message[64];
bool stop_reports = false;
bool rebooting = false;

#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

#define DHT_DATA_PIN 1
#define DHT_DATA 0
#define DHT_READ_ERROR 1

#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 0
#define TRANSPORT_TYPE 0

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*           Pin Related Defines And Data Structures                */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define AT_ANALOG 4
#define AT_MODE_NOT_SET 255

#define MAX_DIGITAL_PINS_SUPPORTED 100
#define MAX_ANALOG_PINS_SUPPORTED 15
#define I2C_NO_REGISTER 254

int analog_read_pins[4] = { A0, A1, A2, A3 };

struct pin_descriptor {
  byte pin_number;
  byte pin_mode;
  bool reporting_enabled;
  int last_value;
};

pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

struct analog_pin_descriptor {
  byte pin_number;
  byte pin_mode;
  bool reporting_enabled;
  int last_value;
  int differential;
};

analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

unsigned long current_millis;
unsigned long previous_millis;
uint8_t analog_sampling_interval = 19;

float cpu_temp_threshold = 0.0;
float cpu_temp_last_value = 0.0;
bool monitor_cpu_temp = false;
unsigned long cpu_temp_current_millis;
unsigned long cpu_temp_previous_millis;
uint16_t cpu_temp_sampling_interval = 1000;

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*  Feature Related Defines, Data Structures and Storage Allocation */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define MAX_SERVOS 8
Servo servos[MAX_SERVOS];
byte pin_to_servo_index_map[MAX_SERVOS];

#define MAX_SONARS 6
struct Sonar {
  uint8_t trigger_pin;
  unsigned int last_value;
  NanoConnectHcSr04 *usonic;
};

Sonar sonars[MAX_SONARS];
byte sonars_index = 0;
byte last_sonar_visited = 0;
unsigned long sonar_current_millis;
unsigned long sonar_previous_millis;
uint8_t sonar_scan_interval = 33;

#define MAX_DHTS 2
NeoPixelConnect *np;

struct DHT {
  uint8_t pin;
  uint8_t dht_type;
  unsigned int last_value;
  DHTNEW *dht_sensor;
};

DHT dhts[MAX_DHTS];
byte dht_index = 0;
unsigned long dht_current_millis;
unsigned long dht_previous_millis;
unsigned int dht_scan_interval = 2200;

#define SPI0_MISO 16
#define SPI1_MISO 12
#define SPI0_MOSI 19
#define SPI1_MOSI 15
#define SPI0_CLK 18
#define SPI1_CLK 14

#define MAX_NUMBER_OF_STEPPERS 4
AccelStepper *steppers[MAX_NUMBER_OF_STEPPERS];
uint8_t stepper_run_modes[MAX_NUMBER_OF_STEPPERS];

// Stepper run mode constants
#define STEPPER_STOP 0
#define STEPPER_RUN 1
#define STEPPER_RUN_SPEED 2
#define STEPPER_RUN_SPEED_TO_POSITION 3

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                       Command Functions                          */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void send_debug_info(byte id, int value) {
  byte debug_buffer[5] = { (byte)4, (byte)DEBUG_PRINT, 0, 0, 0 };
  debug_buffer[2] = id;
  debug_buffer[3] = highByte(value);
  debug_buffer[4] = lowByte(value);
  bleTransport.write(debug_buffer, 5);
}

void serial_loopback() {
  byte loop_back_buffer[3] = { 2, (byte)SERIAL_LOOP_BACK, command_buffer[0] };
  bleTransport.write(loop_back_buffer, 3);
}

void set_pin_mode() {
  byte pin = command_buffer[0];
  byte mode = command_buffer[1];

  switch (mode) {
    case INPUT:
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT_PULLUP);
      break;
    case INPUT_PULLDOWN:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT_PULLDOWN);
      break;
    case OUTPUT:
      the_digital_pins[pin].pin_mode = mode;
      pinMode(pin, OUTPUT);
      break;
    case AT_ANALOG:
      the_analog_pins[pin].pin_mode = mode;
      the_analog_pins[pin].differential = (command_buffer[2] << 8) + command_buffer[3];
      the_analog_pins[pin].reporting_enabled = command_buffer[4];
      break;
    default:
      break;
  }
}

void set_analog_scanning_interval() {
  analog_sampling_interval = command_buffer[0];
}

void digital_write() {
  byte pin = command_buffer[0];
  byte value = command_buffer[1];
  digitalWrite(pin, value);
}

void analog_write() {
  byte pin = command_buffer[0];
  unsigned int value = (command_buffer[1] << 8) + command_buffer[2];
  analogWrite(pin, value);
}

void set_pwm_freq() {
  uint32_t frequency = (command_buffer[0] << 24) + (command_buffer[1] << 16) +
                       (command_buffer[2] << 8) + command_buffer[3];
  analogWriteFreq(frequency);
}

void set_pwm_range() {
  uint32_t pwm_range = (command_buffer[0] << 24) + (command_buffer[1] << 16) +
                       (command_buffer[2] << 8) + command_buffer[3];
  analogWriteRange(pwm_range);
}

void get_cpu_temp() {
  memcpy(&cpu_temp_threshold, &command_buffer[0], sizeof(float));
  cpu_temp_sampling_interval = (command_buffer[4] << 8) + command_buffer[5];
  monitor_cpu_temp = true;
}

void modify_reporting() {
  int pin = command_buffer[1];

  switch (command_buffer[0]) {
    case REPORTING_DISABLE_ALL:
      for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        the_digital_pins[i].reporting_enabled = false;
      }
      for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
        the_analog_pins[i].reporting_enabled = false;
      }
      break;
    case REPORTING_ANALOG_ENABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_analog_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_ANALOG_DISABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_analog_pins[pin].reporting_enabled = false;
      }
      break;
    case REPORTING_DIGITAL_ENABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_digital_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_DIGITAL_DISABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
        the_digital_pins[pin].reporting_enabled = false;
      }
      break;
    default:
      break;
  }
}

void reset_board() {
  stop_all_reports();
  bleTransport.flush();
  delay(100);
  rebooting = true;
  rp2040.reboot();
}

void get_firmware_version() {
  byte report_message[5] = { 4, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR, TRANSPORT_TYPE };
  bleTransport.write(report_message, 5);
}

int find_servo() {
  int index = -1;
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (servos[i].attached() == false) {
      index = i;
      break;
    }
  }
  return index;
}

void servo_attach() {
  byte pin = command_buffer[0];
  int servo_found = -1;
  int minpulse = (command_buffer[1] << 8) + command_buffer[2];
  int maxpulse = (command_buffer[3] << 8) + command_buffer[4];

  servo_found = find_servo();
  if (servo_found != -1) {
    pin_to_servo_index_map[servo_found] = pin;
    servos[servo_found].attach(pin, minpulse, maxpulse);
  } else {
    byte report_message[2] = { SERVO_UNAVAILABLE, pin };
    bleTransport.write(report_message, 2);
  }
}

void servo_write() {
  byte pin = command_buffer[0];
  int angle = command_buffer[1];
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (pin_to_servo_index_map[i] == pin) {
      servos[i].write(angle);
      return;
    }
  }
}

void servo_detach() {
  byte pin = command_buffer[0];
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (pin_to_servo_index_map[i] == pin) {
      pin_to_servo_index_map[i] = -1;
      servos[i].detach();
    }
  }
}

void i2c_begin() {
  byte i2c_port = command_buffer[0];
  if (i2c_port == 0) {
    Wire.begin();
  } else {
    Wire1.begin();
  }
}

void i2c_read() {
  int message_size = 0;
  byte port = command_buffer[0];
  byte address = command_buffer[1];
  byte the_register = command_buffer[2];
  byte num_of_bytes = command_buffer[3];
  bool stop = bool(command_buffer[4]);

  if (port == 0) {
    current_i2c_port = &Wire;
  } else {
    current_i2c_port = &Wire1;
  }

  if (the_register != I2C_NO_REGISTER) {
    current_i2c_port->beginTransmission(address);
    current_i2c_port->write((byte)the_register);
    current_i2c_port->endTransmission(stop);
  }

  current_i2c_port->requestFrom(address, num_of_bytes, stop);
  delay(10);

  if (num_of_bytes < current_i2c_port->available()) {
    byte report_message[4] = { 3, I2C_TOO_MANY_BYTES_RCVD, 1, address };
    bleTransport.write(report_message, 4);
    return;
  } else if (num_of_bytes > current_i2c_port->available()) {
    byte report_message[4] = { 3, I2C_TOO_FEW_BYTES_RCVD, 1, address };
    bleTransport.write(report_message, 4);
    return;
  }

  i2c_report_message[0] = num_of_bytes + 5;
  i2c_report_message[1] = I2C_READ_REPORT;
  i2c_report_message[2] = port;
  i2c_report_message[3] = num_of_bytes;
  i2c_report_message[4] = address;
  i2c_report_message[5] = the_register;

  for (message_size = 0; message_size < num_of_bytes && current_i2c_port->available(); message_size++) {
    i2c_report_message[6 + message_size] = current_i2c_port->read();
  }

  for (int i = 0; i < message_size + 6; i++) {
    bleTransport.write(i2c_report_message[i]);
  }
}

void i2c_write() {
  if (command_buffer[0] == 0) {
    current_i2c_port = &Wire;
  } else {
    current_i2c_port = &Wire1;
  }

  current_i2c_port->beginTransmission(command_buffer[1]);
  for (int i = 0; i < command_buffer[2]; i++) {
    current_i2c_port->write(command_buffer[i + 3]);
  }
  current_i2c_port->endTransmission();
  delayMicroseconds(70);
}

void sonar_new() {
  sonars[sonars_index].usonic = new NanoConnectHcSr04((uint8_t)command_buffer[0],
                                                       (uint8_t)command_buffer[1], pio0, 1);
  sonars[sonars_index].trigger_pin = command_buffer[0];
  sonars_index++;
}

void dht_new() {
  if (dht_index < MAX_DHTS) {
    dhts[dht_index].dht_sensor = new DHTNEW(command_buffer[0]);
    dhts[dht_index].pin = command_buffer[0];
    dhts[dht_index].dht_type = command_buffer[1];
    dht_index++;
  }
}

void init_spi() {
  int spi_port = command_buffer[0];
  int chip_select = command_buffer[1];

  if (spi_port == 0) {
    clock0 = (uint32_t)(command_buffer[2]) << 24;
    clock0 += (uint32_t)(command_buffer[3]) << 16;
    clock0 += command_buffer[4] << 8;
    clock0 += command_buffer[5];
    bitOrder0 = (BitOrder)command_buffer[6];
    dataMode0 = (SPIMode)command_buffer[7];
    chipSelect0 = chip_select;
    current_spi_port = &SPI;
  } else {
    clock1 = (uint32_t)(command_buffer[2]) << 24;
    clock1 += (uint32_t)(command_buffer[3]) << 16;
    clock1 += command_buffer[4] << 8;
    clock1 += command_buffer[5];
    bitOrder1 = (BitOrder)command_buffer[6];
    dataMode1 = (SPIMode)command_buffer[7];
    chipSelect1 = chip_select;
    current_spi_port = &SPI1;
  }

  current_spi_port->begin();
  pinMode(chip_select, OUTPUT);
  digitalWrite(chip_select, HIGH);
}

void write_blocking_spi() {
  int chipSelect;
  int num_bytes = command_buffer[1];

  if (command_buffer[0] == 0) {
    current_spi_port = &SPI;
    SPISettings mySetting(clock0, bitOrder0, dataMode0);
    chipSelect = chipSelect0;
    current_spi_port->beginTransaction(mySetting);
  } else {
    current_spi_port = &SPI1;
    SPISettings mySetting(clock1, bitOrder1, dataMode1);
    chipSelect = chipSelect1;
    current_spi_port->beginTransaction(mySetting);
  }

  digitalWrite(chipSelect, LOW);
  for (int i = 0; i < num_bytes; i++) {
    current_spi_port->transfer(command_buffer[2 + i]);
  }
  digitalWrite(chipSelect, HIGH);
  current_spi_port->endTransaction();
}

void read_blocking_spi() {
  int chipSelect;
  uint8_t spi_register = command_buffer[1];
  uint8_t number_of_bytes = command_buffer[2];

  if (command_buffer[0] == 0) {
    current_spi_port = &SPI;
    SPISettings mySetting(clock0, bitOrder0, dataMode0);
    chipSelect = chipSelect0;
    current_spi_port->beginTransaction(mySetting);
  } else {
    current_spi_port = &SPI1;
    SPISettings mySetting(clock1, bitOrder1, dataMode1);
    chipSelect = chipSelect1;
    current_spi_port->beginTransaction(mySetting);
  }

  spi_report_message[0] = number_of_bytes + 5;
  spi_report_message[1] = SPI_REPORT;
  spi_report_message[2] = command_buffer[0];
  spi_report_message[3] = spi_register;
  spi_report_message[4] = number_of_bytes;

  digitalWrite(chipSelect, LOW);
  current_spi_port->transfer(spi_register | 0x80);

  for (int i = 0; i < number_of_bytes; i++) {
    spi_report_message[i + 5] = current_spi_port->transfer(0x00);
  }
  digitalWrite(chipSelect, HIGH);
  current_spi_port->endTransaction();

  bleTransport.write(spi_report_message, number_of_bytes + 6);
}

void set_format_spi() {}
void spi_cs_control() {}

// OneWire placeholders
void onewire_init() {}
void onewire_reset() {}
void onewire_select() {}
void onewire_skip() {}
void onewire_write() {}
void onewire_read() {}
void onewire_reset_search() {}
void onewire_search() {}
void onewire_crc8() {}

void set_pin_mode_stepper() {
  steppers[command_buffer[0]] = new AccelStepper(command_buffer[1], command_buffer[2],
                                                 command_buffer[3], command_buffer[4],
                                                 command_buffer[5], command_buffer[6]);
}

void init_neo_pixels() {
  byte pin_number = command_buffer[0];
  byte num_pixels = command_buffer[1];
  np = new NeoPixelConnect(pin_number, num_pixels);
}

void show_neo_pixels() {
  np->neoPixelShow();
}

void set_neo_pixel() {
  np->neoPixelSetValue(command_buffer[0], command_buffer[1],
                       command_buffer[2], command_buffer[3], command_buffer[4]);
}

void clear_all_neo_pixels() {
  np->neoPixelClear(command_buffer[0]);
}

void fill_neo_pixels() {
  np->neoPixelFill(command_buffer[0], command_buffer[1],
                   command_buffer[2], command_buffer[3]);
}

void stepper_move_to() {
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];
  if (command_buffer[5]) {
    position *= -1;
  }
  steppers[command_buffer[0]]->moveTo(position);
}

void stepper_move() {
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];
  if (command_buffer[5]) {
    position *= -1;
  }
  steppers[command_buffer[0]]->move(position);
}

void stepper_run() {
  stepper_run_modes[command_buffer[0]] = STEPPER_RUN;
}

void stepper_run_speed() {
  stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED;
}

void stepper_set_max_speed() {
  float max_speed = (float)((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setMaxSpeed(max_speed);
}

void stepper_set_acceleration() {
  float acceleration = (float)((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setAcceleration(acceleration);
}

void stepper_set_speed() {
  float speed = (float)((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setSpeed(speed);
}

void stepper_get_distance_to_go() {
  byte report_message[7] = { 6, STEPPER_DISTANCE_TO_GO, command_buffer[0] };
  long dtg = steppers[command_buffer[0]]->distanceToGo();
  report_message[3] = (byte)((dtg & 0xFF000000) >> 24);
  report_message[4] = (byte)((dtg & 0x00FF0000) >> 16);
  report_message[5] = (byte)((dtg & 0x0000FF00) >> 8);
  report_message[6] = (byte)((dtg & 0x000000FF));
  bleTransport.write(report_message, 7);
}

void stepper_get_target_position() {
  byte report_message[7] = { 6, STEPPER_TARGET_POSITION, command_buffer[0] };
  long target = steppers[command_buffer[0]]->targetPosition();
  report_message[3] = (byte)((target & 0xFF000000) >> 24);
  report_message[4] = (byte)((target & 0x00FF0000) >> 16);
  report_message[5] = (byte)((target & 0x0000FF00) >> 8);
  report_message[6] = (byte)((target & 0x000000FF));
  bleTransport.write(report_message, 7);
}

void stepper_get_current_position() {
  byte report_message[7] = { 6, STEPPER_CURRENT_POSITION, command_buffer[0] };
  long position = steppers[command_buffer[0]]->currentPosition();
  report_message[3] = (byte)((position & 0xFF000000) >> 24);
  report_message[4] = (byte)((position & 0x00FF0000) >> 16);
  report_message[5] = (byte)((position & 0x0000FF00) >> 8);
  report_message[6] = (byte)((position & 0x000000FF));
  bleTransport.write(report_message, 7);
}

void stepper_set_current_position() {
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];
  steppers[command_buffer[0]]->setCurrentPosition(position);
}

void stepper_run_speed_to_position() {
  stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED_TO_POSITION;
}

void stepper_stop() {
  steppers[command_buffer[0]]->stop();
  steppers[command_buffer[0]]->disableOutputs();
  stepper_run_modes[command_buffer[0]] = STEPPER_STOP;
}

void stepper_disable_outputs() {
  steppers[command_buffer[0]]->disableOutputs();
}

void stepper_enable_outputs() {
  steppers[command_buffer[0]]->enableOutputs();
}

void stepper_set_minimum_pulse_width() {
  unsigned int pulse_width = (command_buffer[1] << 8) + command_buffer[2];
  steppers[command_buffer[0]]->setMinPulseWidth(pulse_width);
}

void stepper_set_enable_pin() {
  steppers[command_buffer[0]]->setEnablePin((uint8_t)command_buffer[1]);
}

void stepper_set_3_pins_inverted() {
  steppers[command_buffer[0]]->setPinsInverted((bool)command_buffer[1],
                                               (bool)command_buffer[2],
                                               (bool)command_buffer[3]);
}

void stepper_set_4_pins_inverted() {
  steppers[command_buffer[0]]->setPinsInverted((bool)command_buffer[1],
                                               (bool)command_buffer[2],
                                               (bool)command_buffer[3],
                                               (bool)command_buffer[4],
                                               (bool)command_buffer[5]);
}

void stepper_is_running() {
  byte report_message[4] = { 3, STEPPER_RUNNING_REPORT, command_buffer[0] };
  report_message[3] = steppers[command_buffer[0]]->isRunning();
  bleTransport.write(report_message, 4);
}

void stop_all_reports() {
  stop_reports = true;
  delay(20);
  bleTransport.flush();
}

void enable_all_reports() {
  bleTransport.flush();
  stop_reports = false;
  delay(20);
}

void get_next_command() {
  byte command;
  byte packet_length;
  command_descriptor command_entry;

  memset(command_buffer, 0, sizeof(command_buffer));

  if (!bleTransport.available()) {
    return;
  }

  packet_length = (byte)bleTransport.read();

  while (!bleTransport.available()) {
    delay(1);
  }

  command = (byte)bleTransport.read();
  command_entry = command_table[command];

  if (packet_length > 1) {
    for (int i = 0; i < packet_length - 1; i++) {
      while (!bleTransport.available()) {
        delay(1);
      }
      command_buffer[i] = (byte)bleTransport.read();
    }
  }
  command_entry.command_func();
}

void reset_data() {
  stop_all_reports();
  current_millis = 0;
  previous_millis = 0;
  analog_sampling_interval = 19;

  for (int i = 0; i < MAX_SERVOS; i++) {
    if (servos[i].attached() == true) {
      servos[i].detach();
    }
  }

  sonars_index = 0;
  sonar_current_millis = 0;
  sonar_previous_millis = 0;
  sonar_scan_interval = 33;
  memset(sonars, 0, sizeof(sonars));

  dht_index = 0;
  dht_current_millis = 0;
  dht_previous_millis = 0;
  dht_scan_interval = 2400;
  memset(dhts, 0, sizeof(dhts));
  enable_all_reports();
}

void init_pin_structures() {
  for (byte i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_digital_pins[i].reporting_enabled = false;
    the_digital_pins[i].last_value = 0;
  }

  for (byte i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
    the_analog_pins[i].pin_number = i;
    the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = 0;
    the_analog_pins[i].differential = 0;
  }
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*    Scanning Inputs, Generating Reports And Running Steppers      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void scan_digital_inputs() {
  byte value;
  byte report_message[4] = { 3, DIGITAL_REPORT, 0, 0 };

  for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
    if (the_digital_pins[i].pin_mode == INPUT || the_digital_pins[i].pin_mode == INPUT_PULLUP) {
      if (the_digital_pins[i].reporting_enabled) {
        value = (byte)digitalRead(the_digital_pins[i].pin_number);
        if (value != the_digital_pins[i].last_value) {
          the_digital_pins[i].last_value = value;
          report_message[2] = (byte)i;
          report_message[3] = value;
          bleTransport.write(report_message, 4);
        }
      }
    }
  }
}

void scan_analog_inputs() {
  int value;
  byte report_message[5] = { 4, ANALOG_REPORT, 0, 0, 0 };
  uint8_t adjusted_pin_number;
  int differential;

  current_millis = millis();
  if (current_millis - previous_millis > analog_sampling_interval) {
    previous_millis = current_millis;

    for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
      if (the_analog_pins[i].pin_mode == AT_ANALOG) {
        if (the_analog_pins[i].reporting_enabled) {
          adjusted_pin_number = (uint8_t)(analog_read_pins[i]);
          value = analogRead(adjusted_pin_number);
          differential = abs(value - the_analog_pins[i].last_value);
          if (differential >= the_analog_pins[i].differential) {
            the_analog_pins[i].last_value = value;
            report_message[2] = (byte)i;
            report_message[3] = highByte(value);
            report_message[4] = lowByte(value);
            bleTransport.write(report_message, 5);
            delay(1);
          }
        }
      }
    }
  }
}

void scan_cpu_temp() {
  float cpu_temp, differential;
  char output[sizeof(float)];
  byte report_message[6] = { 5, CPU_TEMP_REPORT, 0, 0, 0, 0 };

  if (monitor_cpu_temp) {
    cpu_temp_current_millis = millis();
    if (cpu_temp_current_millis - cpu_temp_previous_millis > cpu_temp_sampling_interval) {
      cpu_temp_previous_millis = cpu_temp_current_millis;
      cpu_temp = analogReadTemp();
      differential = abs(cpu_temp - cpu_temp_last_value);
      if (differential >= cpu_temp_threshold) {
        cpu_temp_last_value = cpu_temp;
        memcpy(output, &cpu_temp, sizeof(float));
        report_message[2] = output[0];
        report_message[3] = output[1];
        report_message[4] = output[2];
        report_message[5] = output[3];
        bleTransport.write(report_message, 6);
      }
    }
  }
}

void scan_sonars() {
  float distance, j, f;
  uint8_t integ, frac;

  if (sonars_index) {
    sonar_current_millis = millis();
    if (sonar_current_millis - sonar_previous_millis > sonar_scan_interval) {
      sonar_previous_millis = sonar_current_millis;
      distance = sonars[last_sonar_visited].usonic->readSonar();
      if (distance != sonars[last_sonar_visited].last_value) {
        sonars[last_sonar_visited].last_value = distance;
        f = modff(distance, &j);
        integ = (uint8_t)j;
        frac = (uint8_t)f;
        byte report_message[5] = { 4, SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin, integ, frac };
        bleTransport.write(report_message, 5);
      }
      last_sonar_visited++;
      if (last_sonar_visited == sonars_index) {
        last_sonar_visited = 0;
      }
    }
  }
}

void scan_dhts() {
  byte report_message[10] = { 9, DHT_REPORT, DHT_DATA, 0, 0, 0, 0, 0, 0, 0 };
  int rv;
  float humidity, temperature, j, f;

  if (dht_index) {
    dht_current_millis = millis();
    if (dht_current_millis - dht_previous_millis > dht_scan_interval) {
      dht_previous_millis = dht_current_millis;

      for (int i = 0; i < dht_index; i++) {
        report_message[1] = DHT_REPORT;
        report_message[3] = dhts[i].pin;
        rv = dhts[i].dht_sensor->read();

        if (rv != DHTLIB_OK) {
          rv = 0xff;
        }
        report_message[2] = (uint8_t)rv;

        if (rv) {
          bleTransport.write(report_message, 10);
          return;
        } else {
          humidity = dhts[i].dht_sensor->getHumidity();
          if (humidity >= 0.0) {
            report_message[4] = 0;
          } else {
            report_message[4] = 1;
          }
          f = modff(humidity, &j);
          report_message[6] = (uint8_t)j;
          report_message[7] = (uint8_t)(f * 100);

          temperature = dhts[i].dht_sensor->getTemperature();
          if (temperature >= 0.0) {
            report_message[5] = 0;
          } else {
            report_message[5] = 1;
          }
          f = modff(temperature, &j);
          report_message[8] = (uint8_t)j;
          report_message[9] = (uint8_t)(f * 100);
          bleTransport.write(report_message, 10);
        }
      }
    }
  }
}

void get_pico_unique_pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT);
      break;
    case INPUT_PULLUP:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].