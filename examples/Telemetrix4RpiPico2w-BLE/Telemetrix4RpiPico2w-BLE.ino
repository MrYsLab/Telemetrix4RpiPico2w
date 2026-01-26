/*
  Copyright (c) 2025 Alan Yorinks All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
  Version 3 as published by the Free Software Foundation; either
  or (at your option) any later version.
  This library is distributed in the hope that it will be useful,f
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
  along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include "Telemetrix4RpiPico2w.h"
#include <Servo.h>
#include <NanoConnectHcSr04.h>
#include <Wire.h>
#include <dhtnew.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <NeoPixelConnect.h>
#include <BLE.h>

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                     BLE Configuration                             */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// User-configurable device name
#define BLE_DEVICE_NAME "Telemetrix-Pico2W"

// Nordic UART Service UUIDs
#define NORDIC_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NORDIC_UART_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NORDIC_UART_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// BLE objects
BLEService uartService(NORDIC_UART_SERVICE_UUID);
BLECharacteristic txChar(NORDIC_UART_TX_UUID, BLERead | BLENotify, 512);
BLECharacteristic rxChar(NORDIC_UART_RX_UUID, BLEWrite | BLEWriteWithoutResponse, 512);

// BLE connection state
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Buffer for incoming BLE data
#define BLE_BUFFER_SIZE 512
uint8_t bleReceiveBuffer[BLE_BUFFER_SIZE];
int bleReceiveIndex = 0;

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*         Client Command Related Defines and Support               */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Commands Sent By The Client
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
extern void reset_board();
extern void set_pwm_freq();
extern void set_pwm_range();
extern void get_cpu_temp();
extern void get_pico_unique_id();

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
  (&stepper_set_current_position),
  (&stepper_run_speed_to_position),
  (&stepper_stop),
  (&stepper_disable_outputs),
  (&stepper_enable_outputs),
  (&stepper_set_minimum_pulse_width),
  (&stepper_set_enable_pin),
  (&stepper_set_3_pins_inverted),
  (&stepper_set_4_pins_inverted),
  (&stepper_is_running),
  (&stepper_get_current_position),
  { &stepper_get_distance_to_go },
  (&stepper_get_target_position),
  (&reset_board),
  { init_neo_pixels },
  { show_neo_pixels },
  { set_neo_pixel },
  { clear_all_neo_pixels },
  { fill_neo_pixels },
  { set_pwm_freq },
  { set_pwm_range },
  { get_cpu_temp },
  { get_pico_unique_id },
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
#define UNIQUE_ID_REPORT RETRIEVE_PICO_UNIQUE_ID
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
#define FIRMWARE_MINOR 1
#define TRANSPORT_TYPE 1  // 1 = BLE

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

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                       BLE Helper Functions                        */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Write data to BLE (replaces Serial.write)
void ble_write(const byte* data, size_t length) {
  if (deviceConnected && length > 0) {
    // Split into chunks if necessary (BLE has MTU limits)
    const size_t maxChunk = 512;
    for (size_t i = 0; i < length; i += maxChunk) {
      size_t chunkSize = min(maxChunk, length - i);
      txChar.writeValue(data + i, chunkSize);
      delay(1); // Small delay to allow transmission
    }
  }
}

// Check if BLE data is available
bool ble_available() {
  return bleReceiveIndex > 0;
}

// Read one byte from BLE buffer
byte ble_read() {
  if (bleReceiveIndex > 0) {
    byte b = bleReceiveBuffer[0];
    // Shift buffer
    for (int i = 0; i < bleReceiveIndex - 1; i++) {
      bleReceiveBuffer[i] = bleReceiveBuffer[i + 1];
    }
    bleReceiveIndex--;
    return b;
  }
  return 0;
}

// BLE RX characteristic write callback
void onRxCharWritten(BLEDevice central, BLECharacteristic characteristic) {
  size_t len = rxChar.valueLength();
  const uint8_t* data = rxChar.value();

  // Append to receive buffer
  for (size_t i = 0; i < len && bleReceiveIndex < BLE_BUFFER_SIZE; i++) {
    bleReceiveBuffer[bleReceiveIndex++] = data[i];
  }
}

// BLE connection callback
void onConnect(BLEDevice central) {
  deviceConnected = true;
  digitalWrite(LED_BUILTIN, HIGH);
}

// BLE disconnection callback
void onDisconnect(BLEDevice central) {
  deviceConnected = false;
  digitalWrite(LED_BUILTIN, LOW);
  bleReceiveIndex = 0; // Clear buffer on disconnect
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                       Command Functions                          */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void send_debug_info(byte id, int value) {
  byte debug_buffer[5] = { (byte)4, (byte)DEBUG_PRINT, 0, 0, 0 };
  debug_buffer[2] = id;
  debug_buffer[3] = highByte(value);
  debug_buffer[4] = lowByte(value);
  ble_write(debug_buffer, 5);
}

void serial_loopback() {
  byte loop_back_buffer[3] = { 2, (byte)SERIAL_LOOP_BACK, command_buffer[0] };
  ble_write(loop_back_buffer, 3);
}

void set_pin_mode() {
  byte pin = command_buffer[0];
  byte mode = command_buffer[1];

  switch (mode) {
    case INPUT:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      pinMode(pin, INPUT);
      break;
    case INPUT_PULLUP:
      the_digital_pins[pin].pin_mode = mode;
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
  uint16_t cpu_temp_sampling_interval = (command_buffer[4] << 8) + command_buffer[5];
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
  delay(100);
  rebooting = true;
  rp2040.reboot();
}

void get_firmware_version() {
  byte report_message[5] = { 4, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR,
                             TRANSPORT_TYPE };
  ble_write(report_message, 5);
}

/* Servo Commands */
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
    ble_write(report_message, 2);
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

/* i2c functions */
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
    ble_write(report_message, 4);
    return;
  } else if (num_of_bytes > current_i2c_port->available()) {
    byte report_message[4] = { 3, I2C_TOO_FEW_BYTES_RCVD, 1, address };
    ble_write(report_message, 4);
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

  ble_write(i2c_report_message, message_size + 6);
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

/* HC-SR04 */
void sonar_new() {
  sonars[sonars_index].usonic = new NanoConnectHcSr04((uint8_t)command_buffer[0], (uint8_t)command_buffer[1], pio0, 1);
  sonars[sonars_index].trigger_pin = command_buffer[0];
  sonars_index++;
}

/* DHT */
void dht_new() {
  if (dht_index < MAX_DHTS) {
    dhts[dht_index].dht_sensor = new DHTNEW(command_buffer[0]);
    dhts[dht_index].pin = command_buffer[0];
    dhts[dht_index].dht_type = command_buffer[1];
    dht_index++;
  }
}

/* SPI functions */
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
  digitalWrite(chipSelect, LOW);
  current_spi_port->endTransaction();
}

void read_blocking_spi() {
  int chipSelect;
  uint8_t spi_register = command_buffer[1];
  uint8_t number_of_bytes = command_buffer[2];
  uint8_t spi_port = command_buffer[0];

  if (spi_port == 0) {
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
  spi_report_message[2] = spi_port;
  spi_report_message[3] = spi_register;
  spi_report_message[4] = number_of_bytes;

  digitalWrite(chipSelect, LOW);
  current_spi_port->transfer(spi_register | 0x80);

  for (int i = 0; i < number_of_bytes; i++) {
    spi_report_message[i + 5] = current_spi_port->transfer(0x00);
  }
  digitalWrite(chipSelect, LOW);
  current_spi_port->endTransaction();

  ble_write(spi_report_message, number_of_bytes + 6);
}

void set_format_spi() {
}

void spi_cs_control() {
}

/* OneWire placeholders */
void onewire_init() {}
void onewire_reset() {}
void onewire_select() {}
void onewire_skip() {}
void onewire_write() {}
void onewire_read() {}
void onewire_reset_search() {}
void onewire_search() {}
void onewire_crc8() {}

/* Stepper Motor */
void set_pin_mode_stepper() {
  steppers[command_buffer[0]] = new AccelStepper(command_buffer[1], command_buffer[2],
                                                 command_buffer[3], command_buffer[4],
                                                 command_buffer[5], command_buffer[6]);
}

/* Neo Pixel */
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
                       command_buffer[2], command_buffer[3],
                       command_buffer[4]);
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
  ble_write(report_message, 7);
}

void stepper_get_target_position() {
  byte report_message[7] = { 6, STEPPER_TARGET_POSITION, command_buffer[0] };
  long target = steppers[command_buffer[0]]->targetPosition();
  report_message[3] = (byte)((target & 0xFF000000) >> 24);
  report_message[4] = (byte)((target & 0x00FF0000) >> 16);
  report_message[5] = (byte)((target & 0x0000FF00) >> 8);
  report_message[6] = (byte)((target & 0x000000FF));
  ble_write(report_message, 7);
}

void stepper_get_current_position() {
  byte report_message[7] = { 6, STEPPER_CURRENT_POSITION, command_buffer[0] };
  long position = steppers[command_buffer[0]]->currentPosition();
  report_message[3] = (byte)((position & 0xFF000000) >> 24);
  report_message[4] = (byte)((position & 0x00FF0000) >> 16);
  report_message[5] = (byte)((position & 0x0000FF00) >> 8);
  report_message[6] = (byte)((position & 0x000000FF));
  ble_write(report_message, 7);
}

void stepper_set_current_position() {
  long position = (long)(command_buffer[2]) << 24;
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
  ble_write(report_message, 4);
}

void stop_all_reports() {
  stop_reports = true;
  delay(20);
}

void enable_all_reports() {
  stop_reports = false;
  delay(20);
}

void get_next_command() {
  byte command;
  byte packet_length;
  command_descriptor command_entry;

  memset(command_buffer, 0, sizeof(command_buffer));

  if (!ble_available()) {
    return;
  }

  packet_length = (byte)ble_read();

  while (!ble_available()) {
    delay(1);
  }

  command = (byte)ble_read();
  command_entry = command_table[command];

  if (packet_length > 1) {
    for (int i = 0; i < packet_length - 1; i++) {
      while (!ble_available()) {
        delay(1);
      }
      command_buffer[i] = (byte)ble_read();
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

/* Scanning and Reporting */
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
          ble_write(report_message, 4);
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
            ble_write(report_message, 5);
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
        ble_write(report_message, 6);
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
        byte report_message[5] = { 4, SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin,
                                   integ, frac };
        ble_write(report_message, 5);
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
  float humidity, temperature;

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
          ble_write(report_message, 10);
          return;
        } else {
          float j, f;
          float humidity = dhts[i].dht_sensor->getHumidity();
          if (humidity >= 0.0) {
            report_message[4] = 0;
          } else {
            report_message[4] = 1;
          }
          f = modff(humidity, &j);
          report_message[6] = (uint8_t)j;
          report_message[7] = (uint8_t)(f * 100);

          float temperature = dhts[i].dht_sensor->getTemperature();
          if (temperature >= 0.0) {
            report_message[5] = 0;
          } else {
            report_message[5] = 1;
          }

          f = modff(temperature, &j);
          report_message[8] = (uint8_t)j;
          report_message[9] = (uint8_t)(f * 100);
          ble_write(report_message, 10);
        }
      }
    }
  }
}

void get_pico_unique_id() {
  byte unique_id_report_report_message[10] = { 9, UNIQUE_ID_REPORT,
                                               0, 0, 0, 0, 0, 0, 0, 0 };
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

  ble_write(unique_id_report_report_message, 10);
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
            ble_write(report_message, 3);
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
            ble_write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }
          break;
        default:
          break;
      }
    }
  }
}

/* Setup And Loop */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize BLE
  if (!BLE.begin()) {
    // BLE initialization failed - blink LED rapidly
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  // Set device name and local name
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_DEVICE_NAME);

  // Set up Nordic UART Service
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(txChar);
  uartService.addCharacteristic(rxChar);
  BLE.addService(uartService);

  // Set event handlers
  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  rxChar.setEventHandler(BLEWritten, onRxCharWritten);

  // Start advertising
  BLE.advertise();

  // LED blink sequence to show ready
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }

  for (int i = 0; i < MAX_NUMBER_OF_STEPPERS; i++) {
    stepper_run_modes[i] = STEPPER_STOP;
  }

  analogWriteRange(20000);
  init_pin_structures();
}

void loop() {
  // Poll for BLE events
  BLE.poll();

  // Handle connection state changes
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  if (!deviceConnected && oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    bleReceiveIndex = 0; // Clear buffer on disconnect
    // Restart advertising
    BLE.advertise();
  }

  if (!rebooting) {
    get_next_command();

    if (!stop_reports && deviceConnected) {
      scan_digital_inputs();
      scan_analog_inputs();
      scan_sonars();
      scan_dhts();
      scan_cpu_temp();
      run_steppers();
    }
  }
}