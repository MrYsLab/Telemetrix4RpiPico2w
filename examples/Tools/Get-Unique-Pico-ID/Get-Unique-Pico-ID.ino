#include <Arduino.h>
#include <pico/unique_id.h> // Include the Pico SDK unique ID header
#include "Telemetrix4RpiPico2w.h"


void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);
  delay(3000);

  Serial.print("Unique Board ID (bytes): [");
  for (int i = 0; i < 7; i++) {
    Serial.print(board_id.id[i]);
    Serial.print(", ");
  }
  Serial.print(board_id.id[7]);
  Serial.println("]");

}

void loop() {
  // Your main code here
}