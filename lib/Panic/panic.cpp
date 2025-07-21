#include "panic.h"
#include <Arduino.h>

void panic2(const char message[]) {
  while (true) {
    Serial.print("ERROR\t\t");
    Serial.println(message);
    delay(50);
  }
}
