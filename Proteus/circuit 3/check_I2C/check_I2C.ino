#include <Wire.h>

void setup() {
  Wire.begin();
  Wire.setClock(400000);        
  Serial.begin(9600);
  delay(500);
  Serial.println("Scanning I2C bus...");

  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  Serial.print("Total devices: ");
  Serial.println(found);
}

void loop() {}
