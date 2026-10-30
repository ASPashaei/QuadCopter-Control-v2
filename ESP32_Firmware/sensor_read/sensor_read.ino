// ESP32 IR Sensor Distance Calculator
#include <LiquidCrystal.h>
#include "BluetoothSerial.h"
const int analogPin = 34;      
const int resolution = 4095;   
const float refVoltage = 3.3;  
const int rs = 13, en = 14, d4 = 25, d5 = 26, d6 = 27, d7 = 33;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

BluetoothSerial SerialBT;

void setup() {
  
  Serial.begin(115200); 
  // start bluetooth
  SerialBT.begin("Drone_Telemetry");
  
  
  analogReadResolution(12);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("System Booting..");
  delay(1000);
}

void loop() {
  //  Read the raw digital value from the pin
  int rawADC = analogRead(analogPin);
  
  // Prevent division by zero or errors if the sensor is completely covered
  if(rawADC == 0) {
    rawADC = 1; 
  }

  // Convert the raw digital value back into the voltage measured at the pin
  float measuredVoltage = (rawADC / (float)resolution) * refVoltage;
  
  // Reverse the 2/3 voltage divider effect by multiplying by 1.5
  float trueSensorVoltage = measuredVoltage * 1.5;
  

  // Formula: d = 27.86 * V^(-1.15) - 0.42
  float distance = 27.86 * pow(trueSensorVoltage, -1.15) - 0.42;
  
  lcd.setCursor(0, 0);       
  lcd.print("Altitude:       "); 
  lcd.setCursor(0, 1);       
  lcd.print(distance);   
  lcd.print(" cm       ");

  // Send the final distance value over the USB cable
  Serial.println(distance);
  SerialBT.println(distance);
  
  // Wait 20 milliseconds (runs the loop at 50Hz)
  delay(20); 
}
