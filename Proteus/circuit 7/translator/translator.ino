#include <LiquidCrystal.h>

// This still matches the PB and PD ports you just wired!
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  Serial.begin(115200);
  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Proteus Ready");
}

void loop() {
  if (Serial.available()) {
    String incomingData = Serial.readStringUntil('\n');
    incomingData.trim(); 
    
    if (incomingData.length() > 0) {
      lcd.setCursor(0, 0);
      lcd.print("Proteus HIL     "); 
      lcd.setCursor(0, 1);
      lcd.print(incomingData + " cm          ");
    }
  }
}
