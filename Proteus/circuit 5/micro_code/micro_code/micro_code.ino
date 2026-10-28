#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

String line = "";

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);
  lcd.print("Waiting GPS...");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (line.startsWith("$GPGGA")) {
        String f[15];
        int idx = 0, start = 0;
        for (int i = 0; i < line.length() && idx < 15; i++) {
          if (line[i] == ',') {
            f[idx++] = line.substring(start, i);
            start = i + 1;
          }
        }

        if (f[6] == "0") {                    // fix quality field
          lcd.clear();
          lcd.print("NO FIX");
        } else {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("La:"); lcd.print(f[2]); lcd.print(f[3]);
          lcd.setCursor(0, 1);
          lcd.print("Lo:"); lcd.print(f[4]); lcd.print(f[5]);
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
}
