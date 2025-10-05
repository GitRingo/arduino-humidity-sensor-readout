#include <Wire.h>

// CSV Ausgabe Setting
#define DE_EXCEL 1        // 1 = Dezimalkomma (für Excel DE), 0 = Standard (Punkt)
#define SEP '\t'          // Tabulator 

// SHT31 SETUP
const uint8_t SHT_ADDR = 0x44;

// Funktionen
bool startMeasurement() {
  Wire.beginTransmission(SHT_ADDR);
  Wire.write(0x24);
  Wire.write(0x00);  
  return (Wire.endTransmission() == 0);
}

bool readMeasurement(float &temp, float &hum) {
  if (Wire.requestFrom(SHT_ADDR, (uint8_t)6) != 6) return false;

  uint8_t tH = Wire.read(), tL = Wire.read(), tC = Wire.read();
  uint8_t hH = Wire.read(), hL = Wire.read(), hC = Wire.read();

  // Redundanzen prüfen
  if (((tH + tL) & 0xFF) == tC && ((hH + hL) & 0xFF) == hC) {
    uint16_t tRaw = ((uint16_t)tH << 8) | tL;
    uint16_t hRaw = ((uint16_t)hH << 8) | hL;

    temp = -45.0 + 175.0 * ((float)tRaw / 65535.0);
    hum  = 100.0 * ((float)hRaw / 65535.0);
    return true;
  }

  return false;
}

// Zahl füe Excel formatieren
void printCsvFloat(float v, uint8_t prec) {
  char buf[16];
  dtostrf(v, 0, prec, buf);
#if DE_EXCEL
  for (char *p = buf; *p; p++) if (*p == '.') *p = ',';
#endif
  Serial.print(buf);
}

// Hauptprogramm
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // CSV-Header
  Serial.print("t_s");     Serial.print(SEP);
  Serial.print("temp_c");  Serial.print(SEP);
  Serial.println("rh_pct");
}

void loop() {
  static unsigned long lastMeasure = 0;
  unsigned long now = millis();

  if (now - lastMeasure >= 1000) {
    float T, RH;
    if (startMeasurement()) {
      delay(20); // kurzes Delay.
      if (readMeasurement(T, RH)) {
        float t_s = now / 1000.0;
        printCsvFloat(t_s, 3); Serial.print(SEP);
        printCsvFloat(T, 2);   Serial.print(SEP);
        printCsvFloat(RH, 1);  Serial.println();
      }
    }
    lastMeasure = now;
  }
}
