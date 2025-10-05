#include <Wire.h>

// ========= CSV/Locale =========
#define CSV_SEMICOLON_DE 1   // 1 = Semikolon + Dezimalkomma (DE-Excel), 0 = Komma + Dezimalpunkt
#if CSV_SEMICOLON_DE
  #define SEP '\t' 
#else
  #define SEP '\t' 
#endif

// ========= SHT31 =========
const uint8_t SHT_ADDRS[2] = {0x44, 0x45};
uint8_t SHT = 0x44;

uint8_t crc8(const uint8_t* d, int n) {
  uint8_t c=0xFF;
  for (int i=0;i<n;i++){ c^=d[i]; for(uint8_t b=0;b<8;b++) c=(c&0x80)?(uint8_t)((c<<1)^0x31):(uint8_t)(c<<1); }
  return c;
}

bool sht_startMeasure() {
  Wire.beginTransmission(SHT);
  Wire.write(0x24); Wire.write(0x00); // single-shot, high rep., no clock stretch
  return Wire.endTransmission()==0;
}

bool sht_fetch(float &T, float &RH) {
  if (Wire.requestFrom((int)SHT, 6) != 6) return false;
  uint8_t tH=Wire.read(), tL=Wire.read(), tC=Wire.read();
  uint8_t hH=Wire.read(), hL=Wire.read(), hC=Wire.read();
  uint8_t tb[2]={tH,tL}, hb[2]={hH,hL};
  if (crc8(tb,2)!=tC || crc8(hb,2)!=hC) return false;
  uint16_t tRaw=((uint16_t)tH<<8)|tL, hRaw=((uint16_t)hH<<8)|hL;
  T  = -45.0f + 175.0f * (float)tRaw / 65535.0f;
  RH = 100.0f * (float)hRaw / 65535.0f;
  return true;
}

// Zahl hübsch & Excel-freundlich ausgeben
void printCsvFloat(float v, uint8_t prec) {
  char buf[24];
  dtostrf(v, 0, prec, buf);       // keine feste Breite, nur Nachkommastellen
#if CSV_SEMICOLON_DE
  // Dezimalpunkt -> Komma (für DE-Excel)
  for (char* p = buf; *p; ++p) if (*p == '.') *p = ',';
#endif
  Serial.print(buf);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(100000);

  // SHT31-Adresse via Soft-Reset finden
  int8_t idx = -1;
  for (uint8_t i=0;i<2;i++){
    Wire.beginTransmission(SHT_ADDRS[i]);
    Wire.write(0x30); Wire.write(0xA2); // soft reset
    if (Wire.endTransmission()==0){ idx=i; break; }
  }
  if (idx>=0) SHT = SHT_ADDRS[idx];

  // CSV-Header
  Serial.print(F("t_s")); Serial.print(SEP);
  Serial.print(F("temp_c")); Serial.print(SEP);
  Serial.println(F("rh_pct"));
}

void loop() {
  static unsigned long last = 0;
  static bool waiting = false;
  static unsigned long t_cmd = 0;

  unsigned long now = millis();

  // alle 1000 ms neue Messung starten
  if (!waiting && (now - last) >= 1000UL) {
    if (sht_startMeasure()) {
      t_cmd = now; waiting = true;
    }
    last = now;
  }

  // nach >=15 ms Ergebnis holen und CSV-Zeile drucken
  if (waiting && (now - t_cmd) >= 15UL) {
    float T, RH;
    if (sht_fetch(T, RH)) {
      // Zeit (Sekunden seit Start)
      float tsec = now / 1000.0f;
      printCsvFloat(tsec, 3);   Serial.print(SEP);
      printCsvFloat(T, 2);      Serial.print(SEP);
      printCsvFloat(RH, 1);     Serial.println();
    }
    // bei Fehlversuch einfach nächste Sekunde neu versuchen (keine kaputten CSV-Zeilen)
    waiting = false;
  }
}
