#include <WiFi.h>

// Точка доступа БЕЗ ПАРОЛЯ (открытая).
// Проверяем гипотезу: всё, где участвует WPA2, не работает, а открытое - работает.

const char *AP_SSID = "Car";
const int   AP_CHAN = 1;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== open AP test ===");
  Serial.printf("reset reason: %d\n", (int)esp_reset_reason());

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(200);

  // NULL вместо пароля = открытая сеть
  bool ok = WiFi.softAP(AP_SSID, NULL, AP_CHAN, 0, 4);
  Serial.printf("softAP(\"%s\", OTKRYTAYA, ch %d): %s\n",
                AP_SSID, AP_CHAN, ok ? "OK" : "FAIL");

  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  Serial.print("AP IP:    ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("TX power: %d (78 = max)\n", (int)WiFi.getTxPower());
  Serial.printf("mode:     %d (2 = WIFI_AP)\n", (int)WiFi.getMode());
  Serial.println();
  Serial.println("Ishchi set \"Car\" BEZ PAROLYA v WiFiman.");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Serial.printf("[%6lu s] clients: %u | heap: %u\n",
                  millis() / 1000, WiFi.softAPgetStationNum(), ESP.getFreeHeap());
  }
  delay(20);
}
