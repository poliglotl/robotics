#include <WiFi.h>

// Чистый тест точки доступа: без сканирования, без переключения режимов,
// без камеры и моторов. Только softAP и ничего больше.

const char *AP_SSID = "Car";
const char *AP_PASS = "test1234";
const int   AP_CHAN = 1;        // при неудаче попробовать 6, потом 11

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== pure AP test ===");
  Serial.printf("reset reason: %d\n", (int)esp_reset_reason());

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(200);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHAN, 0, 4);
  Serial.printf("softAP(\"%s\", ch %d): %s\n", AP_SSID, AP_CHAN, ok ? "OK" : "FAIL");

  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  Serial.print("AP IP:    ");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP MAC:   ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.printf("TX power: %d (78 = max)\n", (int)WiFi.getTxPower());
  Serial.printf("mode:     %d (2 = WIFI_AP)\n", (int)WiFi.getMode());
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
