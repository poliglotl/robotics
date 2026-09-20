/*
 * 00_wifi_scan - диагностика №1: "а есть ли вообще сеть?"
 *
 * Заливается на ЛЮБОЙ ESP32 (можно на вторую плату, не на робота).
 * Показывает все Wi-Fi сети вокруг. Нужен, чтобы ответить на вопрос:
 * робот реально раздаёт "Car" или нет.
 *
 * ВАЖНО: если запустить это на самом роботе - он перестанет раздавать Wi-Fi,
 * потому что прошивка робота будет затёрта этим скетчем.
 */

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== wifi scan ===");

  WiFi.persistent(false);   // не писать креды в NVS
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);    // сбросить остатки прошлых подключений
  delay(200);
}

void loop() {
  Serial.println("scanning...");
  int n = WiFi.scanNetworks();

  if (n <= 0) {
    Serial.println("сетей не найдено");
  } else {
    Serial.printf("найдено сетей: %d\n", n);
    for (int i = 0; i < n; i++) {
      Serial.printf("%2d | %-32s | RSSI %4d dBm | ch %2d | %s\n",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i),
                    WiFi.channel(i),
                    WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN" : "SECURED");
    }
  }
  WiFi.scanDelete();
  Serial.println();
  delay(5000);
}
