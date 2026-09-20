/*
 * 01_ap_test - диагностика №2: заставить робота РАЗДАВАТЬ Wi-Fi.
 *
 * Это минимальный скетч без камеры и без моторов.
 * Задача: проверить, что плата жива, загружается и поднимает точку доступа
 * на 192.168.4.1. Если этот скетч работает - железо и прошивка в порядке,
 * и можно переходить к 03_car_ap_camera.
 *
 * Плата в Arduino IDE: "AI Thinker ESP32-CAM"
 * Partition Scheme:    "Huge APP (3MB No OTA/1MB SPIFFS)"
 *
 * После заливки: снять перемычку GPIO0 -> GND и нажать RESET.
 * Ищем сеть "Car", пароль "test1234", открываем http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>

const char *AP_SSID = "Car";
const char *AP_PASS = "test1234";   // минимум 8 символов, иначе softAP не поднимется

WebServer server(80);

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>AP test</title></head><body style='font-family:sans-serif'>"
                "<h2>ESP32-CAM: точка доступа работает</h2>"
                "<p>Uptime: " + String(millis() / 1000) + " c</p>"
                "<p>Клиентов: " + String(WiFi.softAPGetStationNum()) + "</p>"
                "<p>Free heap: " + String(ESP.getFreeHeap()) + "</p>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== AP test ===");
  Serial.printf("reset reason: %d\n", (int)esp_reset_reason());

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);

  // softAP(ssid, pass, channel, hidden, max_connections)
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  Serial.printf("softAP(\"%s\"): %s\n", AP_SSID, ok ? "OK" : "FAIL");

  if (!ok) {
    Serial.println("Точка доступа НЕ поднялась. Проверь: пароль >= 8 символов, питание платы.");
    return;
  }

  Serial.print("AP IP:  ");
  Serial.println(WiFi.softAPIP());   // должно быть 192.168.4.1
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());

  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP сервер запущен на порту 80");
}

void loop() {
  server.handleClient();

  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Serial.printf("[%6lu s] клиентов: %u | heap: %u\n",
                  millis() / 1000,
                  WiFi.softAPGetStationNum(),
                  ESP.getFreeHeap());
  }
}
