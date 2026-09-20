/*
 * 02_sta_test - исправленная версия твоего исходного скетча.
 *
 * ЭТО РЕЖИМ КЛИЕНТА (station): плата ПОДКЛЮЧАЕТСЯ к чужому роутеру.
 * Он НЕ раздаёт Wi-Fi и на 192.168.4.1 в этом режиме ничего не будет.
 *
 * Нужен только если хочешь, чтобы робот жил в твоей домашней сети
 * (тогда IP выдаёт роутер, и он будет вида 192.168.0.x / 192.168.1.x).
 *
 * ВАЖНО: в ssid/password нужно вписать СВОЙ РОУТЕР, а не SSID самого робота.
 * Плата не может подключиться к сети, которую раздаёт она же.
 */

#include <WiFi.h>

// <<< сюда креды ДОМАШНЕГО РОУТЕРА, не робота >>>
const char *ssid     = "MyHomeWiFi";
const char *password = "my-home-password";

// ESP32 умеет только 2.4 ГГц. Сеть 5 ГГц он не увидит вообще.

const char *statusText(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "0 WL_IDLE_STATUS (ничего не делает)";
    case WL_NO_SSID_AVAIL:   return "1 WL_NO_SSID_AVAIL (сеть не найдена: опечатка в SSID / 5 ГГц / далеко)";
    case WL_SCAN_COMPLETED:  return "2 WL_SCAN_COMPLETED";
    case WL_CONNECTED:       return "3 WL_CONNECTED";
    case WL_CONNECT_FAILED:  return "4 WL_CONNECT_FAILED (чаще всего неверный пароль)";
    case WL_CONNECTION_LOST: return "5 WL_CONNECTION_LOST";
    case WL_DISCONNECTED:    return "6 WL_DISCONNECTED (роутер отбил или ещё не начал)";
    default:                 return "неизвестный статус";
  }
}

bool connectWiFi(uint32_t timeoutMs) {
  WiFi.persistent(false);      // не сохранять креды во flash (иначе ловишь чужие старые)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);       // сбросить сохранённое состояние
  delay(200);
  WiFi.setSleep(false);        // без энергосбережения - стабильнее для стрима/управления

  Serial.printf("подключаюсь к \"%s\" ...\n", ssid);
  WiFi.begin(ssid, password);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== station test ===");

  // Перед подключением - посмотреть, видна ли вообще нужная сеть.
  int n = WiFi.scanNetworks();
  Serial.printf("видно сетей: %d\n", n);
  bool found = false;
  for (int i = 0; i < n; i++) {
    Serial.printf("  %-32s RSSI %d\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    if (WiFi.SSID(i) == ssid) found = true;
  }
  WiFi.scanDelete();
  Serial.printf("сеть \"%s\" в эфире: %s\n", ssid, found ? "ДА" : "НЕТ");

  if (connectWiFi(20000)) {
    Serial.println("CONNECTED");
    Serial.print("IP:      "); Serial.println(WiFi.localIP());
    Serial.print("gateway: "); Serial.println(WiFi.gatewayIP());
    Serial.printf("RSSI:    %d dBm\n", WiFi.RSSI());
    Serial.println(">> управление будет по IP выше, НЕ по 192.168.4.1");
  } else {
    Serial.printf("FAILED, status = %s\n", statusText(WiFi.status()));
  }
}

void loop() {
  // В отличие от исходника - переподключаемся, а не висим мёртвыми.
  static uint32_t last = 0;
  if (millis() - last > 10000) {
    last = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("связь потеряна, пробую снова");
      connectWiFi(15000);
    } else {
      Serial.printf("online, IP %s, RSSI %d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
  }
}
