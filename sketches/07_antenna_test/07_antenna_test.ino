/*
 * 07_antenna_test - проверка, доходит ли сигнал до антенны.
 *
 * Число найденных сетей скачет от скана к скану, поэтому по нему сравнивать
 * нельзя. Скетч делает 6 сканирований подряд и усредняет ДВА числа:
 * количество сетей и уровень самой сильной из них. Усреднённый уровень
 * сильнейшей сети стабилен и годится для сравнения.
 *
 * КАК ПОЛЬЗОВАТЬСЯ:
 *   1. Залить, дождаться строки SREDNEE, записать оба числа.
 *   2. Выключить питание, аккуратно снять коаксиал с разъёма IPEX.
 *   3. Включить, дождаться SREDNEE, записать числа ещё раз.
 *   4. Сравнить "luchshiy RSSI".
 *
 *   Упал на 10 dBm и больше -> антенна в тракте, работает.
 *   Почти не изменился      -> антенна к радио не подключена.
 */

#include <WiFi.h>

#define SCANS 6

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== antenna test ===");

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);
}

void loop() {
  Serial.printf("delayu %d skanirovaniy, eto okolo minuty...\n", SCANS);

  long sumBest = 0, sumCnt = 0;
  int  done = 0;

  for (int k = 0; k < SCANS; k++) {
    int n = WiFi.scanNetworks();
    int best = -127;
    for (int i = 0; i < n; i++) {
      if (WiFi.RSSI(i) > best) best = WiFi.RSSI(i);
    }
    WiFi.scanDelete();

    Serial.printf("  skan %d: setey %3d | luchshiy RSSI %4d dBm\n", k + 1, n, best);
    if (n > 0) { sumBest += best; sumCnt += n; done++; }
  }

  Serial.println("=========================================");
  if (done) {
    Serial.printf("SREDNEE: setey %ld | luchshiy RSSI %ld dBm\n",
                  sumCnt / done, sumBest / done);
  } else {
    Serial.println("SREDNEE: setey 0 - ne nayti nichego");
  }
  Serial.println("=========================================");
  Serial.println("zapishi eti chisla i povtori bez antenny");
  Serial.println();

  delay(5000);
}
