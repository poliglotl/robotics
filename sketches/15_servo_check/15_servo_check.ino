/*
 * 15_servo_check - проверка ОДНОГО серво через Serial Monitor.
 *
 * Зачем: если серво воет и не двигается, надо разделить причины -
 * питание, земля, сигнал или механический упор. Этот скетч крутит
 * ровно одно серво, чтобы ток тянуло только оно.
 *
 * ПОДКЛЮЧЕНИЕ (одно серво, второе отключить совсем)
 *   коричневый / чёрный  -> GND (общая земля с платой, обязательно)
 *   красный              -> 5 В (выход +5V на плате драйвера, НЕ 3.3V)
 *   оранжевый / жёлтый   -> GPIO 2
 *
 * ПЕРЕД ПЕРВЫМ ЗАПУСКОМ снять качалку (белый рычаг) с вала серво,
 * чтобы она ни во что не упиралась.
 *
 * РАБОТА
 *   Serial Monitor, 115200, строка "Newline".
 *   Печатаешь число 0..180 и Enter - серво едет на этот угол.
 *   "s" + Enter - снять сигнал, серво расслабляется (можно крутить рукой).
 *   "c" + Enter - в центр (90).
 *
 * КАК ИСКАТЬ ПРЕДЕЛЫ
 *   Начать с 90. Потом 100, 110, 120... шагами по 10.
 *   Как только серво уперлось и завыло - сразу "s", и предел = предыдущее
 *   число минус 5. То же в другую сторону: 80, 70, 60...
 *   Найденные числа вписать в 14_show_routine (PAN_MIN/PAN_MAX и т.д.).
 */

#define SERVO_PIN  2
#define SERVO_CH   6
#define SERVO_FREQ 50
#define SERVO_RES  16

// Импульс для 0 и 180 градусов. Начинаем с безопасного 1000-2000.
#define PULSE_MIN_US 1000
#define PULSE_MAX_US 2000

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch, freq, res)  ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, ch, duty)       ledcWrite(pin, duty)
#else
  #define PWM_SETUP(pin, ch, freq, res)  do { ledcSetup(ch, freq, res); \
                                              ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, duty)       ledcWrite(ch, duty)
#endif

static uint32_t angleToDuty(int deg) {
  if (deg < 0)   deg = 0;
  if (deg > 180) deg = 180;
  uint32_t us = PULSE_MIN_US + (uint32_t)deg * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  return (us * 65536UL) / 20000UL;
}

static void servoTo(int deg) {
  PWM_WRITE(SERVO_PIN, SERVO_CH, angleToDuty(deg));
}

static void servoRelax() {
  PWM_WRITE(SERVO_PIN, SERVO_CH, 0);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== 15_servo_check ===");
  Serial.printf("pin GPIO %d, %d Hz, impulse %d..%d us\n",
                SERVO_PIN, SERVO_FREQ, PULSE_MIN_US, PULSE_MAX_US);
  Serial.println("vvedi chislo 0..180, ili s = relax, c = center");

  PWM_SETUP(SERVO_PIN, SERVO_CH, SERVO_FREQ, SERVO_RES);
  servoTo(90);
  Serial.println("start: 90 grad");
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "s" || line == "S") {
    servoRelax();
    Serial.println("relax: signal snyat");
    return;
  }
  if (line == "c" || line == "C") {
    servoTo(90);
    Serial.println("center: 90 grad");
    return;
  }

  int deg = line.toInt();
  if (deg < 0 || deg > 180) {
    Serial.printf("ne ponyal '%s'. Nuzhno 0..180, ili s, ili c\n", line.c_str());
    return;
  }
  servoTo(deg);
  Serial.printf("ugol: %d grad, impulse %lu us\n",
                deg, (unsigned long)(PULSE_MIN_US + (uint32_t)deg *
                (PULSE_MAX_US - PULSE_MIN_US) / 180));
}
