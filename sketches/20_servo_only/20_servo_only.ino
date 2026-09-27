/*
 * 20_servo_only - серво и больше НИЧЕГО. Ни камеры, ни Wi-Fi, ни моторов.
 *
 * Зачем: в машине серво не двигаются, и надо понять, дело в самих серво
 * (питание, провода, механика) или в том, что им мешает камера и Wi-Fi.
 * Пины и каналы здесь те же, что в 18_car_control, так что сравнение честное.
 *
 * ПОДКЛЮЧЕНИЕ
 *   серво 1 (поворот): сигнал -> GPIO 2
 *   серво 2 (наклон):  сигнал -> GPIO 3
 *   у обоих: красный -> +5V платы драйвера, коричневый -> GND
 *   НЕ на 3V3 и НЕ на пин 5V самой ESP32-CAM.
 *
 * ЧТО ДЕЛАЕТ
 *   Печатает, какую частоту дал LEDC, и гоняет оба серво туда-сюда
 *   по кругу: центр -> влево -> центр -> вправо. Каждый шаг подписан.
 *
 * ЧИТАТЬ ТАК
 *   "-> 50 Hz" и серво двигается  = всё в порядке, дело в машине
 *   "-> 50 Hz" и серво НЕ двигается = сигнал есть, виновато питание,
 *                                     земля, провод или механика
 *   "-> 0 Hz"                       = LEDC отказал, виноват код
 */

#define PIN_PAN   2
#define PIN_TILT  3
#define CH_PAN    14
#define CH_TILT   15

#define PULSE_MIN_US 1000
#define PULSE_MAX_US 2000

// Узкие пределы: серво не упрётся в стопор, даже если оно клон SG90.
#define ANG_MIN     60
#define ANG_CENTER  90
#define ANG_MAX    120
#define STEP_MS     20       // один градус за столько мс

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static uint32_t pwmSetup(int pin, int ch, uint32_t f, uint8_t r) {
    (void)ch; return ledcAttach(pin, f, r) ? f : 0;
  }
  #define PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
  static uint32_t pwmSetup(int pin, int ch, uint32_t f, uint8_t r) {
    uint32_t got = (uint32_t)ledcSetup(ch, f, r);
    if (got) ledcAttachPin(pin, ch);
    return got;
  }
  #define PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

static uint32_t angleToDuty(int deg) {
  if (deg < 0)   deg = 0;
  if (deg > 180) deg = 180;
  uint32_t us = PULSE_MIN_US + (uint32_t)deg * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  return (us * 65536UL) / 20000UL;
}

// Плавно ведёт одно серво к углу и печатает, куда именно.
static void sweep(const char *name, int pin, int ch, int from, int to) {
  Serial.printf("  %s: %d -> %d grad\n", name, from, to);
  int step = (to > from) ? 1 : -1;
  for (int a = from; a != to; a += step) {
    PWM_WRITE(pin, ch, angleToDuty(a));
    delay(STEP_MS);
  }
  PWM_WRITE(pin, ch, angleToDuty(to));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== 20_servo_only ===");

  uint32_t fp = pwmSetup(PIN_PAN,  CH_PAN,  50, 16);
  uint32_t ft = pwmSetup(PIN_TILT, CH_TILT, 50, 16);

  Serial.printf("pan  GPIO%-2d kanal %d -> %lu Hz\n",
                PIN_PAN, CH_PAN, (unsigned long)fp);
  Serial.printf("tilt GPIO%-2d kanal %d -> %lu Hz\n",
                PIN_TILT, CH_TILT, (unsigned long)ft);
  Serial.printf("duty: %d grad = %lu, %d grad = %lu, %d grad = %lu (iz 65535)\n",
                ANG_MIN,    (unsigned long)angleToDuty(ANG_MIN),
                ANG_CENTER, (unsigned long)angleToDuty(ANG_CENTER),
                ANG_MAX,    (unsigned long)angleToDuty(ANG_MAX));

  if (!fp || !ft) {
    Serial.println();
    Serial.println("LEDC OTKAZAL: signala na pinah net, delo v kode.");
    Serial.println("Pokazhi etu stroku - ya pomenyayu razreshenie.");
    return;
  }

  Serial.println("LEDC dal 50 Hz. Signal na pinah EST.");
  Serial.println("Dalshe smotri glazami: dvigayutsya li serva.");
  Serial.println();

  PWM_WRITE(PIN_PAN,  CH_PAN,  angleToDuty(ANG_CENTER));
  PWM_WRITE(PIN_TILT, CH_TILT, angleToDuty(ANG_CENTER));
  delay(1000);
}

void loop() {
  static int cycle = 0;
  Serial.printf("krug %d\n", ++cycle);

  // По одному серво за раз: так видно, какое из двух шевелится, и ток
  // тянет только одно - если их два на слабом регуляторе, это важно.
  sweep("pan ", PIN_PAN, CH_PAN, ANG_CENTER, ANG_MIN);
  delay(400);
  sweep("pan ", PIN_PAN, CH_PAN, ANG_MIN, ANG_MAX);
  delay(400);
  sweep("pan ", PIN_PAN, CH_PAN, ANG_MAX, ANG_CENTER);
  delay(800);

  sweep("tilt", PIN_TILT, CH_TILT, ANG_CENTER, ANG_MIN);
  delay(400);
  sweep("tilt", PIN_TILT, CH_TILT, ANG_MIN, ANG_MAX);
  delay(400);
  sweep("tilt", PIN_TILT, CH_TILT, ANG_MAX, ANG_CENTER);
  delay(1500);
}
