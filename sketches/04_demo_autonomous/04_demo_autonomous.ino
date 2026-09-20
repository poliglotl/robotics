/*
 * 04_demo_autonomous - ПЛАН Б для показа в школе.
 *
 * Wi-Fi не используется вообще. Робот просто едет по программе.
 * Нужен на случай, если в классе не получится подключиться (забитый эфир,
 * телефон отвалился, села батарейка телефона) - чтобы было что показать.
 *
 * Логика:
 *   1. После включения мигает фарой 5 секунд - есть время поставить робота
 *      на пол и отойти.
 *   2. Едет "квадратом": вперёд - поворот - вперёд - поворот ... 4 раза.
 *   3. Останавливается и мигает фарой. Для нового круга - нажать RESET.
 *
 * ВАЖНО: это езда БЕЗ датчиков. Робот не видит препятствий и уедет в стену,
 * если места мало. Нужна свободная площадка примерно 2x2 метра.
 *
 * Пины моторов должны быть ТЕ ЖЕ, что в 03_car_ap_camera.
 */

// --------------------------------------------------------------- пины моторов
#define PIN_L_IN1   13
#define PIN_L_IN2   12
#define PIN_R_IN1   15
#define PIN_R_IN2   14
#define PIN_FLASH_LED 4

// ------------------------------------------------------- параметры «программы»
#define START_DELAY_MS   5000   // время, чтобы поставить робота и отойти
#define DRIVE_SPEED       200   // 0..255, скорость по прямой
#define TURN_SPEED        170   // скорость на повороте
#define DRIVE_MS         1200   // сколько едет по прямой
#define TURN_MS           600   // длительность поворота (подберите под ~90 град.)
#define PAUSE_MS          400   // пауза между манёврами
#define LAPS                4   // сколько сторон «квадрата» проехать

// ----------------------------------------------------------------- ШИМ (LEDC)
#define PWM_FREQ  1000
#define PWM_RES   8
#define CH_L_IN1  2
#define CH_L_IN2  3
#define CH_R_IN1  4
#define CH_R_IN2  5

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch)        ledcAttach((pin), PWM_FREQ, PWM_RES)
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((pin), (duty))
#else
  #define PWM_SETUP(pin, ch)        do { ledcSetup((ch), PWM_FREQ, PWM_RES); \
                                         ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((ch), (duty))
#endif

// ------------------------------------------------------------------ моторы
static void sideDrive(int pinA, int chA, int pinB, int chB, int v) {
  if (v > 0) {
    PWM_WRITE(pinA, chA, v);
    PWM_WRITE(pinB, chB, 0);
  } else if (v < 0) {
    PWM_WRITE(pinA, chA, 0);
    PWM_WRITE(pinB, chB, -v);
  } else {
    PWM_WRITE(pinA, chA, 0);
    PWM_WRITE(pinB, chB, 0);
  }
}

static void drive(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  sideDrive(PIN_L_IN1, CH_L_IN1, PIN_L_IN2, CH_L_IN2, left);
  sideDrive(PIN_R_IN1, CH_R_IN1, PIN_R_IN2, CH_R_IN2, right);
}

static void blink(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_FLASH_LED, HIGH); delay(ms);
    digitalWrite(PIN_FLASH_LED, LOW);  delay(ms);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== demo autonomous ===");

  pinMode(PIN_FLASH_LED, OUTPUT);
  digitalWrite(PIN_FLASH_LED, LOW);

  PWM_SETUP(PIN_L_IN1, CH_L_IN1);
  PWM_SETUP(PIN_L_IN2, CH_L_IN2);
  PWM_SETUP(PIN_R_IN1, CH_R_IN1);
  PWM_SETUP(PIN_R_IN2, CH_R_IN2);
  drive(0, 0);

  Serial.printf("старт через %d мс - поставьте робота на пол\n", START_DELAY_MS);
  blink(START_DELAY_MS / 400, 200);   // мигаем всё время ожидания

  for (int lap = 1; lap <= LAPS; lap++) {
    Serial.printf("сторона %d/%d: вперёд\n", lap, LAPS);
    drive(DRIVE_SPEED, DRIVE_SPEED);
    delay(DRIVE_MS);

    drive(0, 0);
    delay(PAUSE_MS);

    Serial.printf("сторона %d/%d: поворот\n", lap, LAPS);
    drive(TURN_SPEED, -TURN_SPEED);   // разворот на месте вправо
    delay(TURN_MS);

    drive(0, 0);
    delay(PAUSE_MS);
  }

  drive(0, 0);
  Serial.println("программа закончена. RESET для повтора.");
  blink(3, 300);
}

void loop() {
  // Страховка: моторы гарантированно выключены, пока плата жива.
  drive(0, 0);
  delay(500);
}
