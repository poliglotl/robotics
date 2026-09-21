/*
 * 14_show_routine - номер для показа. Wi-Fi не используется.
 *
 * Что делает по порядку:
 *   1. мигает фарой 5 секунд - есть время поставить на пол и отойти
 *   2. голова в центр
 *   3. едет петлю (квадрат из четырёх сторон)
 *   4. "танцует" - плавно покачивается влево-вправо
 *   5. осматривается - голова влево, вправо, вверх, вниз
 *   6. фара мигает три раза, конец. RESET для повтора.
 *
 * ПОДКЛЮЧЕНИЕ
 *   Серво 1 (поворот, влево-вправо)  -> GPIO 2
 *   Серво 2 (наклон, вверх-вниз)     -> GPIO 1
 *   Оба серво: красный на 5 В, коричневый на общую землю. НЕ на 3.3V.
 *
 * ВАЖНО
 *   Серво на GPIO 1 дёрнется при включении и при заливке - это нормально.
 *   USE_SERIAL 1 включает логи, но тогда серво 2 будет дёргаться постоянно.
 *   Первый запуск - с вывешенными колёсами и головой, которой есть куда ехать.
 */

#define USE_SERIAL 0      // 1 = логи (серво 2 отключить), 0 = серво 2 работает

#if USE_SERIAL
  #define LOG(...)   Serial.printf(__VA_ARGS__)
  #define LOGLN(s)   Serial.println(s)
#else
  #define LOG(...)
  #define LOGLN(s)
#endif

// ----------------------------------------------------------------- моторы
#define PIN_L_IN1 13
#define PIN_L_IN2 12
#define PIN_R_IN1 15
#define PIN_R_IN2 14
#define PIN_FLASH_LED 4

#define CH_L_IN1 2
#define CH_L_IN2 3
#define CH_R_IN1 4
#define CH_R_IN2 5
#define MOTOR_FREQ 1000
#define MOTOR_RES  8

// ------------------------------------------------------------------ серво
#define PIN_PAN   2       // поворот влево-вправо
#define PIN_TILT  1       // наклон вверх-вниз
#define CH_PAN    6
#define CH_TILT   7
#define SERVO_FREQ 50
#define SERVO_RES  16

// Длительность импульса для 0 и 180 градусов.
// 1000-2000 - безопасно для любого серво. 500-2500 - полный ход, но дешёвые
// клоны SG90 упираются в механический стопор и начинают выть.
#define PULSE_MIN_US 1000
#define PULSE_MAX_US 2000

// Пределы углов. Если серво упирается и начинает выть - сужай эти числа!
#define PAN_MIN     45
#define PAN_CENTER  90
#define PAN_MAX    135
#define TILT_MIN    60
#define TILT_CENTER 90
#define TILT_MAX   120

// --------------------------------------------------- настройки номера
#define START_DELAY_MS  5000   // время поставить робота и отойти

#define DRIVE_SPEED      190   // скорость по прямой, 0..255
#define TURN_SPEED       165   // скорость на повороте
#define DRIVE_MS        1100   // длина одной стороны петли
#define TURN_MS          600   // поворот - подбери под ~90 градusов
#define LAPS               4   // сторон в петле

#define DANCE_SPEED      150   // скорость покачивания - помедленнее, "мило"
#define DANCE_MS         420   // длительность одного качка
#define DANCE_TIMES        3   // сколько раз влево-вправо

#define SERVO_STEP_MS     14   // чем больше, тем плавнее голова
#define PAUSE_MS         350

// ----------------------------------------------------------- ШИМ, оба ядра
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch, f, r)  ledcAttach((pin), (f), (r))
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((pin), (duty))
#else
  #define PWM_SETUP(pin, ch, f, r)  do { ledcSetup((ch), (f), (r)); \
                                         ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty)  ledcWrite((ch), (duty))
#endif

static int panNow  = PAN_CENTER;
static int tiltNow = TILT_CENTER;

// ------------------------------------------------------------------ серво
// Угол 0..180 -> импульс 500..2500 мкс при периоде 20000 мкс.
static uint32_t angleToDuty(int deg) {
  if (deg < 0)   deg = 0;
  if (deg > 180) deg = 180;
  // Безопасный диапазон 1000-2000 мкс: дешёвые SG90 не упираются в стопоры.
  uint32_t us = PULSE_MIN_US + (uint32_t)deg * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  return (us * 65536UL) / 20000UL;
}

static void panTo(int deg)  { PWM_WRITE(PIN_PAN,  CH_PAN,  angleToDuty(deg)); }
static void tiltTo(int deg) { PWM_WRITE(PIN_TILT, CH_TILT, angleToDuty(deg)); }

// Плавно доводит поворот до нужного угла, по одному градусу.
static void panSmooth(int target) {
  if (target < PAN_MIN) target = PAN_MIN;
  if (target > PAN_MAX) target = PAN_MAX;
  int step = (target > panNow) ? 1 : -1;
  while (panNow != target) {
    panNow += step;
    panTo(panNow);
    delay(SERVO_STEP_MS);
  }
}

static void tiltSmooth(int target) {
  if (target < TILT_MIN) target = TILT_MIN;
  if (target > TILT_MAX) target = TILT_MAX;
  int step = (target > tiltNow) ? 1 : -1;
  while (tiltNow != target) {
    tiltNow += step;
    tiltTo(tiltNow);
    delay(SERVO_STEP_MS);
  }
}

// ------------------------------------------------------------------ моторы
static void sideDrive(int pinA, int chA, int pinB, int chB, int v) {
  if (v > 0)      { PWM_WRITE(pinA, chA, v); PWM_WRITE(pinB, chB, 0); }
  else if (v < 0) { PWM_WRITE(pinA, chA, 0); PWM_WRITE(pinB, chB, -v); }
  else            { PWM_WRITE(pinA, chA, 0); PWM_WRITE(pinB, chB, 0); }
}

static void drive(int left, int right) {
  if (left  >  255) left  =  255;
  if (left  < -255) left  = -255;
  if (right >  255) right =  255;
  if (right < -255) right = -255;
  sideDrive(PIN_L_IN1, CH_L_IN1, PIN_L_IN2, CH_L_IN2, left);
  sideDrive(PIN_R_IN1, CH_R_IN1, PIN_R_IN2, CH_R_IN2, right);
}

// Плавный разгон: гасит броски тока, от которых плата уходила в перезагрузку.
static void driveRamp(int left, int right, int ms) {
  const int steps = 12;
  for (int i = 1; i <= steps; i++) {
    drive(left * i / steps, right * i / steps);
    delay(8);
  }
  delay(ms > steps * 8 ? ms - steps * 8 : 0);
}

static void stopMotors() {
  drive(0, 0);
  delay(PAUSE_MS);
}

static void blink(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_FLASH_LED, HIGH); delay(ms);
    digitalWrite(PIN_FLASH_LED, LOW);  delay(ms);
  }
}

// ------------------------------------------------------------- части номера
static void actDriveLoop() {
  LOGLN("petlya");
  for (int lap = 1; lap <= LAPS; lap++) {
    LOG("  storona %d/%d\n", lap, LAPS);
    driveRamp(DRIVE_SPEED, DRIVE_SPEED, DRIVE_MS);
    stopMotors();
    driveRamp(TURN_SPEED, -TURN_SPEED, TURN_MS);
    stopMotors();
  }
}

static void actDance() {
  LOGLN("tanec");
  for (int i = 0; i < DANCE_TIMES; i++) {
    driveRamp(-DANCE_SPEED, DANCE_SPEED, DANCE_MS);   // качок влево
    stopMotors();
    driveRamp(DANCE_SPEED, -DANCE_SPEED, DANCE_MS);   // качок вправо
    stopMotors();
  }
  // возврат примерно в исходное направление
  driveRamp(-DANCE_SPEED, DANCE_SPEED, DANCE_MS / 2);
  stopMotors();
}

static void actLookAround() {
  LOGLN("osmatrivaetsya");
  panSmooth(PAN_MAX);        delay(500);   // влево
  panSmooth(PAN_MIN);        delay(500);   // вправо
  panSmooth(PAN_CENTER);     delay(300);
  tiltSmooth(TILT_MIN);      delay(500);   // вверх
  tiltSmooth(TILT_MAX);      delay(500);   // вниз
  tiltSmooth(TILT_CENTER);   delay(300);
  // напоследок - кивок
  tiltSmooth(TILT_CENTER - 15); delay(200);
  tiltSmooth(TILT_CENTER);      delay(200);
}

void setup() {
#if USE_SERIAL
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== show routine ===");
#endif

  pinMode(PIN_FLASH_LED, OUTPUT);
  digitalWrite(PIN_FLASH_LED, LOW);

  PWM_SETUP(PIN_L_IN1, CH_L_IN1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_L_IN2, CH_L_IN2, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_R_IN1, CH_R_IN1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_R_IN2, CH_R_IN2, MOTOR_FREQ, MOTOR_RES);
  drive(0, 0);

  PWM_SETUP(PIN_PAN,  CH_PAN,  SERVO_FREQ, SERVO_RES);
  PWM_SETUP(PIN_TILT, CH_TILT, SERVO_FREQ, SERVO_RES);
  panTo(PAN_CENTER);
  tiltTo(TILT_CENTER);

  LOG("start cherez %d ms\n", START_DELAY_MS);
  blink(START_DELAY_MS / 400, 200);

  actDriveLoop();
  actDance();
  actLookAround();

  drive(0, 0);
  panTo(PAN_CENTER);
  tiltTo(TILT_CENTER);
  LOGLN("konec. RESET dlya povtora.");
  blink(3, 300);
}

void loop() {
  drive(0, 0);      // страховка: моторы выключены
  delay(500);
}
