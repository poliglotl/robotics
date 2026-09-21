/*
 * 16_show_now - номер для показа. Wi-Fi нет, Serial нет.
 *
 * ENABLE_SERVOS 0 - только колёса и фара. Заливай ЭТО.
 * ENABLE_SERVOS 1 - добавляет голову, если серво успели проверить.
 *
 * Серво (только при ENABLE_SERVOS 1):
 *   поворот  -> GPIO 2, наклон -> GPIO 1
 *   красные -> +5V красной платы, коричневые -> GND, батарейки включены
 *
 * Порядок: фара мигает 5 сек -> квадрат -> покачивание -> голова
 * осматривается -> три мигания. RESET для повтора.
 */

#define ENABLE_SERVOS 0

// ---- пины
#define PIN_L_IN1 13
#define PIN_L_IN2 12
#define PIN_R_IN1 15
#define PIN_R_IN2 14
#define PIN_FLASH 4
#define PIN_PAN    2
#define PIN_TILT   1

// ---- каналы
#define CH_L1 2
#define CH_L2 3
#define CH_R1 4
#define CH_R2 5
#define CH_PAN  6
#define CH_TILT 7

#define MOTOR_FREQ 1000
#define MOTOR_RES  8

// ---- подстройка номера
#define SPEED_FWD  170    // скорость вперёд 0..255
#define SPEED_TURN 150    // скорость поворота
#define DRIVE_MS   1200   // длина стороны квадрата, мс
#define TURN_MS    650    // время поворота на 90 град, мс
#define DANCE_TIMES 3     // сколько покачиваний

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch, freq, res) ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, ch, duty)      ledcWrite(pin, duty)
#else
  #define PWM_SETUP(pin, ch, freq, res) do { ledcSetup(ch, freq, res); \
                                             ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, duty)      ledcWrite(ch, duty)
#endif

// ------------------------------------------------------------------ моторы
// speed от -255 (назад) до 255 (вперёд)
static void wheels(int left, int right) {
  if (left > 255)  left = 255;
  if (left < -255) left = -255;
  if (right > 255)  right = 255;
  if (right < -255) right = -255;

  if (left >= 0) {
    PWM_WRITE(PIN_L_IN1, CH_L1, left);
    PWM_WRITE(PIN_L_IN2, CH_L2, 0);
  } else {
    PWM_WRITE(PIN_L_IN1, CH_L1, 0);
    PWM_WRITE(PIN_L_IN2, CH_L2, -left);
  }

  if (right >= 0) {
    PWM_WRITE(PIN_R_IN1, CH_R1, right);
    PWM_WRITE(PIN_R_IN2, CH_R2, 0);
  } else {
    PWM_WRITE(PIN_R_IN1, CH_R1, 0);
    PWM_WRITE(PIN_R_IN2, CH_R2, -right);
  }
}

static void stopWheels() { wheels(0, 0); }

// Плавный разгон, чтобы робот не дёргался и не буксовал.
static void ramp(int lFrom, int rFrom, int lTo, int rTo) {
  for (int i = 1; i <= 12; i++) {
    wheels(lFrom + (lTo - lFrom) * i / 12,
           rFrom + (rTo - rFrom) * i / 12);
    delay(8);
  }
}

static void goFwd(int ms) {
  ramp(0, 0, SPEED_FWD, SPEED_FWD);
  delay(ms);
  ramp(SPEED_FWD, SPEED_FWD, 0, 0);
  stopWheels();
}

static void spinRight(int ms) {
  ramp(0, 0, SPEED_TURN, -SPEED_TURN);
  delay(ms);
  ramp(SPEED_TURN, -SPEED_TURN, 0, 0);
  stopWheels();
}

static void spinLeft(int ms) {
  ramp(0, 0, -SPEED_TURN, SPEED_TURN);
  delay(ms);
  ramp(-SPEED_TURN, SPEED_TURN, 0, 0);
  stopWheels();
}

// ------------------------------------------------------------------- серво
#if ENABLE_SERVOS
  #define SERVO_FREQ 50
  #define SERVO_RES  16
  #define PULSE_MIN_US 1000
  #define PULSE_MAX_US 2000

  // Узкие пределы: безопасно, даже если серво не успели проверить.
  #define PAN_MIN     70
  #define PAN_CENTER  90
  #define PAN_MAX    110
  #define TILT_MIN    80
  #define TILT_CENTER 90
  #define TILT_MAX   100
  #define SERVO_STEP_MS 14

  static int panNow  = PAN_CENTER;
  static int tiltNow = TILT_CENTER;

  static uint32_t angleToDuty(int deg) {
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    uint32_t us = PULSE_MIN_US + (uint32_t)deg * (PULSE_MAX_US - PULSE_MIN_US) / 180;
    return (us * 65536UL) / 20000UL;
  }

  static void panSmooth(int target) {
    if (target < PAN_MIN) target = PAN_MIN;
    if (target > PAN_MAX) target = PAN_MAX;
    int step = (target > panNow) ? 1 : -1;
    while (panNow != target) {
      panNow += step;
      PWM_WRITE(PIN_PAN, CH_PAN, angleToDuty(panNow));
      delay(SERVO_STEP_MS);
    }
  }

  static void tiltSmooth(int target) {
    if (target < TILT_MIN) target = TILT_MIN;
    if (target > TILT_MAX) target = TILT_MAX;
    int step = (target > tiltNow) ? 1 : -1;
    while (tiltNow != target) {
      tiltNow += step;
      PWM_WRITE(PIN_TILT, CH_TILT, angleToDuty(tiltNow));
      delay(SERVO_STEP_MS);
    }
  }
#endif

// -------------------------------------------------------------------- фара
static void blink(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_FLASH, HIGH);
    delay(onMs);
    digitalWrite(PIN_FLASH, LOW);
    delay(offMs);
  }
}

// -------------------------------------------------------------------- акты
static void actDriveLoop() {
  for (int side = 0; side < 4; side++) {
    goFwd(DRIVE_MS);
    delay(250);
    spinRight(TURN_MS);
    delay(250);
  }
}

static void actDance() {
  for (int i = 0; i < DANCE_TIMES; i++) {
    spinLeft(300);
    delay(150);
    spinRight(600);
    delay(150);
    spinLeft(300);
    delay(300);
  }
}

static void actLookAround() {
#if ENABLE_SERVOS
  panSmooth(PAN_MIN);
  delay(500);
  panSmooth(PAN_MAX);
  delay(500);
  panSmooth(PAN_CENTER);
  delay(400);
  tiltSmooth(TILT_MAX);
  delay(500);
  tiltSmooth(TILT_MIN);
  delay(500);
  tiltSmooth(TILT_CENTER);
  delay(400);
#else
  // Серво нет - вместо головы короткий поклон: назад и вперёд.
  ramp(0, 0, -SPEED_FWD, -SPEED_FWD);
  delay(350);
  ramp(-SPEED_FWD, -SPEED_FWD, 0, 0);
  stopWheels();
  delay(300);
  goFwd(350);
#endif
}

// ------------------------------------------------------------------- setup
void setup() {
  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW);

  PWM_SETUP(PIN_L_IN1, CH_L1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_L_IN2, CH_L2, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_R_IN1, CH_R1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_R_IN2, CH_R2, MOTOR_FREQ, MOTOR_RES);
  stopWheels();

#if ENABLE_SERVOS
  PWM_SETUP(PIN_PAN,  CH_PAN,  SERVO_FREQ, SERVO_RES);
  PWM_SETUP(PIN_TILT, CH_TILT, SERVO_FREQ, SERVO_RES);
  PWM_WRITE(PIN_PAN,  CH_PAN,  angleToDuty(PAN_CENTER));
  PWM_WRITE(PIN_TILT, CH_TILT, angleToDuty(TILT_CENTER));
  delay(600);
#endif

  // 5 секунд на то, чтобы поставить робота на пол и отойти.
  blink(5, 200, 800);

  actDriveLoop();
  delay(400);
  actDance();
  delay(400);
  actLookAround();

  stopWheels();
  blink(3, 150, 150);
  digitalWrite(PIN_FLASH, LOW);
}

void loop() {
  stopWheels();
  delay(1000);
}
