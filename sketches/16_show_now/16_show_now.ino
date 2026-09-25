/*
 * 16_show_now - номер для показа. Wi-Fi нет, Serial нет, серво нет.
 * Эта версия отработала на школьном показе 25.09.
 *
 * WIRING_SIDES 0 - моторы сгруппированы ПЕРЕДНИЕ / ЗАДНИЕ.
 *                  Поворотов нет, номер едет по прямой.
 * WIRING_SIDES 1 - моторы сгруппированы ЛЕВЫЕ / ПРАВЫЕ (после
 *                  перестановки проводов на драйвере). Есть повороты.
 *
 * Серво отключить физически: они воют и не двигаются, причина не найдена.
 *
 * Порядок: фара мигает 5 сек -> номер -> три мигания. RESET для повтора.
 */

#define WIRING_SIDES 0

// ---- ПРАВКА НАПРАВЛЕНИЯ. Меняй 0 на 1, если едет не туда.
#define INVERT_A 0    // группа на пинах 13/12
#define INVERT_B 0    // группа на пинах 15/14

// ---- пины
#define PIN_A_IN1 13
#define PIN_A_IN2 12
#define PIN_B_IN1 15
#define PIN_B_IN2 14
#define PIN_FLASH  4

// ---- каналы
#define CH_A1 2
#define CH_A2 3
#define CH_B1 4
#define CH_B2 5

#define MOTOR_FREQ 1000
#define MOTOR_RES  8

// ---- подстройка номера
#define SPEED_FWD   170   // скорость вперёд 0..255
#define SPEED_TURN  180   // скорость поворота (только WIRING_SIDES 1)
#define DRIVE_MS   1200   // длина проезда, мс
#define TURN_MS    1300   // время поворота на 90 град, мс
#define SHUFFLE_MS  220   // длина одного шага "шарканья"
#define SHUFFLE_N     6   // сколько шагов

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch, freq, res) ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, ch, duty)      ledcWrite(pin, duty)
#else
  #define PWM_SETUP(pin, ch, freq, res) do { ledcSetup(ch, freq, res); \
                                             ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, duty)      ledcWrite(ch, duty)
#endif

// ------------------------------------------------------------------ моторы
// a, b от -255 (назад) до 255 (вперёд). Что такое a и b - зависит от
// того, как провода воткнуты в драйвер: передние/задние или левые/правые.
static void motors(int a, int b) {
#if INVERT_A
  a = -a;
#endif
#if INVERT_B
  b = -b;
#endif

  if (a > 255)  a = 255;
  if (a < -255) a = -255;
  if (b > 255)  b = 255;
  if (b < -255) b = -255;

  if (a >= 0) {
    PWM_WRITE(PIN_A_IN1, CH_A1, a);
    PWM_WRITE(PIN_A_IN2, CH_A2, 0);
  } else {
    PWM_WRITE(PIN_A_IN1, CH_A1, 0);
    PWM_WRITE(PIN_A_IN2, CH_A2, -a);
  }

  if (b >= 0) {
    PWM_WRITE(PIN_B_IN1, CH_B1, b);
    PWM_WRITE(PIN_B_IN2, CH_B2, 0);
  } else {
    PWM_WRITE(PIN_B_IN1, CH_B1, 0);
    PWM_WRITE(PIN_B_IN2, CH_B2, -b);
  }
}

static void allStop() { motors(0, 0); }

// Плавный разгон, чтобы робот не дёргался и не буксовал.
static void ramp(int aFrom, int bFrom, int aTo, int bTo) {
  for (int i = 1; i <= 12; i++) {
    motors(aFrom + (aTo - aFrom) * i / 12,
           bFrom + (bTo - bFrom) * i / 12);
    delay(8);
  }
}

static void goFwd(int ms) {
  ramp(0, 0, SPEED_FWD, SPEED_FWD);
  delay(ms);
  ramp(SPEED_FWD, SPEED_FWD, 0, 0);
  allStop();
}

static void goBack(int ms) {
  ramp(0, 0, -SPEED_FWD, -SPEED_FWD);
  delay(ms);
  ramp(-SPEED_FWD, -SPEED_FWD, 0, 0);
  allStop();
}

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
#if WIRING_SIDES

// Провода стоят по бортам: группа A = левый борт, B = правый.
static void spinRight(int ms) {
  ramp(0, 0, SPEED_TURN, 0);
  delay(ms);
  ramp(SPEED_TURN, 0, 0, 0);
  allStop();
}

static void spinLeft(int ms) {
  ramp(0, 0, 0, SPEED_TURN);
  delay(ms);
  ramp(0, SPEED_TURN, 0, 0);
  allStop();
}

static void actMain() {
  // Квадрат из четырёх сторон.
  for (int side = 0; side < 4; side++) {
    goFwd(DRIVE_MS);
    delay(250);
    spinRight(TURN_MS);
    delay(250);
  }
  delay(400);

  // Покачивание влево-вправо.
  for (int i = 0; i < 3; i++) {
    spinLeft(450);
    delay(150);
    spinRight(900);
    delay(150);
    spinLeft(450);
    delay(300);
  }
}

#else

// Провода стоят передние/задние: поворотов нет.
// "Шарканье" - по очереди толкает передняя и задняя пара. Смотрится
// как будто робот переступает с ноги на ногу.
static void actShuffle() {
  for (int i = 0; i < SHUFFLE_N; i++) {
    motors(SPEED_FWD, 0);
    digitalWrite(PIN_FLASH, HIGH);
    delay(SHUFFLE_MS);
    allStop();
    digitalWrite(PIN_FLASH, LOW);
    delay(120);

    motors(0, SPEED_FWD);
    delay(SHUFFLE_MS);
    allStop();
    delay(120);
  }
}

static void actMain() {
  goFwd(DRIVE_MS);          // выехал
  delay(400);
  goBack(DRIVE_MS / 2);     // откатился
  delay(400);
  actShuffle();             // переступил с ноги на ногу
  delay(400);
  goFwd(DRIVE_MS / 2);      // рывок вперёд
  delay(300);
  goBack(250);              // поклон: назад
  delay(200);
  goFwd(250);               // и вперёд
  delay(300);
  goBack(DRIVE_MS);         // уехал назад на место
}

#endif

// ------------------------------------------------------------------- setup
void setup() {
  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW);

  PWM_SETUP(PIN_A_IN1, CH_A1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_A_IN2, CH_A2, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_B_IN1, CH_B1, MOTOR_FREQ, MOTOR_RES);
  PWM_SETUP(PIN_B_IN2, CH_B2, MOTOR_FREQ, MOTOR_RES);
  allStop();

  // 5 секунд на то, чтобы поставить робота на пол и отойти.
  blink(5, 200, 800);

  actMain();

  allStop();
  blink(3, 150, 150);
  digitalWrite(PIN_FLASH, LOW);
}

void loop() {
  allStop();
  delay(1000);
}
