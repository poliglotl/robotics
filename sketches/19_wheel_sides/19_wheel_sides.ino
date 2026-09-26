/*
 * 19_wheel_sides - проверка, что провода переставлены по бортам.
 *
 * Цикл, по кругу:
 *   фара мигает 1 раз -> должны крутиться ЛЕВЫЕ колёса (оба), 2.5 сек
 *   фара мигает 2 раза -> должны крутиться ПРАВЫЕ колёса (оба), 2.5 сек
 *   фара горит ровно   -> едут ВСЕ четыре вперёд, 2 сек
 *
 * Левое и правое считаем, глядя со стороны батарейного отсека.
 *
 * КОЛЁСА ВЫВЕСИТЬ. Serial Monitor 115200 - там подписано, что сейчас идёт.
 *
 * ЧТО ДОЛЖНО ПОЛУЧИТЬСЯ
 *   1 мигание -> оба левых, оба вперёд
 *   2 мигания -> оба правых, оба вперёд
 *   ровный свет -> все четыре вперёд, робот поехал бы прямо
 *
 * ЕСЛИ НЕ ТАК
 *   крутятся передние и задние вместо бортов -> провода ещё не переставлены
 *   группы поменялись местами -> поставь SWAP_GROUPS 1
 *   вся группа крутится назад -> поставь INVERT_A или INVERT_B в 1
 *   в группе одно колесо вперёд, другое назад -> поменять провода
 *     этого мотора местами на клемме драйвера
 */

#define SWAP_GROUPS 0     // 1 = группа B это левый борт, а не правый
#define INVERT_A    0     // 1 = группа на пинах 13/12 крутится назад
#define INVERT_B    0     // 1 = группа на пинах 15/14 крутится назад

#define PIN_A_IN1 13
#define PIN_A_IN2 12
#define PIN_B_IN1 15
#define PIN_B_IN2 14
#define PIN_FLASH  4

#define CH_A1 2
#define CH_A2 3
#define CH_B1 4
#define CH_B2 5

#define MOTOR_FREQ 1000
#define MOTOR_RES  8
#define TEST_SPEED 180
#define RUN_MS    2500

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch) ledcAttach((pin), MOTOR_FREQ, MOTOR_RES)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
  #define PWM_SETUP(pin, ch) do { ledcSetup((ch), MOTOR_FREQ, MOTOR_RES); \
                                  ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

// Низкий уровень: a - группа на пинах 13/12, b - группа на 15/14.
static void groups(int a, int b) {
#if INVERT_A
  a = -a;
#endif
#if INVERT_B
  b = -b;
#endif
  if (a >= 0) { PWM_WRITE(PIN_A_IN1, CH_A1, a);  PWM_WRITE(PIN_A_IN2, CH_A2, 0);  }
  else        { PWM_WRITE(PIN_A_IN1, CH_A1, 0);  PWM_WRITE(PIN_A_IN2, CH_A2, -a); }
  if (b >= 0) { PWM_WRITE(PIN_B_IN1, CH_B1, b);  PWM_WRITE(PIN_B_IN2, CH_B2, 0);  }
  else        { PWM_WRITE(PIN_B_IN1, CH_B1, 0);  PWM_WRITE(PIN_B_IN2, CH_B2, -b); }
}

// Борта. Если провода переставлены правильно, это и есть левый и правый.
static void sides(int left, int right) {
#if SWAP_GROUPS
  groups(right, left);
#else
  groups(left, right);
#endif
}

static void allStop() { sides(0, 0); }

static void blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_FLASH, HIGH);
    delay(150);
    digitalWrite(PIN_FLASH, LOW);
    delay(250);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== 19_wheel_sides ===");
  Serial.println("levoe i pravoe - glyadya so storony batareynogo otseka");
  Serial.printf("SWAP_GROUPS %d | INVERT_A %d | INVERT_B %d\n",
                SWAP_GROUPS, INVERT_A, INVERT_B);
  Serial.println("KOLYOSA VYVESIT!");
  Serial.println();

  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW);

  PWM_SETUP(PIN_A_IN1, CH_A1);
  PWM_SETUP(PIN_A_IN2, CH_A2);
  PWM_SETUP(PIN_B_IN1, CH_B1);
  PWM_SETUP(PIN_B_IN2, CH_B2);
  allStop();
  delay(2000);
}

void loop() {
  // 1 мигание - левый борт
  Serial.println("1 mignula -> LEVYE kolyosa, oba, vperyod");
  blink(1);
  sides(TEST_SPEED, 0);
  delay(RUN_MS);
  allStop();
  delay(1500);

  // 2 мигания - правый борт
  Serial.println("2 mignula -> PRAVYE kolyosa, oba, vperyod");
  blink(2);
  sides(0, TEST_SPEED);
  delay(RUN_MS);
  allStop();
  delay(1500);

  // Ровный свет - все четыре вперёд
  Serial.println("svet gorit -> VSE chetyre vperyod, dolzhen ehat pryamo");
  digitalWrite(PIN_FLASH, HIGH);
  sides(TEST_SPEED, TEST_SPEED);
  delay(2000);
  allStop();
  digitalWrite(PIN_FLASH, LOW);
  Serial.println();
  delay(3000);
}
