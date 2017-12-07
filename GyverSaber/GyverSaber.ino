/*
     СУПЕР-МЕГО-КРУТОЙ СВЕТОВОЙ МЕЧ НА АРДУИНЕ И АДРЕСНЫХ СВЕТОДИОДАХ!
   ЖЕЛЕЗО:
     Используем адресные светодиоды получить любой  цвет, а также плавное включение
     Используем SD карту, чтобы воспроизводить звуки
     Используем IMU (аксель + гироскоп), чтобы генерировать гудящий звук в зависимости от взмахов
     ИЛИ опираясь на взмахи, воспроизводить готовый звук с карты памяти
   ВОЗМОЖНОСТИ:
     Плавное включение/выключение со звуками меча
     Во время работы меч "пульсирует" случайным образом
     Во время работы издаёт звуки:
       РЕЖИМ 1: тон "гудения" зависит от угловой скорости (гироскоп) поворота меча, т.е. взмаха
       РЕЖИМ 2: гудение и звуки взмахов воспроизводятся с карты памяти
         Медленный взмах - длинный звук взмаха (случайно один из 4)
         Быстрый взмах - короткий звук взмаха (случайно один из 5)
     При ударе меч вспыхивает ярко-белым
     При ударе воспроизводится один из 16 звуков удара (случайно)
       Слабый удар - короткие звуки
       Сильный удар - длинные звуки
     При включении показывает уровень заряда аккумулятора длиной светящейся части в процентах
     Следит за напряжением аккумулятора:
       Аккумулятор разрядился ДО ВКЛЮЧЕНИЯ: меч не включится, светодиод кнопки мигнёт несколько раз
       Аккумулятор разрядился ВО ВРЕМЯ РАБОТЫ: меч выключается
   УПРАВЛЕНИЕ:
     Включение/выключение по удерживанию кнопки
     Тройное нажатие - смена цвета (красный - зелёный - синий - жёлтый - розовый - голубой)
     Пятерное нажатие - смена звукового режима (режим генерации и режим звуков с карты памяти)
     Выбранный цвет и режим хранится в памяти и не сбрасывается при перезагрузке
*/

// ---------------------------- НАСТРОЙКИ -------------------------------
#define NUM_LEDS 30         // число МИКРОСХЕМ на ленте
#define BTN_TIMEOUT 800     // задержка кнопки для удерживания (миллисекунды)
#define BRIGHTNESS 255      // максимальная яркость ленты (0 - 255)

#define SWING_TIMEOUT 500   // таймаут между двумя взмахами
#define SWING_L_THR 150     // порог угловой скорости для взмаха
#define SWING_THR 300       // порог угловой скорости для сильного взмаха
#define STRIKE_THR 150      // порог ускорения для распознавания удара
#define STRIKE_S_THR 300    // порог ускорения для распознавания сильного удара
#define FLASH_DELAY 80      // время вспышки при ударе (миллисекунды)

#define BLINK_ALLOW 1       // разрешить мерцание
#define BLINK_AMPL 20       // амплитуда мерцания клинка
#define BLINK_DELAY 30      // задержка между мерцаниями

#define R1 100000           // сопротивление резистора делителя    
#define R2 51000            // сопротивление резистора делителя
#define BATTERY_SAFE 1      // не включаться и выключаться при низком заряде АКБ
// ---------------------------- НАСТРОЙКИ -------------------------------

#define LED_PIN 6           // пин, куда подключен DIN ленты
#define BTN 3               // пин кнопки
#define IMU_GND A1          // земля акселерометра
#define SD_GND A0           // земля карты
#define VOLT_PIN A6         // пин вольтметра
#define BTN_LED 4           // светодиод кнопки

// -------------------------- БИБЛИОТЕКИ ---------------------------
#include <avr/pgmspace.h>   // библиотека для работы с ПРОГМЕМ
#include <SD.h>             // библиотека для работы с SD картой
#include <TMRpcm.h>         // библиотека для работы с аудио
#include "Wire.h"           // вспомогательная библиотека для работы с акселерометром
#include "I2Cdev.h"         // вспомогательная библиотека для работы с акселерометром
#include "MPU6050.h"        // библиотека для работы с акселерометром
#include <toneAC.h>         // библиотека расширенной генерации звука
#include "FastLED.h"        // библиотека для работы с адресной лентой
#include <EEPROM.h>         // библиотека для работы с энергонезависимой памятью

// создание объектов
CRGB leds[NUM_LEDS];
#define SD_ChipSelectPin 10
TMRpcm tmrpcm;
MPU6050 accelgyro;
// -------------------------- БИБЛИОТЕКИ ---------------------------


// ------------------------------ ПЕРЕМЕННЫЕ ---------------------------------
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long ACC, GYR, COMPL;
int gyroX, gyroY, gyroZ, accelX, accelY, accelZ, freq, freq_f = 20;
float k = 0.2;
unsigned long humTimer = -9000, mpuTimer, nowTimer;
int stopTimer;
boolean bzzz_flag, ls_chg_state, ls_state;
boolean btnState, btn_flag, hold_flag;
byte btn_counter;
unsigned long btn_timer, blink_timer, swing_timer, swing_timeout, battery_timer, bzzTimer;
byte nowNumber;
byte LEDcolor;  // 0 - красный, 1 - синий, 2 - зелёный, 3 - розовый, 4 - жёлтый
byte nowColor, red, green, blue, redOffset, greenOffset, blueOffset;
boolean eeprom_flag, swing_flag, swing_allow, strike_flag, HUMmode;
float voltage;
int blinkOffset;
// Hummude: 0 - генерация, 1 - с карты
// ------------------------------ ПЕРЕМЕННЫЕ ---------------------------------

// --------------------------------- ЗВУКИ УДАРОВ ----------------------------------
// ----------------- ДЛЯ ЭКОНОМИИ ПАМЯТИ НАЗВАНИЯ ХРАНЯТСЯ В PROGMEM ---------------
const char strike1[] PROGMEM = "SK1.wav";
const char strike2[] PROGMEM = "SK2.wav";
const char strike3[] PROGMEM = "SK3.wav";
const char strike4[] PROGMEM = "SK4.wav";
const char strike5[] PROGMEM = "SK5.wav";
const char strike6[] PROGMEM = "SK6.wav";
const char strike7[] PROGMEM = "SK7.wav";
const char strike8[] PROGMEM = "SK8.wav";

const char* const strikes[] PROGMEM  = {        // создаём "массив" имён (по сути это их адреса)
  strike1, strike2, strike3, strike4, strike5, strike6, strike7, strike8
};

int strike_time[8] = {779, 563, 687, 702, 673, 661, 666, 635};

const char strike_s1[] PROGMEM = "SKS1.wav";
const char strike_s2[] PROGMEM = "SKS2.wav";
const char strike_s3[] PROGMEM = "SKS3.wav";
const char strike_s4[] PROGMEM = "SKS4.wav";
const char strike_s5[] PROGMEM = "SKS5.wav";
const char strike_s6[] PROGMEM = "SKS6.wav";
const char strike_s7[] PROGMEM = "SKS7.wav";
const char strike_s8[] PROGMEM = "SKS8.wav";

const char* const strikes_short[] PROGMEM = {        // создаём "массив" имён (по сути это их адреса)
  strike_s1, strike_s2, strike_s3, strike_s4,
  strike_s5, strike_s6, strike_s7, strike_s8
};
int strike_s_time[8] = {270, 167, 186, 250, 252, 255, 250, 238};

const char swing1[] PROGMEM = "SWS1.wav";
const char swing2[] PROGMEM = "SWS2.wav";
const char swing3[] PROGMEM = "SWS3.wav";
const char swing4[] PROGMEM = "SWS4.wav";
const char swing5[] PROGMEM = "SWS5.wav";

const char* const swings[] PROGMEM  = {        // создаём "массив" имён (по сути это их адреса)
  swing1, swing2, swing3, swing4, swing5
};
int swing_time[8] = {389, 372, 360, 366, 337};

const char swingL1[] PROGMEM = "SWL1.wav";
const char swingL2[] PROGMEM = "SWL2.wav";
const char swingL3[] PROGMEM = "SWL3.wav";
const char swingL4[] PROGMEM = "SWL4.wav";

const char* const swings_L[] PROGMEM  = {        // создаём "массив" имён (по сути это их адреса)
  swingL1, swingL2, swingL3, swingL4
};
int swing_time_L[8] = {636, 441, 772, 702};

char BUFFER[10];
// --------------------------------- ЗВУКИ УДАРОВ ---------------------------------

void setup() {
  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(100);
  setAll(0, 0, 0);             // ставим чёрный цвет ленты

  Wire.begin();
  Serial.begin(9600);
  pinMode(BTN, INPUT_PULLUP);
  pinMode(IMU_GND, OUTPUT);
  pinMode(SD_GND, OUTPUT);
  pinMode(BTN_LED, OUTPUT);
  digitalWrite(IMU_GND, 0);
  digitalWrite(SD_GND, 0);
  digitalWrite(BTN_LED, 1);
  randomSeed(analogRead(0));
  // инициализация и настройка воспроизведения с карты
  tmrpcm.speakerPin = 9;
  if (SD.begin(8)) Serial.println("SD ok");
  tmrpcm.setVolume(5);
  tmrpcm.quality(1);

  // инициализация и настройка IMU
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  if ((EEPROM.read(0) >= 0) && (EEPROM.read(0) <= 5)) {  // если был хоть один запуск прошивки
    nowColor = EEPROM.read(0);   // вспоминаем из памяти выбранный цвет
    HUMmode = EEPROM.read(1);    // вспоминаем из памяти выбранный режим
  } else {                       // если это первый запуск
    EEPROM.write(0, 0);          // обнуляем ячейку
    EEPROM.write(1, 0);          // обнуляем ячейку
    nowColor = 0;                // цвет нулевой
  }
  setColor(nowColor);            // устанавливаем цвет

  byte capacity = voltage_measure();   // получить процент заряда аккумулятора
  capacity = map(capacity, 100, 0, (NUM_LEDS / 2 - 1), 1);  // перевести в длину клинка

  for (char i = 0; i <= capacity; i++) {          // включить все диоды выбранным цветом
    setPixel(i, red, green, blue);
    setPixel((NUM_LEDS - 1 - i), red, green, blue);
    FastLED.show();
    delay(25);
  }
  delay(1000);                 // секунда для отображения заряда акума
  setAll(0, 0, 0);             // показываем чёрный цвет ленты
  FastLED.setBrightness(BRIGHTNESS); // яркость
}

void loop() {
  randomBlink();      // мерцание
  getFreq();          // получить частоту для трещалки
  on_off_sound();     // вкл/выкл меча со звуками
  btnTick();          // опрос и отработка кнопки
  strikeTick();       // отработка удара
  swingTick();        // отработка взмаха
  batteryTick();      // проверка акума
}

void btnTick() {
  btnState = !digitalRead(BTN);    // если кнопка нажата
  if (btnState && !btn_flag) {
    btn_flag = 1;
    btn_counter++;                 // прибавить счётчик нажатий
    btn_timer = millis();
  }
  if (!btnState && btn_flag) {     // если была нажата и отпущена
    btn_flag = 0;
    hold_flag = 0;                 // сбросить флаг удержания
  }
  // если кнопка удерживается
  if (btn_flag && btnState && (millis() - btn_timer > BTN_TIMEOUT) && !hold_flag) {
    ls_chg_state = 1;     // включить/выключить меч
    hold_flag = 1;
    btn_counter = 0;
  }
  // если кнопка была нажата несколько раз до таймаута
  if ((millis() - btn_timer > BTN_TIMEOUT) && (btn_counter != 0)) {
    if (ls_state) {
      if (btn_counter == 3) {               // если число нажатий равно 3
        nowColor++;                         // сменить цвет
        if (nowColor >= 6) nowColor = 0;    // закольцевать смену цвета
        setColor(nowColor);                 // установить цвет
        setAll(red, green, blue);           // включить цвет
        eeprom_flag = 1;                    // разрешить запись выбранного цвета в память
      }
      if (btn_counter == 5) {               // если число нажатий равно 3
        HUMmode = !HUMmode;
        if (HUMmode) {
          noToneAC();                       // вырубить трещалку
          tmrpcm.play("HUM.wav");
        } else {
          tmrpcm.disable();                 // выключаем звук
          toneAC(freq_f);
        }
        eeprom_flag = 1;                    // разрешить запись память
      }
    }
    btn_counter = 0;
  }
}

void on_off_sound() {                // блок вкл/выкл меча со звуками
  if (ls_chg_state) {                // если есть запрос на изменение состояния меча
    if (!ls_state) {                 // если меч выключен
      if (voltage_measure() > 10 || !BATTERY_SAFE) {
        tmrpcm.play("ON.wav");         // воспроизвести звук включения
        delay(200);                    // ждём воспроизведение
        light_up();                    // лента включается
        delay(200);                    // ждём воспроизведение
        bzzz_flag = 1;                 // разрешаем трещалку
        ls_state = true;               // запомнить, что меч включен
        if (HUMmode) {
          noToneAC();                       // вырубить трещалку
          tmrpcm.play("HUM.wav");
        } else {
          tmrpcm.disable();                 // выключаем звук
          toneAC(freq_f);
        }
      } else {
        for (int i = 0; i < 5; i++) {
          digitalWrite(BTN_LED, 0);
          delay(400);
          digitalWrite(BTN_LED, 1);
          delay(400);
        }
      }
    } else {                         // если меч включен
      noToneAC();                    // вырубить трещалку
      bzzz_flag = 0;                 // запретить включение трещалки
      tmrpcm.play("OFF.wav");        // воспроизвести звук выключения
      delay(300);                    // ждём воспроизведение
      light_down();                  // лента выключается
      delay(300);                    // ждём воспроизведение
      tmrpcm.disable();              // выключаем звук
      ls_state = false;              // запомнить, что меч выключен
      if (eeprom_flag) {             // если была смена цвета
        eeprom_flag = 0;
        EEPROM.write(0, nowColor);   // записать выбранный цвет в память
        EEPROM.write(1, HUMmode);    // записать выбранный режим в память
      }
    }
    ls_chg_state = 0;                // снять флаг запроса смены состояния
  }

  if (((millis() - humTimer) > 9000) && bzzz_flag && HUMmode) {   // если настало время трещать и разрешено трещать
    tmrpcm.play("HUM.wav");
    humTimer = millis();                            // сбросить таймер
    swing_flag = 1;
    strike_flag = 0;
  }
  long delta = millis() - bzzTimer;
  if ((delta > 3) && bzzz_flag && !HUMmode) {   // если настало время трещать и разрешено трещать
    if (strike_flag) {
      tmrpcm.disable();                               // выключить звук
      strike_flag = 0;
    }
    toneAC(freq_f);                                 // трещать
    bzzTimer = millis();                            // сбросить таймер
  }
}

void randomBlink() {
  if (BLINK_ALLOW && ls_state && (millis() - blink_timer > BLINK_DELAY)) {
    blink_timer = millis();
    blinkOffset = blinkOffset * k + random(-BLINK_AMPL, BLINK_AMPL) * (1 - k);
    if (nowColor == 0) blinkOffset = constrain(blinkOffset, -15, 5);
    redOffset = constrain(red + blinkOffset, 0, 255);
    greenOffset = constrain(green + blinkOffset, 0, 255);
    blueOffset = constrain(blue + blinkOffset, 0, 255);
    setAll(redOffset, greenOffset, blueOffset);
  }
}

void strikeTick() {
  if ((ACC > STRIKE_THR) && (ACC < STRIKE_S_THR)) {      // если ускорение превысило порог
    if (!HUMmode) noToneAC();                        // выключить трещалку
    nowNumber = random(8);             // взять случайное число
    // читаем название трека из PROGMEM
    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes_short[nowNumber])));
    tmrpcm.play(BUFFER);               // воспроизвести звук удара
    strike_flash();
    if (!HUMmode)
      bzzTimer = millis() + strike_s_time[nowNumber] - FLASH_DELAY;
    else
      humTimer = millis() - 9000 + strike_s_time[nowNumber] - FLASH_DELAY;
    strike_flag = 1;
  }
  if (ACC >= STRIKE_S_THR) {           // если ускорение превысило порог
    if (!HUMmode) noToneAC();                        // выключить трещалку
    nowNumber = random(8);             // взять случайное число
    // читаем название трека из PROGMEM
    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes[nowNumber])));
    tmrpcm.play(BUFFER);               // воспроизвести звук удара
    strike_flash();
    if (!HUMmode)
      bzzTimer = millis() + strike_time[nowNumber] - FLASH_DELAY;
    else
      humTimer = millis() - 9000 + strike_time[nowNumber] - FLASH_DELAY;
    strike_flag = 1;
  }
}

void swingTick() {
  if (GYR > 80 && (millis() - swing_timeout > 100) && HUMmode) {
    swing_timeout = millis();
    if (((millis() - swing_timer) > SWING_TIMEOUT) && swing_flag && !strike_flag) {
      if (GYR >= SWING_THR) {      // если ускорение превысило порог
        nowNumber = random(5);             // взять случайное число
        // читаем название трека из PROGMEM
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings[nowNumber])));
        tmrpcm.play(BUFFER);               // воспроизвести звук взмаха
        humTimer = millis() - 9000 + swing_time[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
      if ((GYR > SWING_L_THR) && (GYR < SWING_THR)) {
        nowNumber = random(5);             // взять случайное число
        // читаем название трека из PROGMEM
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings_L[nowNumber])));
        tmrpcm.play(BUFFER);               // воспроизвести звук взмаха
        humTimer = millis() - 9000 + swing_time_L[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
    }
  }
}

void getFreq() {
  if (ls_state) {                                               // если меч включен
    if (millis() - mpuTimer > 500) {                            // каждые полмиллисекунды
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       // получить показания с IMU
      // найти абсолютное значение, разделить на 100
      gyroX = abs(gx / 100);
      gyroY = abs(gy / 100);
      gyroZ = abs(gz / 100);
      accelX = abs(ax / 100);
      accelY = abs(ay / 100);
      accelZ = abs(az / 100);

      ACC = sq((long)accelX) + sq((long)accelY) + sq((long)accelZ);
      ACC = sqrt(ACC);
      GYR = sq((long)gyroX) + sq((long)gyroY) + sq((long)gyroZ);
      GYR = sqrt((long)GYR);
      COMPL = ACC + GYR;
      /*
         // отладка работы IMU
         Serial.print("$");
         Serial.print(gyroX);
         Serial.print(" ");
         Serial.print(gyroY);
         Serial.print(" ");
         Serial.print(gyroZ);
         Serial.println(";");
      */
      freq = (long)COMPL * COMPL / 1500;                        // изменяем частоту по параболе
      freq = constrain(freq, 18, 300);                          // ставим пределы
      freq_f = freq * k + freq_f * (1 - k);                     // сглаживающий фильтр
      mpuTimer = micros();                                      // сбросить таймер
    }
  }
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for (int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  FastLED.show();
}

void light_up() {
  for (char i = 0; i <= (NUM_LEDS / 2 - 1); i++) {          // включить все диоды выбранным цветом
    setPixel(i, red, green, blue);
    setPixel((NUM_LEDS - 1 - i), red, green, blue);
    FastLED.show();
    delay(25);
  }
}
void light_down() {
  for (char i = (NUM_LEDS / 2 - 1); i >= 0; i--) {      // выключить все диоды
    setPixel(i, 0, 0, 0);
    setPixel((NUM_LEDS - 1 - i), 0, 0, 0);
    FastLED.show();
    delay(25);
  }
}
void strike_flash() {
  setAll(255, 255, 255);             // цвет клинка белым
  delay(FLASH_DELAY);                // ждать
  setAll(red, blue, green);          // цвет клинка старым цветом
}

void setColor(byte color) {
  switch (color) {
    // 0 - красный, 1 - синий, 2 - зелёный, 3 - розовый, 4 - жёлтый, 5 - голубой не хотим играть с тобой
    case 0:
      red = 255;
      green = 0;
      blue = 0;
      break;
    case 1:
      red = 0;
      green = 0;
      blue = 255;
      break;
    case 2:
      red = 0;
      green = 255;
      blue = 0;
      break;
    case 3:
      red = 255;
      green = 0;
      blue = 255;
      break;
    case 4:
      red = 255;
      green = 255;
      blue = 0;
      break;
    case 5:
      red = 0;
      green = 255;
      blue = 255;
      break;
  }
}

void batteryTick() {
  if (millis() - battery_timer > 30000 && ls_state && BATTERY_SAFE) {
    if (voltage_measure() < 15) {
      ls_chg_state = 1;
    }
    battery_timer = millis();
  }
}

// супер охуенный алгоритм, получающий из напряжения остаток заряда в процентах
// аппроксимировано вручную по графику разряда литий-иона
byte voltage_measure() {
  voltage = 0;
  for (int i = 0; i < 10; i++) {    // среднее арифметическое из 10 измерений
    voltage += (float)analogRead(VOLT_PIN) * 5 / 1023 * (R1 + R2) / R2;
  }
  voltage = voltage / 10;           // получаем среднее арифметическое
  int volts = voltage / 3 * 100;    // у нас 3 банки. делим на сотку для расчёта
  if (volts > 387)
    return map(volts, 420, 387, 100, 77);
  else if ((volts <= 387) && (volts > 375) )
    return map(volts, 387, 375, 77, 54);
  else if ((volts <= 375) && (volts > 368) )
    return map(volts, 375, 368, 54, 31);
  else if ((volts <= 368) && (volts > 340) )
    return map(volts, 368, 340, 31, 8);
  else if (volts <= 340)
    return map(volts, 340, 260, 8, 0);
}
