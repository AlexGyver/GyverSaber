/*
   Супер-мего-крутой световой меч на Ардуине и адресных светодиодах!
   Железо:
     Используем адресные светодиоды получить любой  цвет, а также плавное включение
     Используем SD карту, чтобы воспроизводить звуки
     Используем IMU (аксель + гироскоп), чтобы генерировать гудящий звук в зависимости от взмахов
   Возможности:
     Плавное включение/выключение со звуками меча
     Тон "гудения" зависит от угловой скорости (гироскоп) поворота меча, т.е. взмаха
     Акселерометр отрабатывает удары, воспроизводится один из 16 звуков удара (случайно)
     Также при ударе меч вспыхивает ярко-белым
     Включение/выключение по удерживанию кнопки
     Тройное нажатие - смена цвета
     Выбранный цвет хранится в памяти и не сбрасывается при перезагрузке
     При включении показывает уровень заряда аккумулятора
*/

// ------------- НАСТРОЙКИ -------------
#define NUM_LEDS 30         // число МИКРОСХЕМ на ленте
#define BTN_TIMEOUT 800     // задержка кнопки для удерживания (миллисекунды)
#define FLASH_DELAY 70      // время вспышки при ударе (миллисекунды)
#define BRIGHTNESS 255      // яркость ленты
#define STRIKE_THR 300      // порог ускорения для распознавания удара
#define STRIKE_S_THR 450    // порог ускорения для распознавания сильного удара
#define R1 100000
#define R2 51000
// ------------- НАСТРОЙКИ -------------

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


// ----------------- ПЕРЕМЕННЫЕ -----------------
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long ACC, GYR, COMPL;
int gyroX, gyroY, gyroZ, accelX, accelY, accelZ, freq, freq_f = 20;
float k = 0.2;
unsigned long toneTimer, mpuTimer, nowTimer;
int stopTimer;
boolean bzzz_flag, ls_chg_state, ls_state;
boolean btnState, btn_flag, hold_flag;
byte btn_counter;
unsigned long btn_timer;
byte nowStrike;
byte LEDcolor;  // 0 - красный, 1 - синий, 2 - зелёный, 3 - розовый, 4 - жёлтый
byte nowColor, red, green, blue;
boolean eeprom_flag;
float voltage;
// ----------------- ПЕРЕМЕННЫЕ -----------------

// --------------------------------- ЗВУКИ УДАРОВ ---------------------------------
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

char BUFFER[15];
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

  // инициализация и настройка IMU
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  if ((EEPROM.read(0) >= 0) && (EEPROM.read(0) <= 5)) {  // если был хоть один запуск прошивки
    nowColor = EEPROM.read(0);   // вспоминаем из памяти выбранный цвет
  } else {                       // если это первый запуск
    EEPROM.write(0, 0);          // обнуляем ячейку
    nowColor = 0;                // цвет нулевой
  }
  setColor(nowColor);            // устанавливаем цвет

  byte capacity = voltage_measure();   // получить процент заряда аккумулятора
  capacity = map(capacity, 100, 0, (NUM_LEDS / 2 - 1), 1);  // перевести в длину клинка

  for (char i = 0; i <= capacity; i++) {          // включить все диоды выбранным цветом
    setPixel(i, red, blue, green);
    setPixel((NUM_LEDS - 1 - i), red, blue, green);
    FastLED.show();
    delay(25);
  }
  delay(1000);                 // секунда для отображения заряда акума
  setAll(0, 0, 0);             // показываем чёрный цвет ленты
  FastLED.setBrightness(BRIGHTNESS); // яркость
}

void loop() {
  getFreq();          // получить частоту для трещалки
  on_off_sound();     // блок вкл/выкл меча со звуками
  btnTick();          // опрос и отработка кнопки
  strikeTick();       // отработка удара
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
    if (btn_counter == 3) {               // если число нажатий равно 3
      nowColor++;                         // сменить цвет
      if (nowColor >= 6) nowColor = 0;    // закольцевать смену цвета
      setColor(nowColor);                 // установить цвет
      setAll(red, blue, green);           // включить цвет
      eeprom_flag = 1;                    // разрешить запись выбранного цвета в память
    }
    btn_counter = 0;
  }
}

void on_off_sound() {                // блок вкл/выкл меча со звуками
  if (ls_chg_state) {                // если есть запрос на изменение состояния меча
    if (!ls_state) {                 // если меч выключен
      tmrpcm.play("ON.wav");         // воспроизвести звук включения
      delay(200);                    // ждём воспроизведение
      light_up();                    // лента включается
      delay(200);                    // ждём воспроизведение
      tmrpcm.disable();              // выключаем звук
      toneAC(freq_f);                // трещать
      bzzz_flag = 1;                 // разрешаем трещалку
      ls_state = true;               // запомнить, что меч включен
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
      }
    }
    ls_chg_state = 0;                // снять флаг запроса смены состояния
  }

  if ((millis() - toneTimer > 3) && (bzzz_flag)) {   // если настало время трещать и разрешено трещать
    toneAC(freq_f);                                  // трещать
    toneTimer = millis();                            // сбросить таймер
  }
}

void strikeTick() {
  if ((ACC > STRIKE_THR) && (ACC < STRIKE_S_THR)) {      // если ускорение превысило порог
    noToneAC();                        // выключить трещалку
    nowStrike = random(8);             // взять случайное число
    // читаем название трека из PROGMEM
    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes_short[nowStrike])));
    tmrpcm.play(BUFFER);               // воспроизвести звук удара
    strike_flash();
    delay(strike_s_time[nowStrike] - FLASH_DELAY); // ждать
    tmrpcm.disable();                  // выключить звук
    toneAC(freq_f);                    // включить трещалку
  }
  if (ACC >= STRIKE_S_THR) {           // если ускорение превысило порог
    noToneAC();                        // выключить трещалку
    nowStrike = random(8);             // взять случайное число
    // читаем название трека из PROGMEM
    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes[nowStrike])));
    tmrpcm.play(BUFFER);               // воспроизвести звук удара
    strike_flash();
    delay(strike_time[nowStrike] - FLASH_DELAY);   // ждать
    tmrpcm.disable();                  // выключить звук
    toneAC(freq_f);                    // включить трещалку
  }
}

void getFreq() {
  if (ls_state) {                                               // если меч включен
    if (millis() - mpuTimer > 500) {                            // каждые полмиллисекунды
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       // получить показания с IMU
      // найти абсолютное значение, разделить на 100
      gyroX = abs(gx / 100);
      // gyroY = abs(gy / 100);
      gyroZ = abs(gz / 100);
      accelX = abs(ax / 100);
      accelY = abs(ay / 100);
      accelZ = abs(az / 100);

      ACC = (sq((long)accelX) + sq((long)accelY) + sq((long)accelZ));
      ACC = sqrt(ACC);
      GYR = (sq((long)gyroX) + sq((long)gyroZ));
      GYR = sqrt((long)GYR);
      COMPL = ACC + GYR;                                        // сложить показания
      /*
         Serial.print("$");
         Serial.print(ACC);
         Serial.println(" ");
         Serial.println(GYR);
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
    setPixel(i, red, blue, green);
    setPixel((NUM_LEDS - 1 - i), red, blue, green);
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

