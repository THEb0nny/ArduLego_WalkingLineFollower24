// https://alexgyver.ru/gyverpid/
// https://github.com/GyverLibs/TimerMs
// https://github.com/GyverLibs/EncButton

#define PID_OPTIMIZED_I // Параметр для оптимизации суммы регулятора

#include <SoftwareSerial.h>
#include <GyverMotor2.h>
#include "GyverPID.h"
#include <TimerMs.h>
#include <EncButton.h>
#include <GParser.h>

#define PRINT_REF_RAW_LINE_SEN_DEBUG false // Отладка сырых значений с датчиков линии true
#define PRINT_REF_LINE_SEN_DEBUG false // Отладка значений серого faLSE
#define PRINT_DT_ERR_U_DEBUG false // Печать информации о loopTime, error, u TRUE

#define CALIBRATION_BEFORE_START true // Калибровка перед стартом

#define MOT1_DIR_PIN 12 // Пин управляющий направлением вращения мотора 1
#define MOT1_PWM_PIN 5 // Пин правляющий скоростью (ШИМ) мотора 1
#define MOT2_DIR_PIN 13 // Пин управляющий направлением вращения мотора 2
#define MOT2_PWM_PIN 6 // Пин правляющий скоростью (ШИМ) мотора 2

#define RESET_BTN_PIN 3 // Пин кнопки для старта, мягкого перезапуска

#define LINE_SEN_COUNT 4 // Количество датчиков линии

#define LINE_S1_PIN A0 // Пин крайнего левого датчика линии
#define LINE_S2_PIN A1 // Пин центрального левого датчика линии
#define LINE_S3_PIN A2 // Пин центрального правого датчика
#define LINE_S4_PIN A3 // Пин крайнего левого датчика

#define CENTRAL_LINE_SEN_COEFF 1.0 // Коэффициент для центральных датчиков линии
#define SIDE_LINE_SEN_COEFF 3.0 // Коэффицент усиления для крайних датчиков линии

#define MAX_RANGE_VAL_LS 255 // Максимальное значенение диапазона для нормализации значений датчика линии

byte lineSensorPins[LINE_SEN_COUNT] = {LINE_S1_PIN, LINE_S2_PIN, LINE_S3_PIN, LINE_S4_PIN}; // Массив пинов  датчиков линии

int refRawWhite[LINE_SEN_COUNT] = {38, 30, 33, 35}; // Значения белого датчиков линии
int refRawBlack[LINE_SEN_COUNT] = {592, 370, 440, 550}; // Значения чёрного датчиков линии
int refRawValue[LINE_SEN_COUNT] = {0, 0, 0, 0}; // Переменная для хранения сырых значений отражения с датчиков линии
int refValue[LINE_SEN_COUNT] = {0, 0, 0, 0}; // Переменная для хранения нормализованных значений отражения с датчиков линии

unsigned long currTime, prevTime, loopTime; // Время

int speed = 255; // Инициализируем переменную скорости

EncButton btn(RESET_BTN_PIN, INPUT_PULLUP, LOW); // Инициализация объекта простой кнопки
TimerMs regulatorTmr(5); // Инициализация объекта таймера цикла регулирования в нужном количестве мс
GyverPID regulator(3, 0, 0); // Инициализируем регулятор и устанавливаем коэффициенты регулятора

GMotor2<DRIVER2WIRE> motorLeft(MOT1_DIR_PIN, MOT1_PWM_PIN); // Объект левого мотора
GMotor2<DRIVER2WIRE> motorRight(MOT2_DIR_PIN, MOT2_PWM_PIN); // Объект правого мотора

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(115200); // Инициализация скорости общения по монитору порта
  Serial.setTimeout(10); // Позволяет задать время ожидания данных, поступающих через последовательный интерфейс
  Serial.println();
  for (byte i = 0; i < LINE_SEN_COUNT; i++) { // Настойка пина пинов датчиков линии
    pinMode(lineSensorPins[i], INPUT);
  }
  motorLeft.reverse(1); // Направление вращение левого мотора
  motorRight.reverse(0); // Направление вращения правого мотора
  motorLeft.setMinDuty(10); // Минимальный сигнал (по модулю), который будет подан на левый мотор
  motorRight.setMinDuty(10); // Минимальный сигнал (по модулю), который будет подан на левый мотор
  regulator.setDirection(REVERSE); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-255, 255); // Пределы регулирования
  regulatorTmr.setPeriodMode(); // Настроем режим таймера регулирования на период
  while (millis() < 500); // Время после старта для возможности запуска, защита от перезагрузки и старта кода сразу
  if (CALIBRATION_BEFORE_START) {
    Serial.println("Press btn to calibrate");
    PauseUntilBtnPressed("Start calibrate"); // Ждём нажатие кнопки для старта калибровки
    while (true) { // Пока не будет нажата кнопка
      btn.tick(); // Опрашиваем кнопку
      if (btn.press()) break; // Произошло нажатие
      CalibrateLineSensor();
    }
    Serial.println("Calibrate done");
  }
  Serial.println("Ready... press btn to start");
  PauseUntilBtnPressed("Start!"); // Ждём нажатия для старта
  regulatorTmr.start(); // Запускаем таймер цикла регулирования
  // Записываем время перед стартом loop
  currTime = millis();
  prevTime = currTime;
}

void loop() {
  CheckBtnClickToReset(); // Вызываем функцию опроса кнопки
  ParseFromSerialInputValues(true); // Парсинг значений из Serial

  if (regulatorTmr.tick()) { // Раз в N мсек выполнять
    currTime = millis();
    loopTime = currTime - prevTime;
    prevTime = currTime;

    // Считываем сырые значения с датчиков линии
    for (byte i = 0; i < LINE_SEN_COUNT; i++) {
      refRawValue[i] = analogRead(lineSensorPins[i]);
    }

    // Нормализует значения с датчиков линии
    for (byte i = 0; i < LINE_SEN_COUNT; i++) {
      refValue[i] = GetNormalizedRefValuesLineSen(refRawValue[i], refRawBlack[i], refRawWhite[i]);
    }

    CheckBtnClickToReset(); // Повторно вызываем функцию опроса кнопки

    float error = (refValue[1] - refValue[2]) * CENTRAL_LINE_SEN_COEFF + (refValue[0] - refValue[1]) * SIDE_LINE_SEN_COEFF; // Нахождение ошибки регулирования
    regulator.setpoint = error; // Передаём ошибку

    regulator.setDt(loopTime != 0 ? loopTime : 1); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора

    MotorsControl(u, speed); // Для управления моторами регулятором

    CheckBtnClickToReset(); // Повторно вызываем функцию опроса кнопки
    
    // Для отладки значений серого
    if (PRINT_REF_RAW_LINE_SEN_DEBUG) {
      // Serial.println("rawRefLS: " + String(refRawValue[0]) + ", " + String(refRawValue[1]) + ", " + String(refRawValue[2]) + ", " + String(refRawValue[3])); // Вывод сырых значений
      String str = "rawRefLS: ";
      for (byte i = 0; i < LINE_SEN_COUNT; i++) {
        if (i < LINE_SEN_COUNT - 1) str += String(refRawValue[i]) + "\t";
        else str += String(refRawValue[i]);
      }
      Serial.println(str);
    }
    // Для отладки обработанных значений с датчика
    if (PRINT_REF_LINE_SEN_DEBUG) {
      // Serial.println("refLS: " + String(refValue[0]) + ", " + String(refValue[1]) + ", " + String(refValue[2]) + ", " + String(refValue[3])); // Вывод обработанных значений
      String str = "refLS: ";
      for (byte i = 0; i < LINE_SEN_COUNT; i++) {
        if (i < LINE_SEN_COUNT - 1) str += String(refValue[i]) + "\t";
        else str += String(refValue[i]);
      }
      Serial.println(str);
    }
    // Для отладки основной информации о регулировании
    if (PRINT_DT_ERR_U_DEBUG) {
      Serial.println("loopTime: " + String(loopTime) + "\terror: " + String(error) + "\tu: " + String(u));
    }
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lMotorSpeed = speed + dir, rMotorSpeed = speed - dir;
  // float z = (float) speed / max(abs(lMotorSpeed), abs(rMotorSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  // lMotorSpeed *= z, rMotorSpeed *= z;
  motorLeft.setSpeed(lMotorSpeed);
  motorRight.setSpeed(rMotorSpeed);
}

// Калибровка и нормализация значений с датчика линии
int GetNormalizedRefValuesLineSen(int rawRefLineSenVal, int blackRawRefLineS, int whiteRawRefLineS) {
  int lineSensorVal = map(rawRefLineSenVal, blackRawRefLineS, whiteRawRefLineS, 0, MAX_RANGE_VAL_LS);
  lineSensorVal = constrain(lineSensorVal, 0, MAX_RANGE_VAL_LS);
  return lineSensorVal;
}

void CalibrateLineSensor() {
  // Считываем сырые значения с датчиков линии и находим максимальные и минимальные значения
  for (byte i = 0; i < LINE_SEN_COUNT; i++) {
    refRawValue[i] = analogRead(lineSensorPins[i]);
    if (refRawValue[i] > refRawBlack[i]) {
      refRawBlack[i] = refRawValue[i];
    }
    if (refRawValue[i] < refRawWhite[i]) {
      refRawWhite[i] = refRawValue[i];
    }
  }
}

void PauseUntilBtnPressed(String str) {
  while (true) { // Ждём нажатие кнопки для старта
    btn.tick(); // Опрашиваем кнопку
    if (btn.press()) { // Произошло нажатие
      Serial.println(str);
      break;
    }
  }
}

// Функция опроса о нажатии кнопки
void CheckBtnClickToReset() {
  btn.tick(); // Опрашиваем кнопку в первый раз
  if (btn.press()) { // Произошло нажатие
    Serial.println("Btn press and reset");
    delay(50); // Нужна задержка иначе не выведет сообщение
    softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  }
}

// Парсинг значений из Serial
void ParseFromSerialInputValues(bool debug) {
  if (Serial.available() > 2) { // Если что-то прислали
    char inputStr[64]; // Массив символов для записи из Serial
    int amount = Serial.readBytesUntil(';', inputStr, 64); // Считать посимвольно до символа конца пакета точки с запятой и записать количество полученных байт в переменную
    inputStr[amount] = NULL; // Если отправляющее устройство не отправит нулевой символ, то он не запишется в буффер и вывод строк будет некорректным, решение дописать вручную и т.о. закрываем строку
    GParser data(inputStr, ','); // Парсим массив символов по символу запятой
    int am = data.split(); // Получаем количество данных, внимание, ломает строку!
    for (int i = 0; i < am; i++) {
      String tmpStr = data[i];
      tmpStr.replace(" ", ""); // Удалить пробел, если он был введёт по ошибке
      tmpStr.trim(); // Удаление ведущими и конечные пробелы
      char tmpCharArr[tmpStr.length()];
      tmpStr.toCharArray(tmpCharArr, tmpStr.length() + 1);
      if (debug) Serial.println(String(i) + ") " + tmpStr); // Вывести начальную строку
      GParser data2(tmpCharArr, ':'); // Парсим массив символов по символу запятой
      int am2 = data2.split(); // Получаем количество данных, внимание, ломает строку!
      if (am2 > 1) { // Если существует не только ключ, а ещё и значение
        String key = data2[0]; // Ключ - первое значение
        String value = data2[1]; // Значение - второе, или data.getInt(1), чтобы получить целое число
        if (debug) Serial.println("key: " + key + ", value: " + String(value)); // Вывод
        // Присваивание значений
        if (key.equals("kp")) {
          regulator.Kp = value.toFloat();
        } else if (key.equals("ki")) {
          regulator.Ki = value.toFloat();
        } else if (key.equals("kd")) {
          regulator.Kd = value.toFloat();
        } else if (key.equals("speed")) {
          speed = value.toInt();
        }
      }
    }
    if (debug) Serial.println(); // Перевод на новую строку для разделения значений, которые были введены
  }
}