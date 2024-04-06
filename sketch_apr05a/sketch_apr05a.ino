// https://www.youtube.com/watch?time_continue=1&v=fG4Vc6EBjkM&feature=emb_logo
// https://alexgyver.ru/gyverpid/
// https://github.com/GyverLibs/TimerMs
// https://github.com/GyverLibs/EncButton

#define PID_OPTIMIZED_I // Параметр для оптимизации суммы регулятора

#include <SoftwareSerial.h>
#include <GyverMotor2.h>
#include "GyverPID.h"
#include <TimerMs.h>
#include <EncButton.h>

#define PRINT_REF_RAW_LINE_SEN_DEBUG false // Отладка сырых значений с датчиков линии true
#define PRINT_REF_LINE_SEN_DEBUG false // Отладка значений серого faLSE
#define PRINT_DT_ERR_U_DEBUG false // Печать информации о loopTime, error, u TRUE

#define MOTORS_CONTROL_FUNC_DEBUG false // Отдалка функции MotorsControl
#define MOTOR_SPEED_FUNC_DEBUG false // Отдалка функции MotorsControl

#define RESET_BTN_PIN 3 // Пин кнопки для старта, мягкого перезапуска

#define SERVO_L1_PIN 10 // Пин левого первого серво мотора
#define SERVO_L2_PIN 8 // Пин левого второго серво мотора
#define SERVO_R1_PIN 9 // Пин правого серво мотора
#define SERVO_R2_PIN 7 // Пин правого серво мотора

#define LINE_S1_PIN A0 // Пин крайнего левого датчика линии
#define LINE_S2_PIN A1 // Пин центрального левого датчика линии
#define LINE_S3_PIN A2 // Пин центрального правого датчика
#define LINE_S4_PIN A3 // Пин крайнего левого датчика

#define RAW_REF_WHITE_LINE_S1 30 // Значение белого крайнего левого датчика линии
#define RAW_REF_WHITE_LINE_S2 36 // Значение белого правого датчика линии
#define RAW_REF_WHITE_LINE_S3 34 // Значение белого левого датчика линии
#define RAW_REF_WHITE_LINE_S4 33 // Значение белого крайнего правого датчика линии

#define RAW_REF_BLACK_LINE_S1 300 // Значение чёрного крайнего левого датчика линии
#define RAW_REF_BLACK_LINE_S2 500 // Значение чёрного центральнего левого датчика линии
#define RAW_REF_BLACK_LINE_S3 470 // Значение чёрного правого датчика линии
#define RAW_REF_BLACK_LINE_S4 450 // Значение чёрного крайнего правого датчика линии

#define COEFF_CENTRAL_LINE_SEN 1 // Коэффициент для центральных датчиков линии
#define COEFF_SIDE_LINE_SEN 3.0 // Коэффицент усиления для крайних датчиков линии

unsigned long currTime, prevTime, loopTime; // Время

float Kp = 1, Ki = 0, Kd = 0; // Коэффиценты регулятора
int speed = 255; // Инициализируем переменную скорости

EncButton<EB_TICK, RESET_BTN_PIN> btn; // Инициализация объекта простой кнопки
TimerMs regulatorTmr(10); // Инициализация объекта таймера цикла регулирования
GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем регулятор и устанавливаем коэффициенты регулятора

GMotor2<DRIVER2WIRE> motorLeft(12, 5);
GMotor2<DRIVER2WIRE> motorRight(13, 6);

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println();
  pinMode(LINE_S1_PIN, INPUT); // Настойка пина пинов датчиков линии
  pinMode(LINE_S2_PIN, INPUT);
  pinMode(LINE_S3_PIN, INPUT);
  pinMode(LINE_S4_PIN, INPUT);
  motorLeft.reverse(1);
  motorRight.reverse(0);
  motorLeft.setMinDuty(10);
  motorRight.setMinDuty(10);
  regulator.setDirection(REVERSE); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-200, 200); // Пределы регулятора
  regulatorTmr.setPeriodMode(); // Настроем режим условия регулирования на период
  while (millis() < 500); // Время после старта для возможности запуска, защита от перезагрузки и старта кода сразу
  Serial.println("Ready... press btn");
  while (true) { // Ждём нажатие кнопки для старта
    btn.tick(); // Опрашиваем кнопку
    if (btn.press()) { // Произошло нажатие
      Serial.println("Go!!!");
      break;
    }
  }
  regulatorTmr.start(); // Запускаем таймер цикла регулирования
  // Записываем время перед стартом loop
  currTime = millis();
  prevTime = currTime;
}

void loop() {
  CheckBtnClick(); // Вызываем функцию опроса кнопки
  ParseSerialInputValues(); // Парсинг значений из Serial

  if (regulatorTmr.tick()) { // Раз в 10 мсек выполнять
    currTime = millis();
    loopTime = currTime - prevTime;
    prevTime = currTime;
    // Считываем сырые значения с датчиков линии
    int rawRefLineS1 = analogRead(LINE_S1_PIN);
    int rawRefLineS2 = analogRead(LINE_S2_PIN);
    int rawRefLineS3 = analogRead(LINE_S3_PIN);
    int rawRefLineS4 = analogRead(LINE_S4_PIN);
    // Калибруем/обрабатываем значения с датчиков линии
    int refLineS1 = GetCalibValColorS(rawRefLineS1, RAW_REF_BLACK_LINE_S1, RAW_REF_WHITE_LINE_S1);
    int refLineS2 = GetCalibValColorS(rawRefLineS2, RAW_REF_BLACK_LINE_S2, RAW_REF_WHITE_LINE_S2);
    int refLineS3 = GetCalibValColorS(rawRefLineS3, RAW_REF_BLACK_LINE_S3, RAW_REF_WHITE_LINE_S3);
    int refLineS4 = GetCalibValColorS(rawRefLineS4, RAW_REF_BLACK_LINE_S4, RAW_REF_WHITE_LINE_S4);
    CheckBtnClick(); // Повторно вызываем функцию опроса кнопки
    float error = CalcLineSensorsError(1, refLineS1, refLineS2, refLineS3, refLineS4); // Нахождение ошибки
    regulator.setpoint = error; // Передаём ошибку

    if (regulator.Kp != Kp) regulator.Kp = Kp; // Установка значений Kp, если они были изменены
    if (regulator.Ki != Ki) regulator.Ki = Ki; // Установка значений Ki, если они были изменены
    if (regulator.Kd != Kd) regulator.Kd = Kd; // Установка значений Kd, если они были изменены

    regulator.setDt(loopTime != 0 ? loopTime : 1); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора

    MotorsControl(u, speed); // Для управления моторами регулятором

    CheckBtnClick(); // Повторно вызываем функцию опроса кнопки
    
    // Для отладки значений серого
    if (PRINT_REF_RAW_LINE_SEN_DEBUG) {
      Serial.println("rawRefLS: " + String(rawRefLineS1) + ", " + String(rawRefLineS2) + ", " + String(rawRefLineS3) + ", " + String(rawRefLineS4)); // Вывод сырых значений
    }
    // Для отладки обработанных значений с датчика
    if (PRINT_REF_LINE_SEN_DEBUG) {
      Serial.println("refLS: " + String(refLineS1) + ", " + String(refLineS2) + ", " + String(refLineS3) + ", " + String(refLineS4)); // Вывод обработанных значений
    }
    // Для отладки основной информации о регулировании
    if (PRINT_DT_ERR_U_DEBUG) {
      Serial.print("loopTime: " + String(loopTime) + "\t");
      Serial.print("error: " + String(error) + "\t");
      Serial.println("u: " + String(u));
    }
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lMotorSpeed = speed + dir, rMotorSpeed = speed - dir;
  float z = (float) speed / max(abs(lMotorSpeed), abs(rMotorSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lMotorSpeed *= z, rMotorSpeed *= z;
  motorLeft.setSpeed(lMotorSpeed);
  motorRight.setSpeed(rMotorSpeed);
}

// Калибровка и нормализация значений с датчика линии
int GetCalibValColorS(int rawRefLineSenVal, int blackRawRefLineS, int whiteRawRefLineS) {
  int lineSensorVal = map(rawRefLineSenVal, blackRawRefLineS, whiteRawRefLineS, 0, 100);
  lineSensorVal = constrain(lineSensorVal, 0, 100);
  return lineSensorVal;
}

// Посчитать ошибку регулирования
float CalcLineSensorsError(byte calcMetod, int sLeftLineSenRefVal, int cLeftLineSenRefVal, int cRightLineSenRefVal, int sRightLineSenRefVal) {
  float error = 0;
  if (calcMetod == 0) error = cLeftLineSenRefVal - cRightLineSenRefVal;
  else if (calcMetod == 1) error = (COEFF_SIDE_LINE_SEN * sLeftLineSenRefVal + COEFF_CENTRAL_LINE_SEN * cLeftLineSenRefVal) - (cRightLineSenRefVal * COEFF_CENTRAL_LINE_SEN + sRightLineSenRefVal * COEFF_SIDE_LINE_SEN);
  return error;
}

// Парсинг значений из Serial
void ParseSerialInputValues() {
    if (Serial.available() > 2) {
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    inputStr.replace(" ", ""); // Убрать возможные пробелы между символами
    byte strIndex = inputStr.length(); // Переменая для хронения индекса вхождения цифры в входной строке, изначально равна размеру строки
    for (byte i = 0; i < 10; i++) { // Поиск первого вхождения цифры от 0 по 9 в подстроку
      byte index = inputStr.indexOf(String(i)); // Узнаём индекс, где нашли цифру параметра цикла
      if (index < strIndex && index != 255) strIndex = index;  // Если индекс цифры меньше strIndex, то обновляем strIndex 
    }
    String key = inputStr.substring(0, strIndex); // Записываем ключ с начала строки до первой цицры
    float value = inputStr.substring(strIndex, inputStr.length()).toFloat(); // Записываем значение с начала цифры до конца строки
    if (key == "p") {
      regulator.Kp = value;
    } else if (key == "i") {
      regulator.Ki = value;
      regulator.integral = 0;
    } else if (key == "d") {
      regulator.Kd = value;
    } else if (key == "s") {
      speed = value;
    }
    Serial.println(key + " = " + String(value)); // Печать информации о ключе и значении
  }
}

// Функция опроса о нажатии кнопки
void CheckBtnClick() {
  btn.tick(); // Опрашиваем кнопку в первый раз
  if (btn.press()) { // Произошло нажатие
    Serial.println("Btn press and reset");
    delay(50); // Нужна задержка иначе не выведет сообщение
    softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  }
}
