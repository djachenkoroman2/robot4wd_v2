#include <Arduino.h>
#include"pins.h"
#include <uPID.h>


#define SERIALSPEED 9600
#define MESURE_INTERVAL 100
#define PULSE_PER_REVOLUTION 20

// Переменные для измерения скорости
volatile unsigned int pulses = 0;
unsigned long lastTime = 0;
unsigned int rpm = 0;

float currentRPM = 0;
// ПИД выход
int pwmOutput = 0;

// Функция прерывания для подсчета импульсов
void countPulses() {
  pulses++;
};

float timeInMinutes = (MESURE_INTERVAL / 1000.0) / 60.0;

uPID pid(P_INPUT | I_SATURATE);

void setup() {
  
  pinMode( PIN_MOTORDRV1_ENA, OUTPUT );
  pinMode( PIN_MOTORDRV1_IN1, OUTPUT );
  pinMode( PIN_MOTORDRV1_IN2, OUTPUT );
  pinMode(PIN_SPD_SENSOR_1, INPUT_PULLUP);
  Serial.begin(SERIALSPEED); // Увеличим скорость для отладки
  attachInterrupt(digitalPinToInterrupt(PIN_SPD_SENSOR_1), countPulses, RISING);

  pid.setKp(0.5);
  pid.setKi(0.01);
  pid.setKd(0.01);

  // pid.Kbc = 0.1;
  pid.setDt(MESURE_INTERVAL);
  pid.outMax = 255;
  pid.outMin = 0;

  pid.setpoint = 90;

// Инициализация
  lastTime = millis();
  
  Serial.println("Система регулирования скорости двигателя");
  Serial.println("========================================");


}

void loop() {
  // Измерение скорости каждые measureInterval миллисекунд
  if (millis() - lastTime >= MESURE_INTERVAL) {
    // Отключить прерывания на время расчетов
    noInterrupts();
    unsigned int pulseCount = pulses;
    pulses = 0;
    interrupts();
    
    // Расчет RPM
    // Формула: RPM = (импульсы / время в минутах) / импульсы_на_оборот
    // время в минутах = (интервал в мс / 1000) / 60
    
    currentRPM = (pulseCount / timeInMinutes) / PULSE_PER_REVOLUTION;
    
    
    // Преобразование в ШИМ значение (0-255)
    pwmOutput += pid.compute(currentRPM);
    
    // Ограничение выхода ШИМ
    if (pwmOutput > 255) pwmOutput = 255;
    if (pwmOutput < 0) pwmOutput = 0;
    
    // Применение ШИМ к двигателю
    analogWrite( PIN_MOTORDRV1_ENA, pwmOutput);
    digitalWrite( PIN_MOTORDRV1_IN1, HIGH );
    digitalWrite( PIN_MOTORDRV1_IN2, LOW );

    lastTime = millis();
    
    // Вывод информации в монитор порта
    Serial.print("Цель: ");
    Serial.print(pid.setpoint);
    Serial.print(" RPM | Текущее: ");
    Serial.print(currentRPM);
    Serial.print(" RPM | ШИМ: ");
    Serial.println(pwmOutput);
  }
  
  // Проверка ввода с последовательного порта для изменения целевой скорости
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    pwmOutput=0;
    float newvalue = input.toFloat();
    pid.setpoint = newvalue;
    Serial.print("Новая целевая скорость: ");
    Serial.print(pid.setpoint);
    Serial.println(" RPM");
  }
}
