#include <Arduino.h>
#include"pins.h"
#include"abstract_filter.h"
#include"simple_kalman_filter.h"
#include"ma_filter.h"


#define SERIALSPEED 9600
#define DT 400
#define DT2 50
#define SPD_TARGET 13

volatile unsigned int rot = 0;
unsigned long speedRPM = 0;
unsigned long lastCalcTime = 0;
unsigned long lastCalcTime2 = 0;
unsigned long currentTime = 0;
unsigned long currentTime2 = 0;

unsigned char val = 150;



// SimpleKalmanFilter kalmanFilter(0.01, 0.7, 0); // Q=0.01, R=0.1
MovingAverageFilter MAFilter(10); 

AbstractFilter& filter1=MAFilter; 
// AbstractFilter& filter2=kalmanFilter; 

float Kp, Ki, Kd;
float integral, pError;


float computePID(float setpoint, float input, float dt) {
    float error = setpoint - input;
    float derivative = (error - pError) / dt;
    pError = error;
    integral += error * dt;
    // return error * Kp + integral * Ki + derivative * Kd;
    return error * Kp + integral * Ki;
}


void detect() {
    rot++;
}

void setup() {

    pinMode( PIN_MOTORDRV1_ENA, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN1, OUTPUT );
    pinMode( PIN_MOTORDRV1_IN2, OUTPUT );
    pinMode(PIN_SPD_SENSOR_1, INPUT_PULLUP);

    Serial.begin(SERIALSPEED); // Увеличим скорость для отладки

    attachInterrupt(digitalPinToInterrupt(PIN_SPD_SENSOR_1), detect, RISING);


    Kp = 20;
    Ki = 5;
    Kd = 0;
    pError = 0;
    integral = 0;

    lastCalcTime2=lastCalcTime = millis();
}

void loop() {

    // Быстрая проверка ввода (может создавать артефакты на графике)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '0') val=70;
        if (c == '1') val=90;
        if (c == '2') val=110; 
        if (c == '3') val=130;
        if (c == '4') val=150;
        if (c == '5') val=170; 
        if (c == '6') val=190;
        if (c == '7') val=210;
        if (c == '8') val=230; 
        if (c == '9') val=255; 
    }


    currentTime = millis();
    if (currentTime - lastCalcTime >= DT) 
    {
        // Отключаем прерывания на время чтения/сброса переменной
        noInterrupts();
        unsigned int rotations = rot;
        rot = 0;
        interrupts();
        
        // Вычисляем RPM
        if (lastCalcTime > 0) {
            unsigned long timeDiff = currentTime - lastCalcTime;
            if (timeDiff > 0 && rotations > 0) {
                // filteredSpeed = filter2.update(filter1.update(rotations / timeDiff));
                speedRPM = int(filter1.update(rotations));
                
            } else {
                speedRPM = 0;
            }
        }
        
        lastCalcTime = currentTime;
    }

    currentTime2 = millis();
    if (currentTime2 - lastCalcTime2 >= DT2)
    {
        float output = computePID(SPD_TARGET, speedRPM, DT2/1000);
        // int val = map(speedRPM+output,0,15,0,255);
        analogWrite( PIN_MOTORDRV1_ENA, val+output);
        digitalWrite( PIN_MOTORDRV1_IN1, HIGH );
        digitalWrite( PIN_MOTORDRV1_IN2, LOW );

        Serial.print(speedRPM);
        Serial.print(" ");
        Serial.println(SPD_TARGET);
        // Serial.print(" ");
        // Serial.println(val+output, 1); 
        lastCalcTime2 = currentTime2;
    }
    delay(1);
}
