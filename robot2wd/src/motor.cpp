#include <Arduino.h>
#include "motor.h"

int Speed = 150;

void SetupMotor()
{
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void Forward()
{
    analogWrite(ENA, Speed);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);

    analogWrite(ENB, Speed);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
}

void Backward()
{
    analogWrite(ENA, Speed);
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);

    analogWrite(ENB, Speed);
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
}
void BackwardA()
{
    analogWrite(ENA, Speed);
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
}

void BackwardB()
{
    analogWrite(ENB, Speed);
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
}

void ForwardA()
{
    analogWrite(ENA, Speed);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
}

void ForwardA(int s)
{
    analogWrite(ENA, s);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
}

void ForwardB()
{
    analogWrite(ENB, Speed);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
}

void ForwardB(int s)
{
    analogWrite(ENB, s);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
}

void Stop()
{
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void StopA()
{
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, LOW);
}

void StopB()
{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void Right()
{
    ForwardA();
    BackwardB();
}

void Left()
{
    ForwardB();
    BackwardA();
}
