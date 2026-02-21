#ifndef MOTOR_H
#define MOTOR_H

#define CMD_FORWARD_A 1 
#define CMD_FORWARD_B 2 
#define CMD_FORWARD_AB 3 
#define CMD_RIGHT 4 
#define CMD_LEFT 5 
#define CMD_FORWARD_AB2 6 
#define CMD_RIGHT2 7 
#define CMD_LEFT2 8 

#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 6
#define IN3 7
#define IN4 8

void SetupMotor();

void Forward();
void Backward();
void Stop();
void Left();
void Right();
void ForwardA();
void StopA();
void ForwardB();
void StopB();
void BackwardA();
void BackwardB();
void ForwardA(int s);
void ForwardB(int s);

#endif