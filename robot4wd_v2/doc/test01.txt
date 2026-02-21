#include <Arduino.h>

#define SPD 60
const int FwdPin_B = 27; 
const int BwdPin_B = 14; 

const int FwdPin_A = 26; 
const int BwdPin_A = 25; 


void setup() {
  pinMode(FwdPin_B, OUTPUT);   
  pinMode(BwdPin_B, OUTPUT);   

  pinMode(FwdPin_A, OUTPUT);   
  pinMode(BwdPin_A, OUTPUT);   
 
  
}
void loop() {

  analogWrite(FwdPin_B, SPD); 
  digitalWrite(BwdPin_B, LOW);

  analogWrite(FwdPin_A, SPD); 
  digitalWrite(BwdPin_A, LOW);

  
  delay(3000);
  
  analogWrite(FwdPin_B, 0);
  digitalWrite(BwdPin_B, LOW);

  analogWrite(FwdPin_A, 0);
  digitalWrite(BwdPin_A, LOW);

  
  delay(1000);
  
  digitalWrite(FwdPin_B, LOW);
  analogWrite(BwdPin_B, SPD);
  
  digitalWrite(FwdPin_A, LOW);
  analogWrite(BwdPin_A, SPD);
  
  delay(3000);
  
  analogWrite(BwdPin_B, 0);
  digitalWrite(BwdPin_B, LOW);

  analogWrite(BwdPin_A, 0);
  digitalWrite(BwdPin_A, LOW);

  
  delay(1000);
}