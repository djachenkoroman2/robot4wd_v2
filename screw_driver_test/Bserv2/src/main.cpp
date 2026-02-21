#include <Arduino.h>
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#define LED 2
BluetoothSerial BT; // Bluetooth object

const int FwdPin_A = 33; 
const int BwdPin_A = 25;  
const int FwdPin_B = 27; 
const int BwdPin_B = 14; 

const int StartPin_A = 32; 
const int StartPin_B = 26;  

int r1=0;
int r2=0;
int l1=0;
int l2=0;
int k1=0;
int s1=0;
int s2=0;

struct Str
{
  int val_b;
  int val_i;
  int val_kn;
};

Str buf;



void callback_function(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_START_EVT) {
    Serial.println("Initialized SPP");
  }
  else if (event == ESP_SPP_SRV_OPEN_EVT ) {
    Serial.println("Client connected");
  }
  else if (event == ESP_SPP_CLOSE_EVT  ) {
    Serial.println("Client disconnected");
  }
  else if (event == ESP_SPP_DATA_IND_EVT ) {
    Serial.println("Data received");
    while (BT.available()) { // As long as there is data to receive
    BT.readBytes((byte *)&buf, sizeof(buf));
    Serial.print("Received left: ");
    Serial.println(buf.val_b);

if (k1 == 0){
 
  s1=buf.val_i;
  s2=buf.val_b;
  k1=1;

  digitalWrite(StartPin_A,HIGH);  
  digitalWrite(StartPin_B,HIGH);
}

if (buf.val_kn==0) { /////////////////////////////////////////////////////////////////////////
  s1=buf.val_i;
  s2=buf.val_b;
}

if (buf.val_i >= s1) {
  r1=map(buf.val_i, s1, 400, 0, 800); // правый двигатель вперед
  analogWrite(BwdPin_A,LOW);  
  analogWrite(FwdPin_A,r1); 
}

if (buf.val_i < s1) {
  r2=map(buf.val_i, 0, s1-1, 800, 0); // правый двигатель назад
  analogWrite(FwdPin_A,LOW);
  analogWrite(BwdPin_A,r2);   
}

if (buf.val_b >= s2) {
  l1=map(buf.val_b, s2, 400, 0, 800); // левый двигатель вперед
  analogWrite(BwdPin_B,LOW);  
  analogWrite(FwdPin_B,l1); 
}

if (buf.val_b < s2) {
  l2=map(buf.val_b, 0, s2-1, 800, 0); // левый двигатель назад
  analogWrite(FwdPin_B,LOW);
  analogWrite(BwdPin_B,l2);   
}



    Serial.print("Received rigt: ");
    Serial.println(buf.val_i);
    }
  }
}
void setup() {
  pinMode(FwdPin_A, OUTPUT);    // Устанавливаем FwdPin_A как выход
  pinMode(BwdPin_A, OUTPUT);    // Устанавливаем BwdPin_A как выход
  pinMode(FwdPin_B, OUTPUT);    // Устанавливаем FwdPin_B как выход
  pinMode(BwdPin_B, OUTPUT);    // Устанавливаем BwdPin_B как выход

  pinMode(StartPin_A, OUTPUT);    // Устанавливаем StartPin_A как выход
  pinMode(StartPin_B, OUTPUT);    // Устанавливаем StartPin_B как выход

  Serial.begin(9600); // Initializing the serial connection for debugging
  BT.begin("ESP32_LED_Control"); // Name of your Bluetooth Device and in slave mode
  Serial.println("Bluetooth device is ready to pair");
  BT.register_callback(callback_function); // We register the "callback_function" function as a callback function.
  pinMode (LED, OUTPUT); // Change the PIN of the led to OUTPUT
}
void loop() {

}
