#include <Arduino.h>
#include "BluetoothSerial.h"
#include <GyverMotor2.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define CMD_FORWARD 1
#define CMD_LEFT 4
#define CMD_RIGHT 2
#define CMD_BACKWARD 3
#define CMD_STOP 5

#define SPD 60
const int FwdPin_B = 27; 
const int BwdPin_B = 14; 

const int FwdPin_A = 26; 
const int BwdPin_A = 25; 

BluetoothSerial BT; // Bluetooth object
GMotor2<DRIVER2WIRE> motorB(FwdPin_B, BwdPin_B);
GMotor2<DRIVER2WIRE> motorA(FwdPin_A, BwdPin_A);

struct Str
{
  uint8_t cmd;
  uint8_t val;
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
      Serial.print(buf.cmd);
      Serial.print(" ");
      Serial.println(buf.val);

      if(buf.cmd==CMD_FORWARD)
      {
        motorA.setSpeed(buf.val);
        motorB.setSpeed(buf.val);
      }
      if(buf.cmd==CMD_RIGHT)
      {
        motorA.setSpeed(buf.val);
        motorB.setSpeed(-buf.val);
      }
      if(buf.cmd==CMD_LEFT)
      {
        motorA.setSpeed(-buf.val);
        motorB.setSpeed(buf.val);
      }
      if(buf.cmd==CMD_BACKWARD)
      {
        motorA.setSpeed(-buf.val);
        motorB.setSpeed(-buf.val);
      }
      if(buf.cmd==CMD_STOP)
      {
        motorA.brake();
        motorB.brake();
      }
    }
  }
}

void setup() {

  motorB.setMinDuty(70);
  motorA.setMinDuty(70);
  
  pinMode(FwdPin_B, OUTPUT);   
  pinMode(BwdPin_B, OUTPUT);   

  pinMode(FwdPin_A, OUTPUT);   
  pinMode(BwdPin_A, OUTPUT);   
 
  Serial.begin(9600); // Initializing the serial connection for debugging
  BT.begin("ESP32_ROBOT"); // Name of your Bluetooth Device and in slave mode
  Serial.println("Bluetooth device is ready to pair");
  BT.register_callback(callback_function); // We register the "callback_function" function as a callback function.

}
void loop() {
}