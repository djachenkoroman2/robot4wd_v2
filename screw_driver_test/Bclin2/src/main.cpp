#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothSerial.h" // We will include the Serial Bluetooth header
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial BT; // Bluetooth Object
String serverName = "ESP32_LED_Control";
bool connected;
#define left_PIN  36 // ESP32 pin GPIO36 (ADC0) connected to VRX pin
#define right_PIN  39 // ESP32 pin GPIO36 (ADC0) connected to VRX pin

#define TIME_INTERVAL 50 * 1
unsigned long Timer1 = 0;

#define TIME_INTERVAL2 500 * 1
unsigned long Timer2 = 0;


#define OLED_MOSI 23
#define OLED_CLK 18
#define OLED_DC 16
#define OLED_CS 5
#define OLED_RESET 17
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

struct Str
{
  int val_b; //левый
  int val_i; // правый
  int val_kn;   // кнопки.
};

Str buf;

int disL;
int disR;

void setup() {

display.begin(SSD1306_SWITCHCAPVCC);
 display.clearDisplay();
 display.display();
 display.setTextSize(1); // установка размера шрифта
 display.setTextColor(WHITE); // установка цвета текста
 display.setCursor(22,5);
  display.println("left");
  display.display();
  display.setCursor(70,5);
  display.println("right");
  display.display();

  pinMode (15, OUTPUT);
  pinMode (4, OUTPUT);

  digitalWrite(15, HIGH);
  digitalWrite(4, LOW);

  pinMode (34, INPUT);

  Serial.begin(9600); // Initializing serial connection for debugging
  BT.begin("ESP32_client", true); // Name of your Bluetooth Device and in master mode
  Serial.println("Bluetooth Device is in master mode. Connecting to the host...");
  connected = BT.connect(serverName);
  if(connected) {
    Serial.println("Connected Succesfully!");
    digitalWrite(15, LOW);
    digitalWrite(4, HIGH);
  } else {
    while(!BT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
}
void loop() {
 // buf.val_kn=analogRead(34);
 //Serial.println(buf.val_kn);
buf.val_b=analogRead(left_PIN);
if (millis() - Timer1 >= TIME_INTERVAL)
  {
  buf.val_b=analogRead(left_PIN);

if (analogRead(34) >= 100 && analogRead(34) < 250){
  buf.val_kn=0;
}
else {
  buf.val_kn=10;
}
  buf.val_i=analogRead(right_PIN);
  buf.val_b = map(buf.val_b, 0, 4095, 400, 0);
  disL=buf.val_b;
  //Serial.println(buf.val_b);
  buf.val_i=map(buf.val_i, 0, 4095, 400, 0);
  disR=buf.val_i;
  //Serial.println(buf.val_i);
  BT.write((byte*)&buf, sizeof(buf));
 Timer1 = millis();
  }

  if (millis() - Timer2 >= TIME_INTERVAL2)
  {

display.clearDisplay();
display.display();
display.setCursor(22,5);
display.println("left");
display.display();
display.setCursor(70,5);
display.println("right");
display.display();
display.setCursor(27,15);
display.println(disL);
display.display();
display.setCursor(75,15);
display.println(disR);
display.display();
 Timer2 = millis();
  }

}
