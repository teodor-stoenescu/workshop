/////////////
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>

#include "WiFiConfig.h"


#include "AbstractSensor.h"
#include "DummySensor.h"
#include "MPU9265.h"
#include "HCSR04.h"
#include "MPR121.h"
#include "AnalogSensor.h"
#include "MAX30100Sensor.h"

#define TRIG_PIN 12
#define ECHO_PIN 13


#define GPIN 12
#define BPIN 13
#define RPIN 14

int red = 0;
int green = 0;
int blue = 0;

float accel0 = 0;

const char *buttonNames[12] = {
    "/banana",
    "/pear",
    "/plum",
    "/orange",
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr
};

MPU9265 mpu9265("/accel", "/gyro", "/mag", "/yawpitchroll", "/dist", "/quat");
HCSR04<TRIG_PIN, ECHO_PIN> *hcsr04 = HCSR04<TRIG_PIN, ECHO_PIN>::GetInstance("/dist");
MPR121 mpr121(buttonNames);
AnalogSensor max9814("/mic", A0, 18);
MAX30100Sensor *max30100 = MAX30100Sensor::GetInstance("/pulse", "/beat", MAX30100_PULSE | MAX30100_BEAT);

AbstractSensor *sensors[] = {
    &mpu9265,  // accel
    //hcsr04,    // dist
    //&mpr121,   // touch
    //&max9814,  // analog
    //max30100     // puls
};
const int sensorCount = sizeof(sensors) / sizeof(sensors[0]);
 
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;
 
IPAddress maxHostIp(WIFI_MAX_HOST);
const short int maxHostPort = WIFI_MAX_PORT;

WiFiUDP udp;

void setup() {
    Serial.begin(115200);
    
  pinMode(RPIN, OUTPUT);
  pinMode(GPIN, OUTPUT);
  pinMode(BPIN, OUTPUT);

    delay(100);
    
    // We start by connecting to a WiFi network
    
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    delay(500); 
    
    for (int i = 0; i < sensorCount; ++i) {
        sensors[i]->Init();
    }


}
 
void loop() {
    OSCBundle bundle;

    for (int i = 0; i < sensorCount; ++i) {
        sensors[i]->Sense(&bundle);
    }


    if (random(1) < 0.001) {
    // leds
    float newAccel = bundle.getOSCMessage("/accel")->getFloat(0);
    float activity = abs(accel0 - newAccel);
    activity -= 0.01;
    Serial.println(activity);
    //Serial.println(activity);
    accel0 = newAccel;
    int thresh = 70;

      if (green < 125 && green >= -2) {
        if (((activity * thresh) + green) < 125) {
          green = green + (activity * thresh);
        }
      }
      if (red < 255 && red >= -2) {
        if (((activity * thresh) + red) < 255) {
          red = red + (activity * thresh);
        }
      }
      
      activity *= 0.1;
      if (blue < 125 && blue >= 0) {
        if ((blue + (activity * -thresh)) < 255 && (blue + (activity * -thresh)) >= 0) {
          blue = blue + (activity * -thresh);
          //red = red + (activity * -5);
        }
      }
      analogWrite(BPIN, blue);
      analogWrite(GPIN, green);
      analogWrite(RPIN, red);
  
      Serial.println(red);
      Serial.println(green);
      Serial.println(blue);
      Serial.println("---");
    }

    
    udp.beginPacket(maxHostIp, maxHostPort);
    //Serial.println(bundle.getOSCMessage("/accel")->getFloat(0));
    bundle.send(udp);
    udp.endPacket();

    delay(30); // 18
}
