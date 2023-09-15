//Testing the Time of flight sensor


#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include "basic.h"
#include <WiFi.h>

//Create time of flight sensor object
Adafruit_VL53L0X toF = Adafruit_VL53L0X();


void setup() {

}


void loop() {
    uint32_t getDistStart = timerReadMicros(timer1);

    uint8_t dist = getDist(toF);    //Get a distance measurement from the Tof sensor
    
    uint32_t getDistEnd = timerReadMicros(timer1);
    Serial.print("Dist measurement micros: ");
    Serial.println(getDistEnd - getDistStart);
}