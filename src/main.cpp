#include <Arduino.h>
#include <WiFi.h>
#include "basic.h"
#include <Wire.h>
#include <stdlib.h>
#include "secrets.h"

//Globals
uint8_t I2CPort = 0;
uint8_t arrayCount = 0;
//int16_t XHiReg = 0x03;
uint8_t dataCount = 0;

//int16_t Acc1Avg[3];   //XYZ vector

const char* ssid = NETWORK;
const char* password = PASS;
WiFiServer wifiServer(80);
WiFiClient client;
int16_t socketTestData = 4040;

char bytes[18];
accVector Acc1Vectors[accPacketSize];

hw_timer_t * timer1 = NULL;
uint32_t AccPacketStart;
uint32_t AccPacketStartMicro;
uint32_t AccVectorEnd = 0;
uint32_t AccVectorEndMicro = 0;

//Functions

void setup() {
  
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.begin(115200);
  #ifdef DEBUG
    Serial.println("I am alive!");
  #endif /*DEBUG*/

  WiFi.begin(ssid, password);
  uint8_t wifiAttempts = 0;
  while(WiFi.status() != WL_CONNECTED)  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    wifiAttempts++;
    //Serial.println(wifiAttempts, DEC);
      //Reset ESP32 after 12 failed connection attempts
        if (wifiAttempts > 12) {
        ESP.restart();
      }
    }
  Serial.println("Connected to the WiFi Network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();

  //Start the timer
  timer1 = timerBegin(0, 10, true);
  timerStart(timer1);

  #ifdef DEBUG
    Serial.print("Core: ");
    Serial.println(xPortGetCoreID());
  #endif /*DEBUG*/
}

void loop() {

  client = wifiServer.available();
 
  if (client) {
    while (client.connected()) {
      while (client.available() > 0) {
        uint8_t byteCode = client.read();

        #ifdef DEBUG
          Serial.print("byteCode: ");
          Serial.println(byteCode, HEX);
        #endif /*DEBUG*/

        if (byteCode == 0xFF) {
          //Send Acc data

          #ifdef DEBUG
            Serial.println("Start Acc data packet");
          #endif /*DEBUG*/

          accVector Acc1Vector;

          //Measuring Time
          AccPacketStart = timerRead(timer1);
          #ifdef DEBUG
            Serial.print("AccPacketStart: ");
            Serial.println(AccPacketStart);
          #endif /*DEBUG*/
          AccPacketStartMicro = timerReadMicros(timer1);
          int32_t AccVectorTime = AccPacketStart - AccVectorEnd;
          uint32_t AccVectorTimeMicro = AccPacketStartMicro - AccVectorEndMicro;
          #ifdef DEBUG
            Serial.print("AccVectorTime: ");
            Serial.println(AccVectorTime);
            Serial.print("AccVectorTimeMicro: ");
            Serial.println(AccVectorTimeMicro);
            Serial.print("AccPacketStartMicro: ");
            Serial.println(AccPacketStartMicro);
          #endif /*DEBUG*/
          
          //Get data
          Acc1Vector = getAccAxes();  //Gets data from the accelerometers
          //Acc1Vectors is a circular array holding the last 500 samples
          if (dataCount < accPacketSize) {
            Acc1Vectors[dataCount] = Acc1Vector;  //Puts acceleration vector into vector array
          } else {
            dataCount = 0;
          }
          vectortoBytes(Acc1Vector);
          dataCount++;

          #ifdef DEBUG
            Serial.print("accVector.XAcc: ");
            Serial.println(accVector.XAcc, DEC);
            Serial.print("accVector.YAcc: ");
            Serial.println(accVector.YAcc, DEC);
            Serial.print("accVector.ZAcc: ");
            Serial.println(accVector.ZAcc, DEC);
            Serial.print("accVector.XT: ");
            Serial.println(accVector.XT, DEC);
            Serial.print("accVector.YT: ");
            Serial.println(accVector.YT, DEC);
            Serial.print("accVector.ZT: ");
            Serial.println(accVector.ZT, DEC);
          #endif /*DEBUG*/

          //Write vector byte array to socket
          for(int i =0; i < 18; i++) {
            client.write(bytes[i]);
            
              Serial.print("HEX ");
              Serial.print(i);
              Serial.print(": ");
              Serial.println(bytes[i], HEX);

            #ifdef DEBUG
              Serial.print("DEC ");
              Serial.print(i);
              Serial.print(": ");
              Serial.println(bytes[i], DEC);
              Serial.print("HEX ");
              Serial.print(i);
              Serial.print(": ");
              Serial.println(bytes[i], HEX);
            #endif /*DEBUG*/
          }

              #ifdef DEBUG
                Serial.print("socketTestData Sent: ");
                Serial.println(socketTestData, HEX);
              #endif /*DEBUG*/

              //Timing Tests 
              AccVectorEnd = AccPacketStart;
              AccVectorEndMicro = AccPacketStartMicro;

              #ifdef DEBUG
                
                Serial.print("AccVectorEnd: ");
                Serial.println(AccVectorEnd);

                Serial.print("AccVectorEndMicro: ");
                Serial.println(AccVectorEndMicro);
              
                Serial.print("AccVectorTime: ");
                Serial.println(AccVectorTime);

                Serial.print("AccVectorTimeMicro: ");
                Serial.println(AccVectorTimeMicro);
              #endif /*DEBUG*/
          }  
        }
      }
    //client.stop();
    Serial.println("Client disconnected");
    Serial.println();

  }
  
  if (timerRead(timer1) >= 0x100000000) {   //Full 32 bits = 0x100000000 (~ 9min with 8MHz timer); 24 bits = 0x1000000 (2s with 8MHz timer)
    uint32_t rollOver = timerRead(timer1);

      #ifdef DEBUG
        Serial.print("rollOver: ");
        Serial.println(rollOver);
      #endif /*DEBUG*/

      #ifdef DEBUG
        uint32_t rollOverMicro = timerReadMicros(timer1);
        Serial.print("rollOverMicro: ");
        Serial.println(rollOverMicro);
      #endif /*DEBUG*/

    timerRestart(timer1);
    #ifdef DEBUG
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      Serial.println("TIMER ROLLOVER");
      #endif /*DEBUG*/
  }
  
  }
