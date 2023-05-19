/*
main.cpp

Created May 1, 2023 by Joel Legassie
Sensor polling is initiated when the ESP32 receives 0xFF from the client and continues until the client closes the connection


*/
#include <Arduino.h>
#include <WiFi.h>
#include "basic.h"
#include <Wire.h>
#include <stdlib.h>
#include "secrets.h"
#include <math.h>

//Globals
uint8_t I2CPort = 0;
uint8_t vecCount = 0;  //Count number of samples taken for timing tests
//uint8_t dataCount = 0;

//int16_t Acc1Avg[3];   //XYZ vector

const char* ssid = NETWORK;
const char* password = PASS;
WiFiServer wifiServer(80);
WiFiClient client;
int16_t socketTestData = 4040;

char bytes[SOCKPACKSIZE];
accVector accVecArray[NUMSENSORS][MOVINGAVGSIZE]; //array of vector arrays 
//accVector Acc1Vectors[accPacketSize];
uint8_t sampleCount = 0;    //Counts number of samples for the moving average filter


//Timer stuff
hw_timer_t * timer1 = NULL;

//Measurement globals - can remove from production
uint32_t AccVecStart;
uint32_t AccVecStartMicro;
uint32_t AccPacketStart;
uint32_t AccPacketStartMicro;
uint32_t AccVectorEnd = 0;
uint32_t AccVectorEndMicro = 0;
uint32_t AccPacketEnd;
uint32_t AccPacketEndMicro;


/************************
 * setup()
*************************/
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

/************************
 * loop()
*************************/
void loop() {

  if (vecCount == 0) {
      AccPacketStartMicro = timerReadMicros(timer1);
  }

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

          //accVector Acc1Vector;

          //Measuring Time
          AccVecStart = timerRead(timer1);
          #ifdef DEBUG
            Serial.print("AccVecStart: ");
            Serial.println(AccVecStart);
          #endif /*DEBUG*/
          AccVecStartMicro = timerReadMicros(timer1);
          
          //Get data
          
          while (sampleCount < MOVINGAVGSIZE) {
            uint32_t getDataStart = timerReadMicros(timer1);
            accVecArray[0][sampleCount] = getAccAxes(1);  //Gets data from the accelerometer on I2C port 1 (SCL0 /SDA0)
            accVecArray[1][sampleCount] = getAccAxes(2);  //Gets data from the accelerometer on I2C port 2 (SCL1 /SDA1)
            accVecArray[2][sampleCount] = getAccAxes(1);  //Gets data from the accelerometer on I2C port 1 (SCL0 /SDA0)
            accVecArray[3][sampleCount] = getAccAxes(2);  //Gets data from the accelerometer on I2C port 2 (SCL1 /SDA1)

            uint32_t getDataEnd = timerReadMicros(timer1);
            Serial.print("data Time Micros: ");
            Serial.println(getDataEnd - getDataStart);
            sampleCount++;
          }

          if (sampleCount == MOVINGAVGSIZE) {        //After moving average size of samples (3) filter
            //accVector AccVectorMAVG[NUMSENSORS];
            for (int i =0; i < NUMSENSORS; i++) {   //One vector per sensor
              vectortoBytes(accVecArray[i][sampleCount], i);  //Puts data into byte format for socket TX
              //AccVectorMAVG[i] = movingAvg(i);     

              //vectortoBytes(AccVectorMAVG[i], i);  //Puts data into byte format for socket TX
              //vectortoBytes(AccVectorMAVG[1], 1);  //Puts data into byte format for socket TX
            }

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
            uint32_t TXStart = timerReadMicros(timer1);
            for(int i = 0; i < SOCKPACKSIZE; i++) {
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
            uint32_t TXEnd = timerReadMicros(timer1);
            Serial.print("Tx Time Micros: ");
            Serial.println(TXEnd - TXStart);

            vecCount++;  //Increment count - for timing
          }
          if (vecCount == 50) {
            //AccPacketEnd = timerRead(timer1);
            AccPacketEndMicro = timerReadMicros(timer1);
            Serial.print("packet Time Micros: ");
            Serial.println(AccPacketEndMicro - AccPacketStartMicro);
            vecCount = 0;
          } 

              #ifdef DEBUG
                Serial.print("socketTestData Sent: ");
                Serial.println(socketTestData, HEX);
              #endif /*DEBUG*/

              //Timing Tests 
              AccVectorEnd = timerRead(timer1);
              AccVectorEndMicro = timerReadMicros(timer1);

              uint32_t AccVectorTime = AccVectorEnd - AccVecStart;
              uint32_t AccVectorTimeMicro = AccVectorEndMicro - AccVecStartMicro;

              Serial.print("AccVectorTime: ");
              Serial.println(AccVectorTime);
              Serial.print("AccVectorTimeMicro: ");
              Serial.println(AccVectorTimeMicro);

              #ifdef DEBUG
                Serial.print("AccVectorTime: ");
                Serial.println(AccVectorTime);
                Serial.print("AccVectorTimeMicro: ");
                Serial.println(AccVectorTimeMicro);
                Serial.print("AccVecStartMicro: ");
                Serial.println(AccVecStartMicro);
              #endif /*DEBUG*/

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
