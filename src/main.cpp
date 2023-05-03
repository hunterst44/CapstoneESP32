#include <Arduino.h>
#include <WiFi.h>
//#include <SPIFFS.h>
#include "basic.h"
#include <Wire.h>
#include <stdlib.h>
#include "secrets.h"

/*
MXC4005XC-B Accelerometer I2C requirements:
The first byte transmitted by the master following a START is used to address the slave device. The first 7 bits
contain the address of the slave device, and the 8th bit is the R/W* bit (read = 1, write = 0; the asterisk indicates
active low, and is used instead of a bar). If the transmitted address matches up to that of the MXC400xXC, then the
MXC400xXC will acknowledge receipt of the address, and prepare to receive or send data.

If the master is writing to the MXC400xXC, then the next byte that the MXC400xXC receives, following the address
byte, is loaded into the address counter internal to the MXC400xXC. The contents of the address counter indicate
which register on the MXC400xXC is being accessed. If the master now wants to write data to the MXC400xXC, it
just continues to send 8-bit bytes. Each byte of data is latched into the register on the MXC400xXC that the address
counter points to. The address counter is incremented after the transmission of each byte.

If the master wants to read data from the MXC400xXC, it first needs to write the address of the register it wants to
begin reading data from to the MXC400xXC address counter. It does this by generating a START, followed by the
address byte containing the MXC400xXC address, with R/W* = 0. The next transmitted byte is then loaded into the
MXC400xXC address counter. Then, the master repeats the START condition and re-transmits the MXC400xXC
address, but this time with the R/W* bit set to 1. During the next transmission period, a byte of data from the
MXC400xXC register that is addressed by the contents of the address counter will be transmitted from the
MXC400xXC to the master. As in the case of the master writing to the MXC400xXC, the contents of the address
counter will be incremented after the transmission of each byte. 

I2C Address (7bit):
5 - 15H (0x0F)?

Addresses Register:
0x03 XOUT upper [0-7]
0x04 XOUT lower [4-7]

0x05 YOUT upper [0-7]
0x06 YOUT lower [4-7]

0x07 ZOUT upper [0-7]
0x08 ZOUT lower [4-7]

To do: debug  I2C
Get Orientation
Data design
*/

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
void changeI2CPort(uint8_t I2CPort) {   //Change the port of the I2C multiplexor
  Wire.beginTransmission(I2CADDR);
  Wire.write(I2CPort);
  Wire.endTransmission();
}

// void playWithCastingtwoBytes(int16_t *twoByte) {

//   #ifdef DEBUG
//     Serial.println();
//     Serial.println("playWithCastingtwoBytes(uint16_t *twoByte)");
//     Serial.print("twobyte: ");
//     Serial.println(*twoByte, HEX);
//   #endif /*DEBUG*/

//   char* bytes = (char*) twoByte;
//   #ifdef DEBUG
//     Serial.print("sizeof bytes: ");
//     Serial.println(sizeof(bytes), DEC);
//     Serial.print("byte 0: ");
//     Serial.println(bytes[0], HEX);
//     Serial.print("byte 1: ");
//     Serial.println(bytes[1], HEX);
//   #endif /*DEBUG*/

//   #ifdef DEBUG
//     Serial.println();
//   #endif /*DEBUG*/
// }

void vectortoBytes(accVector vector) {
  #ifdef DEBUG
    Serial.println();
    Serial.println("VectortoBytes(accVector vector)");
  #endif /*DEBUG*/

  //char bytes[18];
  
  int16_t XAccTmp = vector.XAcc;
  char* XAccBytes = (char*) &XAccTmp;
  
  #ifdef DEBUG
    Serial.print("sizeof XAccBytes: ");
    Serial.println(sizeof(XAccBytes), DEC);
    Serial.print(XAccBytes[0], HEX);
    Serial.print(", ");
    Serial.print(XAccBytes[1], HEX);
    Serial.println();
  #endif /*DEBUG*/

  int16_t YAccTmp = vector.YAcc;
  char* YAccBytes = (char*) &YAccTmp;

  #ifdef DEBUG
    Serial.print("sizeof YAccBytes: ");
    Serial.println(sizeof(YAccBytes), DEC);
    Serial.print(YAccBytes[0], HEX);
    Serial.print(", ");
    Serial.print(YAccBytes[1], HEX);
    Serial.println();
  #endif /*DEBUG*/

  int16_t ZAccTmp = vector.ZAcc;
  char* ZAccBytes = (char*) &ZAccTmp;

  #ifdef DEBUG
    Serial.print("sizeof ZAccBytes: ");
    Serial.println(sizeof(ZAccBytes), DEC);
    Serial.print(ZAccBytes[0], HEX);
    Serial.print(", ");
    Serial.print(ZAccBytes[1], HEX);
    Serial.println();
  #endif /*DEBUG*/

  int32_t XTTmp = vector.XT;
  char* XTBytes = (char*) &XTTmp;

    Serial.print("sizeof XTBytes: ");
    Serial.println(sizeof(XTBytes), DEC);
    Serial.print(XTBytes[0], HEX);
    Serial.print(", ");
    Serial.print(XTBytes[1], HEX);
    Serial.print(", ");
    Serial.print(XTBytes[2], HEX);
    Serial.print(", ");
    Serial.print(XTBytes[3], HEX);
    Serial.println();
  
  #ifdef DEBUG
    Serial.print("sizeof XTBytes: ");
    Serial.println(sizeof(XTBytes), DEC);
    Serial.print(XTBytes[0], HEX);
    Serial.print(", ");
    Serial.print(XTBytes[1], HEX);
    Serial.print(", ");
    Serial.print(XTBytes[2], HEX);
    Serial.print(", ");
    Serial.print(XTBytes[3], HEX);
    Serial.println();
  #endif /*DEBUG*/

  int32_t YTTmp = vector.YT;
  char* YTBytes = (char*) &YTTmp;

    Serial.print("sizeof YTBytes: ");
    Serial.println(sizeof(YTBytes), DEC);
    Serial.print(YTBytes[0], HEX);
    Serial.print(", ");
    Serial.print(YTBytes[1], HEX);
    Serial.print(", ");
    Serial.print(YTBytes[2], HEX);
    Serial.print(", ");
    Serial.print(YTBytes[3], HEX);
    Serial.println();
  
  #ifdef DEBUG
    Serial.print("sizeof YTBytes: ");
    Serial.println(sizeof(YTBytes), DEC);
    Serial.print(YTBytes[0], HEX);
    Serial.print(", ");
    Serial.print(YTBytes[1], HEX);
    Serial.print(", ");
    Serial.print(YTBytes[2], HEX);
    Serial.print(", ");
    Serial.print(YTBytes[3], HEX);
    Serial.println();
  #endif /*DEBUG*/

  int32_t ZTTmp = vector.ZT;
  char* ZTBytes = (char*) &ZTTmp;

    Serial.print("sizeof ZTBytes: ");
    Serial.println(sizeof(ZTBytes), DEC);
    Serial.print(ZTBytes[0], HEX);
    Serial.print(", ");
    Serial.print(ZTBytes[1], HEX);
    Serial.print(", ");
    Serial.print(ZTBytes[2], HEX);
    Serial.print(", ");
    Serial.print(ZTBytes[3], HEX);
    Serial.println();

  #ifdef DEBUG
    Serial.print("sizeof ZTBytes: ");
    Serial.println(sizeof(ZTBytes), DEC);
    Serial.print(ZTBytes[0], HEX);
    Serial.print(", ");
    Serial.print(ZTBytes[1], HEX);
    Serial.print(", ");
    Serial.print(ZTBytes[2], HEX);
    Serial.print(", ");
    Serial.print(ZTBytes[3], HEX);
    Serial.println();
  #endif /*DEBUG*/

  bytes[0] = XAccBytes[0];
  bytes[1] = XAccBytes[1];
  bytes[2] = YAccBytes[0];
  bytes[3] = YAccBytes[1];
  bytes[4] = ZAccBytes[0];
  bytes[5] = ZAccBytes[1];

  bytes[6] = XTBytes[0];
  bytes[7] = XTBytes[1];
  bytes[8] = XTBytes[2];
  bytes[9] = XTBytes[3];

  bytes[10] = YTBytes[0];
  bytes[11] = YTBytes[1];
  bytes[12] = YTBytes[2];
  bytes[13] = YTBytes[3];

  bytes[14] = ZTBytes[0];
  bytes[15] = ZTBytes[1];
  bytes[16] = ZTBytes[2];
  bytes[17] = ZTBytes[3];

#ifdef DEBUG
  Serial.println();
  Serial.print("Bytes: ");
  for (int i =0; i < sizeof(bytes); i++) {
    Serial.println(bytes[i], HEX);
  }
  Serial.println();
#endif /*DEBUG*/
}

int16_t readAccReg(uint8_t Port, uint8_t r) {
  #ifdef DEBUG
    Serial.println();
    Serial.println("readAccReg(uint8_t Port, int r)");
  #endif /*DEBUG*/

  int16_t regOut = 0;
  
  if (Port != I2CPort) {
    I2CPort = Port;
    changeI2CPort(Port);
  }
  
  #ifdef DEBUG
    Serial.println("Multiplexor Port selected");
    Serial.println("Send Device Address then register address (r)");
  #endif /*DEBUG*/

  Wire.beginTransmission(MXCI2CADDR);    //Open TX with start address and stop
  Wire.write(r);                  //Send the register we want to read to the sensor
  
  #ifdef DEBUG
    Serial.print("r transmitted: ");
    Serial.println(r, HEX);
  #endif /*DEBUG*/

  uint8_t error = Wire.endTransmission();  //Send a stop
      if (error == 0) {
        #ifdef DEBUG
          Serial.print("I2C device found at address 0x15\n");
        #endif /*DEBUG*/

      } else {
          #ifdef DEBUG
            Serial.print("I2C Error: ");
            Serial.println(error,HEX);
          #endif /*DEBUG*/
      }

    Wire.requestFrom(MXCI2CADDR, 1, 1);   //Send read request
    while(Wire.available()) {
      regOut = Wire.read();

      #ifdef DEBUG
        Serial.print("Register Output: ");
        Serial.println(regOut, HEX);
      #endif /*DEBUG*/
    }
    
    #ifdef DEBUG
      Serial.println();
    #endif /*DEBUG*/

    return regOut;
  }

int16_t getAxisAcc(int16_t axisHi, int16_t axisLo) {
  #ifdef DEBUG
    Serial.println();
    Serial.println("getAxisAcc(int16_t axisHi, int16_t axisLo)");
    Serial.print("axisAccHi First: ");
    Serial.println(axisHi, HEX);
    Serial.print("axisAccLo First: ");
    Serial.println(axisLo, HEX);
  #endif /*DEBUG*/

    int16_t axisAcc = 0;
    axisAcc = axisHi << 4;           //High value 
    
    #ifdef DEBUG
      Serial.print("axisAccHi Shift: ");
      Serial.println(axisAcc, HEX);
    #endif /*DEBUG*/
    
    axisAcc = axisAcc + (axisLo >> 4);
    
    #ifdef DEBUG
      Serial.print("axisAccLo: ");
      Serial.println((axisLo >> 4), HEX);
      Serial.print("axisAcc: ");
      Serial.println(axisAcc, HEX);
      Serial.println();
    #endif /*DEBUG*/

    return axisAcc;
  }

accVector getAccAxes() {
 //Read Axes of Acc1
  #ifdef DEBUG
    Serial.println();
    Serial.print("accVector getAccAxes()");
  #endif /*DEBUG*/
    accVector accVector;
    
    //Get Orientation register values
    int16_t orientReg = readAccReg(AccPort1, 1);
    
    #ifdef DEBUG
      Serial.print("orientReg: ");
      Serial.println(orientReg, HEX);
    #endif /*DEBUG*/
  /*
    orientation:
    bit 0 - DRDY 
    bit 4 ORXY0
    bit 5 ORXY1
    bit 6 ORZ

    Orientation ORXY0  ORXY1
        +X        0      0
        +Y        0      1
        -X        1      0
        -Y        1      1
  */
 orientReg = orientReg & 0b01110000;

 #ifdef DEBUG
    Serial.print("orientReg Shifted: ");
    Serial.println(orientReg, HEX);
  #endif /*DEBUG*/

 uint8_t orient = (uint8_t)(orientReg >> 4);

 #ifdef DEBUG
    Serial.print("orientation bits: ");
    Serial.println(orient, HEX);
    if (orient == 0) {
        Serial.print("orient = 0: +X");
    } else if (orient == 1) {
        Serial.print("orient = 1: +Y");
    } else if (orient == 2) {
        Serial.print("orient = 2: -X");
    } else if (orient == 3) {
        Serial.print("orient = 3: -Y");
    } 
  #endif /*DEBUG*/

    //Get X register values
    //XHi
    int16_t XHi = readAccReg(AccPort1, 3);

    #ifdef DEBUG
      Serial.print("XHi: ");
      Serial.println(XHi, DEC);
    #endif /*DEBUG*/

    //XLo  
    int16_t XLo = readAccReg(AccPort1, 4);

    #ifdef DEBUG
      Serial.print("XLo: ");
      Serial.println(XLo, DEC);
    #endif /*DEBUG*/

    //Combine Hi and Lo to get axis value
    accVector.XAcc = getAxisAcc(XHi, XLo);

    #ifdef DEBUG
      Serial.print("accVector.XAcc: ");
      Serial.println(accVector.XAcc, DEC);
    #endif /*DEBUG*/

    //Get timer value
    accVector.XT = (int32_t)timerRead(timer1);
      Serial.print("accVector.XT: ");
      Serial.println(accVector.XT);

    #ifdef DEBUG
      Serial.print("accVector.XT: ");
      Serial.println(accVector.XT);
      Serial.print("timerReadMicros(timer1): ");
      Serial.print(timerReadMicros(timer1));
      Serial.println();
    #endif /*DEBUG*/

    //Get Y register values
    //YHi
    int16_t YHi = readAccReg(AccPort1, 5);

    #ifdef DEBUG
      Serial.print("YHi: ");
      Serial.println(YHi, DEC);
    #endif /*DEBUG*/

    //YLo  
    int16_t YLo = readAccReg(AccPort1, 6);

    #ifdef DEBUG
      Serial.print("YLo: ");
      Serial.println(YLo, DEC);
    #endif /*DEBUG*/

    //Combine Hi and Lo to get axis value
    accVector.YAcc = getAxisAcc(YHi, YLo);

    #ifdef DEBUG
      Serial.print("accVector.YAcc: ");
      Serial.println(accVector.YAcc, DEC);
    #endif /*DEBUG*/

    //Get timer value
    accVector.YT = (int32_t)timerRead(timer1);
      
      Serial.print("accVector.YT: ");
      Serial.println(accVector.YT);

    #ifdef DEBUG
      Serial.print("accVector.YT: ");
      Serial.println(accVector.YT);
      Serial.print("timerReadMicros(timer1): ");
      Serial.print(timerReadMicros(timer1));
      Serial.println();
    #endif /*DEBUG*/

    //Get Z register values
    //Zi  
    int16_t ZHi = readAccReg(AccPort1, 7);

    #ifdef DEBUG
      Serial.print("ZHi: ");
      Serial.println(ZHi, DEC);
    #endif /*DEBUG*/

    //ZLo  
    int16_t ZLo = readAccReg(AccPort1, 8);

    #ifdef DEBUG
      Serial.print("ZLo: ");
      Serial.println(ZLo, DEC);
    #endif /*DEBUG*/

    //Combine Hi and Lo to get axis value
    accVector.ZAcc = getAxisAcc(ZHi, ZLo);

    #ifdef DEBUG
      Serial.print("accVector.ZAcc: ");
      Serial.println(accVector.ZAcc, DEC);
    #endif /*DEBUG*/

    //Get timer value
    accVector.ZT = (int32_t)timerRead(timer1);
      Serial.print("accVector.ZT: ");
      Serial.println(accVector.ZT);

    #ifdef DEBUG
      Serial.print("accVector.ZT: ");
      Serial.println(accVector.ZT);
      Serial.print("timerReadMicros(timer1): ");
      Serial.print(timerReadMicros(timer1));
      Serial.println();
    #endif /*DEBUG*/

    return accVector;
}

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

          for(int i =0; i < 18; i++) {
            client.write(bytes[i]);
            Serial.print("DEC ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(bytes[i], DEC);
            Serial.print("HEX ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(bytes[i], HEX);

            #ifdef DEBUG
            Serial.print("Data Sent ");
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

  // if (dataCount < accPacketSize) {
  //   #ifdef DEBUG
  //     Serial.println(); 
  //     Serial.print("******"); 
  //     Serial.print("Loop#: ");  
  //     Serial.println(dataCount, DEC);
  //   #endif /*DEBUG*/

    // accVector Acc1Vector;
  
    // #ifdef DEBUG
    //   Serial.println("Start Acc data packet");
    // #endif /*DEBUG*/

    // AccPacketStart = timerRead(timer1);

    // #ifdef DEBUG
    //   Serial.print("AccPacketStart: ");
    //   Serial.println(AccPacketStart);
    // #endif /*DEBUG*/

    // AccPacketStartMicro = timerReadMicros(timer1);

    // #ifdef DEBUG
    //   Serial.print("AccPacketStartMicro: ");
    //   Serial.println(AccPacketStartMicro);
    // #endif /*DEBUG*/
    
    // Acc1Vector = getAccAxes();  //Gets data from the accelerometers

    // #ifdef DEBUG
    //   Serial.print("accVector.XAcc: ");
    //   Serial.println(accVector.XAcc, DEC);
    //   Serial.print("accVector.YAcc: ");
    //   Serial.println(accVector.YAcc, DEC);
    //   Serial.print("accVector.ZAcc: ");
    //   Serial.println(accVector.ZAcc, DEC);
    //   Serial.print("accVector.XT: ");
    //   Serial.println(accVector.XT, DEC);
    //   Serial.print("accVector.YT: ");
    //   Serial.println(accVector.YT, DEC);
    //   Serial.print("accVector.ZT: ");
    //   Serial.println(accVector.ZT, DEC);
    // #endif /*DEBUG*/

    //playWithCastingVector(Acc1Vector);
    
    

    //set dataCount  to zero to start sensor 
  // }
  
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
