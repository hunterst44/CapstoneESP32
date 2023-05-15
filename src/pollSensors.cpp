/*pollSensors.cpp

Created May 15, 2023 by Joel Legassie

Contains functions used to gather data from MXC4005XC-B Accelerometer

void getAccAxes(uint8_t Port)   -- Controls flow of the sensor reading 
    calls:
    int16_t readAccReg(uint8_t Port, uint8_t r)  --  gets raw bits from sensor  
        calls:
            void changeI2CPort(uint8_t I2CPort) -- sets I2C port on multiplexor     
    int16_t getAxisAcc(int16_t axisHi, int16_t axisLo)  --  Create acceleration vector from raw bits (incl time data)

vectortoBytes(accVector vector) -- makes byte array for TX

*/
#include <Arduino.h>
#include <WiFi.h>
#include "basic.h"
#include <Wire.h>
#include <stdlib.h>
#include "secrets.h"


/************************
 * getAccAxes()
*************************/

accVector getAccAxes(uint8_t Port) {
 //Read Axes of Acc1
  #ifdef DEBUG
    Serial.println();
    Serial.print("accVector getAccAxes()");
  #endif /*DEBUG*/
    
    accVector accVector;
    
//     //Get Orientation register values
//     int16_t orientReg = readAccReg(Port, 1);
    
//     #ifdef DEBUG
//       Serial.print("orientReg: ");
//       Serial.println(orientReg, HEX);
//     #endif /*DEBUG*/
//   /*
//     orientation:
//     bit 0 - DRDY 
//     bit 4 ORXY0
//     bit 5 ORXY1
//     bit 6 ORZ

//     Orientation ORXY0  ORXY1
//         +X        0      0
//         +Y        0      1
//         -X        1      0
//         -Y        1      1
//   */
//  orientReg = orientReg & 0b01110000;

//  #ifdef DEBUG
//     Serial.print("orientReg Shifted: ");
//     Serial.println(orientReg, HEX);
//   #endif /*DEBUG*/

//  uint8_t orient = (uint8_t)(orientReg >> 4);

//  #ifdef DEBUG
//     Serial.print("orientation bits: ");
//     Serial.println(orient, HEX);
//     if (orient == 0) {
//         Serial.print("orient = 0: +X");
//     } else if (orient == 1) {
//         Serial.print("orient = 1: +Y");
//     } else if (orient == 2) {
//         Serial.print("orient = 2: -X");
//     } else if (orient == 3) {
//         Serial.print("orient = 3: -Y");
//     } 
//   #endif /*DEBUG*/

    //Get X register values
    //XHi
    int16_t XHi = readAccReg(Port, 3);

    #ifdef DEBUG
      Serial.print("XHi: ");
      Serial.println(XHi, DEC);
    #endif /*DEBUG*/

    //XLo  
    int16_t XLo = readAccReg(Port, 4);

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
    int16_t YHi = readAccReg(Port, 5);

    #ifdef DEBUG
      Serial.print("YHi: ");
      Serial.println(YHi, DEC);
    #endif /*DEBUG*/

    //YLo  
    int16_t YLo = readAccReg(Port, 6);

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
    int16_t ZHi = readAccReg(Port, 7);

    #ifdef DEBUG
      Serial.print("ZHi: ");
      Serial.println(ZHi, DEC);
    #endif /*DEBUG*/

    //ZLo  
    int16_t ZLo = readAccReg(Port, 8);

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


/****************************************
 * readAccReg(uint8_t Port, uint8_t r)
****************************************/

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

/****************************************
 * changeI2CPort(uint8_t I2CPort)
****************************************/

void changeI2CPort(uint8_t I2CPort) {   //Change the port of the I2C multiplexor
  Wire.beginTransmission(I2CADDR);
  Wire.write(I2CPort);
  Wire.endTransmission();
}

/********************************************
 * getAxisAcc(int16_t axisHi, int16_t axisLo)
*********************************************/

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

/********************************************
 * vectortoBytes(accVector vector)
*********************************************/
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