// ----------------------------------------------------------------------------
// Definition of macros
// ----------------------------------------------------------------------------

//#define DEBUG
#define HTTP_PORT 80
#define LOCALADDR 1
#define I2CADDR 0x70  //I2C Address for multiplexor
#define I2C_SDA 0     //I2C pins
#define I2C_SCL 1
#define MXCI2CADDR 0x15   //I2C Address for MXC400 Accelerometer
#define AccPort1 1        //Ports for Accelerometer 1 (for multiplexor)
#define AccPort2 2
#define AccPort3 3
#define AccPort4 4
#define XOUTHI 0x03      //Registers for Acceleration data output
#define XOUTLO 0x04 
#define YOUTHI 0x05 
#define YOUTLO 0x06 
#define ZOUTHI 0x07 
#define ZOUTLO 0x08 
#define accPacketSize 500     //Size of a unit of acc samples

///************************************
//          State Model Globals
//*************************************

extern uint8_t state;
extern uint8_t debug; 
struct accVector {
    int16_t XAcc;
    int16_t YAcc;
    int16_t ZAcc;
    uint32_t XT;
    uint32_t YT;
    uint32_t ZT;
};

///************************************
//          I2C Globals
//*************************************
extern void changeI2CPort(uint8_t I2CPort);
extern int16_t readAccReg(uint8_t Port, uint8_t r);
extern int16_t getAxisAcc(int16_t axisHi, int16_t axisLo);
extern accVector getAccAxes();
extern void vectortoBytes(accVector vector);

//**********************************
//           WiFI Server Globals
//**********************************
//extern AsyncWebServer server;
//extern AsyncWebSocket ws;

extern uint8_t socketRXArr[24];
extern uint8_t socketDataIn;