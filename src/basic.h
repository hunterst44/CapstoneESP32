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
// #define AccPort1 1        //Ports for Accelerometer 1 (for multiplexor)
// #define AccPort2 2
// #define AccPort3 3
// #define AccPort4 4
#define XOUTHI 0x03      //Registers for Acceleration data output
#define XOUTLO 0x04 
#define YOUTHI 0x05 
#define YOUTLO 0x06 
#define ZOUTHI 0x07 
#define ZOUTLO 0x08 
//#define accPacketSize 500     //Size of a unit of acc samples
#define NUMSENSORS 4       //NUmber of sensors
#define ACCPACKSIZE 18     //Size in bytes to send a sample from 1 accelerometer
#define SOCKPACKSIZE 36   //Total size of packet set to socket client (ACCPACKSIZE * number of sensors)
#define MOVINGAVGSIZE 5   //Number samples to include in moving average (decimation)
#define SAMPLEINC 2       //Number of samples to increment for each decimation calculation

///************************************
//          Data Globals
//*************************************

// extern uint8_t state;
// extern uint8_t debug; 
struct accVector {
    int16_t XAcc;
    int16_t YAcc;
    int16_t ZAcc;
    uint32_t XT;
    uint32_t YT;
    uint32_t ZT;
};

extern hw_timer_t * timer1;
extern uint8_t I2CPort;
extern char bytes[SOCKPACKSIZE];
extern accVector accVecArray[NUMSENSORS][MOVINGAVGSIZE];
extern uint8_t tdxCount;
extern uint32_t TdXStart;
extern uint32_t TdXEnd;

///************************************
//          I2C Globals
//*************************************
extern accVector getAccAxes(uint8_t Port);
extern int16_t readAccReg(uint8_t Port, uint8_t r);
extern void changeI2CPort(uint8_t I2CPort);
extern int16_t getAxisAcc(int16_t axisHi, int16_t axisLo);
extern void vectortoBytes(accVector vector, uint8_t sensorIndex);
extern void cubicSpline(uint8_t senseIndex);

//**********************************
//           WiFI Server Globals
//**********************************
//extern AsyncWebServer server;
//extern AsyncWebSocket ws;

extern uint8_t socketRXArr[24];
extern uint8_t socketDataIn;