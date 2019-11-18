/****************************************Copyright(c)*****************************************************
**                                    X-Fab Texas, Inc.
**
**                                http://www.xfabtexas.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:             main.cpp
** Latest modified Date:  2019-10-16
** Latest Version:        V0.2.0
** Descriptions:          Main functions working
**
**--------------------------------------------------------------------------------------------------------
** Modify by:             Kevin Craker
** Modified date:         2019-10-16
** Version:               V0.2.0
** Descriptions:          Modified,From DobotDemoForSTM32
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#define TRACE     0
// #define HW_VERSION
const char FIRMWARE_VERSION[] = "0.9.0";

#include "stdio.h"
#include <EEPROMex.h> 
#include <Magician.h>
#include <Protocol.h>
#include "FlexiTimer2.h"
#include <MENWIZ.h>
#include "hardwareDefinition.h"
#include <ReticleStation.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 10
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// int address = 0;
uint8_t processFlag = 1;
uint8_t completed = 0;
char buf[40];
uint32_t robotReply;
float temperature, humidity, temperature1, humidity1;

const char signature[] = {32, 3, 0};
const char nCursor[] = {4, 0};
const char blank[] = {32, 0};
const char bHigh[] = {7, 0};
const char bLow[] = {6, 0};
const char thermometer[] = {1, 0};
const char drip[] = {5, 0};

// Globals and setup required by Robot
// Set Serial TX & RX Buffer Size
#define SERIAL_TX_BUFFER  64
#define SERIAL_RX_BUFFER  256

// Create global parameters for Robot control
EndEffectorParams         gEndEffectorParams;
JOGJointParams            gJOGJointParams;
JOGCoordinateParams       gJOGCoordinateParams;
JOGCommonParams           gJOGCommonParams;
JOGCmd                    gJOGCmd;
PTPCoordinateParams       gPTPCoordinateParams;
PTPCommonParams           gPTPCommonParams;
PTPCmd                    gPTPCmd;
PTPJumpParams             gPTPJumpParams;

uint64_t gQueuedCmdIndex;

struct joy_params {
  uint16_t jmin;
  uint16_t jmid;
  uint16_t jmax;
};

joy_params joy[2];
int joy_stg[2];

ReticleStation station[5];


// Globals and setup required by Menwiz menu system
// Create global object menu and lcd
menwiz menu;

// instantiate global variables to bind to menu
int      tp=0;
float    f=26.0;
boolean  bb=0;
byte     b=50;

#include "menuHandlers.h"



/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/



void setup(){
  Serial.begin(9600);
  Serial1.begin(115200);
  dht.begin();
   // have a look on memory before menu creation
  // Serial.println(sizeof(menwiz));  
  printf_begin();

  joy_stg[0]        = 0;
  joy_stg[1]        = 6;

  for(int i; i < 5; i++)
  {
    station[i].setAddress(12+(16*i));
  }
  station[0].setSensePin(inPin[8]);
  station[1].setSensePin(inPin[9]);
  station[2].setSensePin(inPin[10]);
  station[3].setSensePin(inPin[1]);
  station[4].setSensePin(inPin[1]);
  
  #include "menu.h"
  #include "digitalIO.h"

  EEPROM.readBlock(joy_stg[0], joy[0]);
  EEPROM.readBlock(joy_stg[1], joy[1]);
  
  // Set Timer Interrupt to call Serialread every 100mSec
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();

  
  }

/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/



void loop(){

  InitRAM();
  ProtocolInit();
  menu.draw();
  delay(5000);
  
  while(!initRobot());

  SetJOGJointParams(&gJOGJointParams);
  SetJOGCoordinateParams(&gJOGCoordinateParams);
  SetJOGCommonParams(&gJOGCommonParams);
  SetPTPJumpParams(&gPTPJumpParams);
  
  resetRobot();
  
  BTU.check();
  BTD.check();
  BTL.check();
  BTR.check();
  BTE.check();
  BTC.check();
  BTG.check();
  BTM.check();

  while(1){

  menu.draw();
  if (BTC.check()==4) {
    menu.chgTimeout(300000);
    menu.exitFlag();
    }
  //PUT APPLICATION CODE HERE (if any)
  if(completed&&(((millis()/100)%10)>5))
     digitalWrite(outPin[0],HIGH);
  else
    digitalWrite(outPin[0],LOW);
  if((station[0].caseSet()+station[1].caseSet()+station[2].caseSet())==0)
    completed = 0;    
    }
  }

/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/

// user defined callbacks

uint8_t makeChar(uint8_t value) {
  uint8_t valueIn = value;
  if (value > 10)
    value = value/10;
  value = value + 48;
  return value;
}

  
void act(){
  Serial.println("FIRED ACTION!");
 }

void menuQuit(){
  menu.chgTimeout(5);
  menu.exitFlag();
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void InitRAM(void)
{
    //Set JOG Model
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;
   
    gJOGCmd.cmd = AP_DOWN;
    gJOGCmd.isJoint = JOINT_MODEL;

    

    //Set PTP Model
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = JUMP_XYZ;
    gPTPCmd.x = 200;
    gPTPCmd.y = 0;
    gPTPCmd.z = 0;
    gPTPCmd.rHead = 0;

    gPTPJumpParams.jumpHeight = 50;
    gPTPJumpParams.zLimit = 80;

    gQueuedCmdIndex = 0;
}

/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/
void Serialread()
{
  while(Serial1.available()) {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}


/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/

int Serial_putc( char c, struct __file * )
{
    Serial.write( c );
    return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:    
** Output parameters:
** Returned value:      
*********************************************************************************************************/

void printf_begin(void)
{
    fdevopen( &Serial_putc, 0 );
}

/*********************************************************************************************************
** Function name:       setSpeed
** Descriptions:        Change robot speed dependent on two variable
** Input parameters:    rspeed (requested speed), hspeed (high speed)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void setSpeed(uint8_t rspeed, uint8_t hspeed){
  
  gJOGCoordinateParams.velocity[0] = 2*rspeed*hspeed;
  gJOGCoordinateParams.velocity[1] = 2*rspeed*hspeed;
  gJOGCoordinateParams.velocity[2] = 2*rspeed*hspeed;
  gJOGCoordinateParams.velocity[3] = 2*rspeed*hspeed;
  gJOGCoordinateParams.acceleration[0] = 80;
  gJOGCoordinateParams.acceleration[1] = 80;
  gJOGCoordinateParams.acceleration[2] = 80;
  gJOGCoordinateParams.acceleration[3] = 80;
  SetJOGCoordinateParams(&gJOGCoordinateParams);

  gPTPCoordinateParams.xyzVelocity = 2*rspeed*hspeed;
  gPTPCoordinateParams.rVelocity = 2*rspeed*hspeed;
  gPTPCoordinateParams.xyzAcceleration = 20;
  gPTPCoordinateParams.rAcceleration = 20;
  SetPTPCoordinateParams(&gPTPCoordinateParams);
}
