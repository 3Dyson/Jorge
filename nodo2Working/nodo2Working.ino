#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 10; // CS pin arduino              
unsigned char len = 0; // length of data
unsigned char buf[5] = {0, 0, 0, 0, 0}; // buffer to save data of CAN
unsigned char stmp[5] = {0, 0, 0, 0, 0}; // vector to send data over CAN

float K1 = 0.0; // gains
float K2 = 0.0;
float tauk = 0.0;
const float v_max = 3.45;  
int pin_r = 6; // reference output pin 

#define refPos  0.5 // positive an negative values for the reference
#define refNeg  -0.5

int topCount = 100; // period to change the reference (actuationPeriod = 0.012)
unsigned int count = 0; // T_ref = 2 * topCount*actuationPeriod 

unsigned long sysTime = 0;

float r = refNeg; // reference
unsigned char *ptr_r = (unsigned char*)&r;

int x1 = 0; // state x1 at the output of the second integrator
unsigned char *ptr_x1 = (unsigned char*)&x1;

int x2 = 0; // state x2 at the output of the first integrator
unsigned char *ptr_x2 = (unsigned char*)&x2;

float x11 = 0.0; // state x1 
unsigned char *ptr_x11 = (unsigned char*)&x11;

float x22 = 0.0; // state x2
unsigned char *ptr_x22 = (unsigned char*)&x22;

int uk = 0; // control
float uk1 = 0.0; // uk-1
unsigned char *ptr_uk1 = (unsigned char*)&uk1;

float x_hat_11 = 0.0;
float x_hat_21 = 0.0;

float B_11 = 0.03259; // matrix B
float B_21 = -0.2553;

float A_11 = 1.0; // matrix A
float A_12 = -0.2553;
float A_21 = 0.0;
float A_22 = 1.0;

unsigned long TS1 = 0; // 'offset' of clock synchronization
unsigned long T_S1 = 0;
unsigned long TM1 = 0;
byte *ptr_TM1 = (byte*)&TM1;
long offset = 0;

char e[10]; // vectors to send data over serial port
char e1[10];
char e2[10];
int marca = 1; // stamp of node activation
long reset_timer = 0; // reset timer of plx daq

MCP_CAN CAN(SPI_CS_PIN); // set CS pin

void calculateControl() 
{
  for (int i = 1; i >= 0; i--)
    ptr_x1[i] = buf[i];

  for (int i = 1; i >= 0; i--)
    ptr_x2[i] = buf[2 + i];

  tauk = (float)buf[4] / 1000;

  x11 = ((float)x1 * 5 / 1024) - v_max / 2 ;
  x22 = ((float)x2 * 5 / 1024) - v_max / 2 ;

  A_12 = -23.8095 * tauk; // polynomials for matrix elements
  B_11 = 283.4461 * (tauk * tauk);
  B_21 = A_12;

  // polynomials for controller gains
  K1 = 44399448.08 * (tauk * tauk * tauk * tauk) - 2949673.35 * (tauk * tauk * tauk) + 84402.9 * (tauk * tauk) - 1414.04 * tauk + 14.0780;
  K2 = -22103894.7 * (tauk * tauk * tauk * tauk) + 1465116.84 * (tauk * tauk * tauk) - 41985.38 * (tauk * tauk) + 725.74 * tauk - 8.8157;

  x_hat_11 = ((A_11 * x11) + (A_12 * x22)) + (B_11 * uk1);
  x_hat_21 = ((A_21 * x11) + (A_22 * x22)) + (B_21 * uk1);
  
  uk1 = (-K1 * (x_hat_11 - r)) - (K2 * x_hat_21);
  
  if (uk1 > 1.65)
      uk1 = 1.65;

  if (uk1 < -1.65)
      uk1 = -1.65;
  
  uk = uk1;
  uk = (uk / v_max) * 168 + 84;     
  uk = (int)uk;
  
  stmp[0] = uk;
  CAN.sendMsgBuf(0x03, 0, 1, stmp); // sends control action to the actuator 0x03
}

void ref()
{
  count++;
  
  if (count >= topCount) // change reference
  {
    digitalWrite(pin_r, digitalRead(pin_r) ^ 1);
    count = 0;

    if (r == refPos) 
    {
      r = refNeg;
    }
    else if (r == refNeg)
    {
      r = refPos;
    }
  }
}

void enviaSchedule() // sends schedule data over serial port
{ 
  Serial.print("DATA,TIME,");
  dtostrf(sysTime, 5 , 3 , e1); // changes data to char
  dtostrf(marca, 5 , 3 , e); 
  Serial.print(e1);Serial.print(",");
  Serial.println(e);Serial.println();
}

void enviaSinc() // sends synchronization data over serial port
{ 
  if (reset_timer == 1)
    Serial.println("RESETTIMER"); // reset timer of plx daq
     
  Serial.print("DATA,TIME,TIMER,");
  sysTime = millis() - offset - 1;
  dtostrf(sysTime, 5 , 3 , e2); // changes data to char
  Serial.print(e2);Serial.println();
}

void setup()
{
  Serial.begin(38400);
  pinMode(pin_r, OUTPUT); // digital output of reference
  
  while (CAN_OK != CAN.begin(CAN_125KBPS)) // init can bus : baudrate = 125k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!'Controlador'");
  Serial.println("CLEARDATA"); // clear previous data
  Serial.println("LABEL, Hora, tiempo, Datos_C");
}

void loop()
{
  if (CAN_MSGAVAIL == CAN.checkReceive()) // checks incoming data
  { 
    T_S1 = millis(); // message arrival time
    CAN.readMsgBuf(&len, buf); // save data in the buffer

    if (CAN.getCanId() == 0x02) // start to calculate control
    {
      //sysTime = millis()-offset-1; // time stamp to plx-daq
      calculateControl();
      ref();
      //enviaSchedule(); // send data to plx-daq
    }

    if (CAN.getCanId() == 0x00) // clock synchronization of the first message 0x00
    {
      if (buf[4] == 0) {
        for (int i = 3; i >= 0; i--)
          ptr_TM1[i] = buf[i];
        TS1 = T_S1;
      }

      if (buf[4] == 1) // clock synchronization of the second message 0x00
      {
        for (int i = 3; i >= 0; i--)
          ptr_TM1[i] = buf[i];
        offset = TS1 - TM1; 
        //reset_timer += 1; // send data to plx-daq
        //enviaSinc();  //
      }
    }
  }
}
