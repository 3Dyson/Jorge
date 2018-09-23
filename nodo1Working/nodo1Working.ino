#include "mcp_can.h"
#include <SPI.h>

const int SPI_CS_PIN = 10; // CS pin arduino
unsigned char stmp[5] = {0, 0, 0, 0, 0}; // vector to send data over CAN 
unsigned char len = 0; // length of data 
unsigned char buf[5] = { 0, 0, 0, 0, 0}; // buffer to save data of CAN

int x1; // states
byte *ptr_x1 = (byte*)&x1;
int x2;
byte *ptr_x2 = (byte*)&x2;

unsigned long tauk = 0; 
unsigned long timeStamp = 0;                            
unsigned long tk = 0;
byte *ptr_tk = (byte*)&tk;

unsigned long TS1 = 0; // 'offset' of clock synchronization
unsigned long T_S1 = 0;
unsigned long TM1 = 0;
byte *ptr_TM1 = (byte*)&TM1;
unsigned long sysTime = 0;
long offset = 0;

char e[10]; // vectors to send data over serial port
char e1[10];
char e2[10];
char e3[10];
int marca = 1; // stamp of node activation 
long reset_timer = 0; // reset timer of plx daq

MCP_CAN CAN(SPI_CS_PIN); // set CS pin  

void datos() // read states of the plant
{
  x1 = analogRead(A0);
  x2 = analogRead(A1);

  timeStamp = millis() - offset - 1;                     
  tauk = tk - timeStamp;

  for (int i = 1; i >= 0; i--)
    stmp[i] = ptr_x1[i];

  for (int i = 1; i >= 0; i--)
    stmp[2 + i] = ptr_x2[i];

  stmp[4] = tauk;
  CAN.sendMsgBuf(0x02, 0, 5, stmp); // sends data to the controller 0x02
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

void enviaTauk() // sends tauk data over serial port
{ 
  Serial.print("DATA,TIME,");
  sysTime = millis() - offset - 1;
  dtostrf(sysTime, 5 , 3 , e2); // changes data to char
  dtostrf(tauk, 5 , 3 , e3);
  Serial.print(e2);Serial.print(","); 
  Serial.print(e3);Serial.println();
}

void setup()
{
  Serial.begin(38400);
  
  while (CAN_OK != CAN.begin(CAN_125KBPS)) // init can bus : baudrate = 125k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
  }
  Serial.println("CAN BUS Shield init ok! 'Sensor'");
  Serial.println("CLEARDATA"); // clear previous data
  Serial.println("LABEL, Hora, tiempo, Datos_S");
}

void loop()
{
  if (CAN_MSGAVAIL == CAN.checkReceive()) // checks incoming data
  {
    T_S1 = millis(); // message arrival time
    CAN.readMsgBuf(&len, buf); // save data in the buffer

    if (CAN.getCanId() == 0x01) // start to read and send the states
    {    
      //sysTime = millis()-offset-1; // time stamp to plx-daq
      for (int i = 3; i >= 0; i--)
        ptr_tk[i] = buf[i];
      datos();
      //enviaTauk(); // send data to plx-daq
      //enviaSchedule(); // 
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
