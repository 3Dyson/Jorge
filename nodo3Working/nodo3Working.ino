#include <SPI.h>
#include "mcp_can.h"
#include <avr/io.h>
#include <avr/interrupt.h>

const int SPI_CS_PIN = 10; // CS pin arduino 
const int pwm1 = 6; // pwm output
int h = 12; // actuation period
long tiempo_anterior = -h;                           

int uk = 0; // control action
unsigned char len = 0; // length of data
unsigned char buf[4] = {0, 0, 0, 0}; // buffer to save data of CAN                 
unsigned char stmp0[6] = {0, 0, 0, 0, 0, 0}; // vectors to send data over CAN         
unsigned char stmp1[4] = {0, 0, 0, 0};               

unsigned long tk = 0; // actuation instant
byte *ptr_tk = (byte*)&tk;                           

unsigned long message_time = 0; // time to synchronization
byte *ptr_message = (byte*)&message_time;            
                              
int start = 0; // system start stamp
int inicio = 14; // start button
int marca = 1; // stamp of node activation 
char e[10]; // vectors to send data over serial port
char e1[2];                                          
char e2[10];                                         
long reset_timer = 0; // reset timer of plx daq 

MCP_CAN CAN(SPI_CS_PIN); // set CS pin 

void enviaSchedule() // sends schedule data over serial port                               
{ 
  Serial.print("DATA,TIME,");
  dtostrf(message_time, 5 , 3 , e1); // changes data to char
  dtostrf(marca, 5 , 3 , e);                         
  Serial.print(e1); Serial.print(",");
  Serial.println(e); Serial.println();
}

void enviaSinc() // sends synchronization data over serial port
{ 
  if (reset_timer == 1)            
    Serial.println("RESETTIMER"); // reset timer of plx daq
     
  Serial.print("DATA,TIME,TIMER,");          
  dtostrf(millis(), 5 , 3 , e2); // changes data to char
  Serial.print(e2); Serial.println();
}

void setup() 
{
  cli(); // disable interrupts 
  Serial.begin(38400);
    
  while (CAN_OK != CAN.begin(CAN_125KBPS)) // init can bus : baudrate = 125k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
  }
  Serial.println("CAN BUS Shield init ok!'Actuador'");  
  Serial.println("CLEARDATA"); // clear previous data
  Serial.println("LABEL, Hora, tiempo, Datos_A");     

  // int1 set to 2 sec 
  // TCNT = 2^(res.timer)-((period [sec] * Tosc)/ preescale)
  // TCNT = 65535 - ((2 * 16 Mhz)/1024)
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = (0 << OCIE1B) | (0 << OCIE1A) | (1 << TOIE1);
  TCCR1B |= (1 << CS10);     
  TCCR1B |= (0 << CS11);
  TCCR1B |= (1 << CS12);
  TCNT1 = 34285; 
  OCR1A = 0x00;
  OCR1B = 0x00;

  // int2 set to 12ms
  // TCNT = 2^(res.timer)-((period [sec] * Tosc)/ preescale)
  // TCNT = 255 - ((0.012 * 16 Mhz)/1024)
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (1 << TOIE2);
  TCCR2B |= (1 << CS20);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS22);
  TCNT2 = 67;                                   
  OCR2A = 0x00;                                 
  OCR2B = 0x00;                                 
  sei(); // enable interruptions
}

ISR(TIMER1_OVF_vect) // starts to synchronization
{
    TCNT1 = 34285; // timer value 
    message_time = millis(); // time to synchronize
    for (int i = 3; i >= 0; i--)        
      stmp0[i] = ptr_message[i];
    stmp0[4] = 0;
    CAN.sendMsgBuf(0x00, 0, 5, stmp0); // sends the first synchronization mesagge  0x00

    message_time = millis(); // exact time that message was sent
    for (int i = 3; i >= 0; i--)        
      stmp0[i] = ptr_message[i];
    stmp0[4] = 1;
    CAN.sendMsgBuf(0x00, 0, 6, stmp0); // sends the second synchronization mesagge  0x00
    
    //reset_timer += 1; // sends data to plx-daq
    //enviaSinc(); //
    
    if (start > 0) {                    
      start += 1;                        
  }                                      
}

ISR(TIMER2_OVF_vect)
{
  TCNT2 = 67; // timer value 
  if (start > 0)
  {
    analogWrite(pwm1, uk); // control action to the plant
    message_time = millis();             
    tk += message_time - tiempo_anterior; // tk = tk + h 
    tiempo_anterior = message_time;       

    for (int i = 3; i >= 0; i--)
      stmp1[i] = ptr_tk[i];

    CAN.sendMsgBuf(0x01, 0, 4, stmp1); // sends tk message to the sensor 0x01
    //enviaSchedule(); // send data to plx-daq
 }
}

void loop() 
{ 
  if (digitalRead(inicio) == HIGH) // system start
      start = 1;
  delay(1);
  
  if (start >= 10)
  {
    start = 0;
  }

  if (CAN_MSGAVAIL == CAN.checkReceive()) // check incoming data
  {
    CAN.readMsgBuf(&len, buf); // save data in the buffer
    if (CAN.getCanId() == 0x03) {
     uk = buf[0];
    }
  }
}
