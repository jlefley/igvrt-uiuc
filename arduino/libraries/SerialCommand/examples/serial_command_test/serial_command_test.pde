#include "TimerOne.h"
#include "SerialCommand.h"

#define LEDPIN 13                      // LED connected to digital pin 13

#define CMDSTART 'a'
#define CMDSTART_LEN 0

#define CMDDRIVE 'b'
#define CMDDRIVE_LEN 4

void setup()   {                
  // initialize the digital pin as an output:
  pinMode(LEDPIN, OUTPUT);

  // init timer
  Timer1.initialize(100000);
  Timer1.attachInterrupt(timerCB);
  
  // init serial
  SerialCmd.initialize(9600, serialDecode, serialExecute);
}

void timerCB()
{
  digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);
}

unsigned int serialDecode(unsigned char cmd)
{
  switch(cmd)
  {
    case CMDSTART:
      return CMDSTART_LEN;
    case CMDDRIVE:
      return CMDDRIVE_LEN;
  }
  
  return 0;
}

void serialExecute(unsigned char cmd, unsigned char * data)
{
  switch(cmd)
  {
    case CMDSTART:
      Serial.println("CMDSTART");
      break;
    case CMDDRIVE:
      Serial.println("CMDDRIVE");
      Serial.write(data, CMDDRIVE_LEN);
      break;
  }
}

void loop()
{
 SerialCmd.process();
}

