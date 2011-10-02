
#include "SerialCommand.h"
#include <Servo.h>


//define modes of operation and amount of data intended to be received
#define MOTOR_CRTL 0xF1
#define MOTOR_CRTL_LEN 4

#define SENSOR_DATA_LEN 7

//timeout
#define TIMEOUT_MS 2000
TimeOut failSafe;

//variables to hold velocity values
int vR = 0;
int vL = 0;

//variable to hold sonar value
int range = 0;

//servo vars
Servo left_motor;
Servo right_motor;


void setup()
{
  // init serial
  SerialCmd.initialize(9600, serialDecode, serialExecute);
  
  // init failsafe timer
  failSafe.set(TIMEOUT_MS);
  
  // init servo objects for pwm comm to motors
  left_motor.attach(9);
  right_motor.attach(10);
  
  // output "zero velocity" pwm signal to motors for 2 seconds to allow sync
  left_motor.writeMicroseconds(1500);
  right_motor.writeMicroseconds(1500);
  
  delay(2000);
}

void writeMotorController(int vel_L, int vel_R)
{
  /*==============================================
   * RoboteQ motor controller
   * R/C mode, open loop
   * Expects "velocity" values from -100 to 100
   *==============================================*/
  
  int pulse_L = 0;
  int pulse_R = 0;
  
  // check velocity value ranges
  if(vel_L < -100)
    vel_L = -100;
  
  if(vel_L > 100)
    vel_L = 100;
  
  if(vel_R < -100)
    vel_R = -100;
  
  if(vel_R > 100)
    vel_R = 100;
  
  // map to proper PWM duty cycle values
  pulse_L = map(vel_L, -100, 100, 1000, 2000);
  pulse_R = map(vel_R, -100, 100, 1000, 2000);
  
  // set "measured" velocity values to set ones (this is open loop, so there is no measurement)
  //vL = vel_L;
  //vR = vel_R;
  
  // set "measured" velocity values to PWM duty cycle values (for diagnostic only)
  vL = pulse_L;
  vR = pulse_R;
  
  // set duty cycles on analog ports
  left_motor.writeMicroseconds(pulse_L);
  right_motor.writeMicroseconds(pulse_R);
}

unsigned int serialDecode(unsigned char cmd)
{
  switch(cmd)
  {
    case MOTOR_CRTL:
      return MOTOR_CRTL_LEN;
  }
  
  return 0;
}

void serialExecute(unsigned char cmd, unsigned char * data)
{
  switch(cmd)
  {
    case MOTOR_CRTL:

      writeMotorController((data[0] << 8) + data[1], (data[2] << 8) + data[3]);
      
      failSafe.clear();
      failSafe.set(TIMEOUT_MS);

      break;
      
  }
}

void sendSensorData()
{
  unsigned char sensor_data[SENSOR_DATA_LEN];
  
  sensor_data[0] = 0xFF;
  sensor_data[1] = vL >> 8;
  sensor_data[2] = vL & 0xFF;
  sensor_data[3] = vR >> 8;
  sensor_data[4] = vR & 0xFF;
  sensor_data[5] = range >> 8;
  sensor_data[6] = range & 0xFF;
      
  Serial.write(sensor_data,SENSOR_DATA_LEN);
}

void loop()
{
  if(failSafe.expired())
  {
    writeMotorController(0,0); 
    failSafe.clear();
  }
  
  while(SerialCmd.process());
  
  sendSensorData();
  
  delay(100);
}

