#include <Wire.h>
#include <LSM303.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Pin definitions for sonar sensors
// RX = Enable sensor
// ANALOG = Sensor output
#define CENTER_SONAR_RX 30
#define LEFT_SONAR_RX 31
#define RIGHT_SONAR_RX 32
#define CENTER_SONAR_ANALOG 0
#define LEFT_SONAR_ANALOG 1
#define RIGHT_SONAR_ANALOG 2

//Pin definitions for status light
#define STATUS_LIGHT 3

LSM303 compass;

boolean flash = false;

void setup() {
  pinMode(CENTER_SONAR_RX,OUTPUT);
  pinMode(LEFT_SONAR_RX,OUTPUT);
  pinMode(RIGHT_SONAR_RX,OUTPUT);
  pinMode(STATUS_LIGHT,OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  digitalWrite(STATUS_LIGHT,HIGH);
  
  // Initialize Timer1
    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
 
    // set compare match register to desired timer count:
    OCR1A = 15624;
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler:
    TCCR1B |= (1 << CS10);
    TCCR1B |= (1 << CS12);
    // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);
    // enable global interrupts:
    sei();
  
  
  // Old calibration values
  //compass.m_max.x = +353; compass.m_max.y = +532; compass.m_max.z = +454;
  //compass.m_min.x = -813; compass.m_min.y = -651; compass.m_min.z = -5429;
  
  // Calibration values
  compass.m_max.x = +451; compass.m_max.y = +654; compass.m_max.z = +494;
  compass.m_min.x = -816; compass.m_min.y = -621; compass.m_min.z = -557;
  
  //Allow sonar sensors to calibrate
  digitalWrite(CENTER_SONAR_RX,HIGH);
  digitalWrite(LEFT_SONAR_RX,HIGH);
  digitalWrite(RIGHT_SONAR_RX,HIGH);
  
  delay(1000);
  
  digitalWrite(CENTER_SONAR_RX,LOW);
  digitalWrite(LEFT_SONAR_RX,LOW);
  digitalWrite(RIGHT_SONAR_RX,LOW);
  
}

void loop() {
  
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    
    switch (incomingByte) {
      case 'a':
        flash = true;
        break;
      case 's':
        flash = false;
        break;
    }   
  }
  
  
  compass.read();
  
  int heading = compass.heading((LSM303::vector){0,-1,0});
  
  int sonarValueCenter = readSonar(CENTER_SONAR_RX,CENTER_SONAR_ANALOG,60);
  int sonarValueLeft = readSonar(LEFT_SONAR_RX,LEFT_SONAR_ANALOG,60);
  int sonarValueRight = readSonar(RIGHT_SONAR_RX,RIGHT_SONAR_ANALOG,60);

  Serial.print("$");
  Serial.print((int)compass.a.x,DEC);
  Serial.print(",");
  Serial.print((int)compass.a.y,DEC);
  Serial.print(",");
  Serial.print((int)compass.a.z,DEC); 
  Serial.print(",");
  Serial.print((int)compass.m.x,DEC);
  Serial.print(",");
  Serial.print((int)compass.m.y,DEC);
  Serial.print(",");
  Serial.print((int)compass.m.z,DEC);
  Serial.print(",");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(sonarValueCenter/2);
  Serial.print(",");
  Serial.print(sonarValueLeft/2);
  Serial.print(",");
  Serial.print(sonarValueRight/2);
  Serial.print("\n");
  
  delay(100);
}

int readSonar(int RX_pin, int AN_pin, int dly_time) {
  digitalWrite(RX_pin,HIGH);
  delay(dly_time);
  int sonarValue = analogRead(AN_pin);
  digitalWrite(RX_pin,LOW);
  return sonarValue;
}

ISR(TIMER1_COMPA_vect) {
  if (flash) {
    digitalWrite(STATUS_LIGHT, !digitalRead(STATUS_LIGHT));
  }
  else {
    digitalWrite(STATUS_LIGHT,HIGH);
  }
}
