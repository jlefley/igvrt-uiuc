#include <Wire.h>
#include <LSM303DLH.h>

LSM303DLH compass;

int sonarPin=A0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.enable();
  
  // Calibration values
  compass.m_max.x = +353; compass.m_max.y = +532; compass.m_max.z = +454;
  compass.m_min.x = -813; compass.m_min.y = -651; compass.m_min.z = -529;
}

void loop() {
  compass.read();

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


  
  int heading = compass.heading((vector){0,-1,0});
  
  Serial.print(heading);

  Serial.print(",");
  int sonarValue = analogRead(sonarPin);
  Serial.print(sonarValue/2);

  Serial.print("\n");
  
  
  
  delay(100);
}

