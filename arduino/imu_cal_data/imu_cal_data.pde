#include <Wire.h>
#include <LSM303DLH.h>

LSM303DLH compass;

vector m_min;
vector m_max;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.enable();
  
  m_min.x = m_min.y = m_min.z = 0;
  m_max.x = m_max.y = m_max.z = 0;
}

void loop() {
  compass.read();

  if(compass.m.x < m_min.x)
    m_min.x = compass.m.x;
  
  if(compass.m.y < m_min.y)
    m_min.y = compass.m.y;
  
  if(compass.m.z < m_min.z)
    m_min.z = compass.m.z;
  
  
  if(compass.m.x > m_max.x)
    m_max.x = compass.m.x;
  
  if(compass.m.y > m_max.y)
    m_max.y = compass.m.y;
  
  if(compass.m.z > m_max.z)
    m_max.z = compass.m.z;
  
  Serial.print("min: ");
  Serial.print((int)m_min.x,DEC);
  Serial.print(" ");
  Serial.print((int)m_min.y,DEC);
  Serial.print(" ");
  Serial.print((int)m_min.z,DEC);
  Serial.print(" max: ");
  Serial.print((int)m_max.x,DEC);
  Serial.print(" ");
  Serial.print((int)m_max.y,DEC);
  Serial.print(" ");
  Serial.print((int)m_max.z,DEC);
  Serial.print("\n");
  
  
  delay(100);
}

