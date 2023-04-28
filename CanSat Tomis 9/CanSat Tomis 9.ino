#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <Adafruit_SI1145.h>
#include <MPU9250_asukiaaa.h>
#ifdef ESP32_HAL_I2C_H
#define SDA_PIN 21
#define SCL_PIN 22
#endif
#define ss 5
#define rst 14
#define dio0 2
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
int counter = 0;
#define SEALEVELPRESSURE_HPA (1013.25)
TinyGPSPlus gps;
Adafruit_BME280 bme;
Adafruit_SI1145 uv = Adafruit_SI1145();
MPU9250_asukiaaa mySensor;
unsigned long delayTime;
float gX, gY, gZ, aX, aY, aZ, aSqrt, mDirection;
uint16_t mX, mY, mZ;
void setup() {
  pinMode(2 , INPUT );
  Serial.begin(9600);
  Serial2.begin(9600);
    LoRa.setPins(ss, rst, dio0);
  bme.begin();  
  //int temp=bme.readTemperature();
    
  
    delayTime = 1000;
  if (!LoRa.begin(433E6)) {

    Serial.println("Starting LoRa failed!");

    while (1);

}
if (! uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
   Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginGyro();
  mySensor.beginAccel();
  mySensor.beginMag();
}
void updateSerial(){
  delay(500);
  while (Serial.available())  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}
void displayInfo()
{
    LoRa.beginPacket();
  float hum=bme.readHumidity();
  float temp=bme.readTemperature();
  float pres=bme.readPressure();
  float UVindex = uv.readUV();
  float x=analogRead(2);
  UVindex /= 100.0;
  if (gps.location.isValid()){

  if (gps.time.isValid())
  {
    if ((gps.time.hour()+3) < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour()+3);
    LoRa.print(gps.time.hour()+3);
    Serial.print(":");
    LoRa.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
     LoRa.print(gps.time.minute());
    LoRa.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    LoRa.print(gps.time.second());
    LoRa.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
     Serial.print(", ");
     LoRa.print(gps.time.centisecond());
     LoRa.print(", ");
  }
  else
  {
    Serial.print("Not Available");
  }
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(", H: ");
    Serial.print(hum);
       Serial.print(", T: ");
    Serial.print(temp);
       Serial.print(", P: ");
    Serial.print(pres);

      Serial.print(", Vis:"); Serial.print(uv.readVisible());
  Serial.print(", IR:"); Serial.print(uv.readIR());
  Serial.print(", UV:");  Serial.print(UVindex);

  mySensor.gyroUpdate();
  gX = mySensor.gyroX();gY = mySensor.gyroY();gZ = mySensor.gyroZ();
  mySensor.accelUpdate();
  aX = mySensor.accelX();aY = mySensor.accelY();aZ = mySensor.accelZ();aSqrt = mySensor.accelSqrt();
  mySensor.magUpdate();
  mX = mySensor.magX();mY = mySensor.magY();mZ = mySensor.magZ();mDirection = mySensor.magHorizDirection();
  
  Serial.print(", gX: ");Serial.print(gX);Serial.print(", gY: ");Serial.print(gY);  Serial.print(", gZ: ");Serial.print(gZ);
  Serial.print(", aX: ");Serial.print(aX);Serial.print(", aY: ");Serial.print(aY);Serial.print(", aZ: ");Serial.print(aZ);Serial.print(", aSqrt: ");Serial.print(aSqrt);
  Serial.print(", mX: ");Serial.print(mX);Serial.print(", mY: ");Serial.print(mY);Serial.print(", mZ: ");Serial.print(mZ);Serial.print(", mDir: ");Serial.print(mDirection);  
  Serial.print(", gas: ");Serial.println(x);   
  LoRa.print(gps.location.lat(), 6);
    LoRa.print(F(","));
    LoRa.print(gps.location.lng(), 6);
     LoRa.print(" ");
  LoRa.print(temp);
  LoRa.print(", ");
 LoRa.print(hum);
  LoRa.print(", ");
   LoRa.print(pres);
     LoRa.print(", ");
   LoRa.print(uv.readVisible());
    LoRa.print(", ");
LoRa.print(uv.readIR()); 
 LoRa.print(", ");
LoRa.print(UVindex);  
LoRa.print(", ");LoRa.print(gX);LoRa.print(", ");LoRa.print(gY);  LoRa.print(", ");LoRa.print(gZ);
  LoRa.print(", ");LoRa.print(aX);LoRa.print(", ");LoRa.print(aY);LoRa.print(", ");LoRa.print(aZ);LoRa.print(", ");LoRa.print(aSqrt);
  //LoRa.print(", ");LoRa.print(mX);LoRa.print(", ");LoRa.print(mY);LoRa.print(", ");LoRa.print(mZ);LoRa.print(", ");LoRa.print(mDirection);

   LoRa.endPacket();
   
    delay(395);
  }
  else
  {
    Serial.print(F("INVALID"));
LoRa.print(F("INVALID"));    
   Serial.print(", H: ");
    Serial.print(hum);
       Serial.print(", T: ");
    Serial.print(temp);
       Serial.print(", P: ");
    Serial.print(pres);
      Serial.print(", Vis:"); Serial.print(uv.readVisible());
  Serial.print(", IR:"); Serial.print(uv.readIR());
  Serial.print(", UV:");  Serial.print(UVindex);
    mySensor.gyroUpdate();
  gX = mySensor.gyroX();gY = mySensor.gyroY();gZ = mySensor.gyroZ();
  mySensor.accelUpdate();
  aX = mySensor.accelX();aY = mySensor.accelY();aZ = mySensor.accelZ();aSqrt = mySensor.accelSqrt();
  mySensor.magUpdate();
  mX = mySensor.magX();mY = mySensor.magY();mZ = mySensor.magZ();mDirection = mySensor.magHorizDirection();
  
  Serial.print(", gX: ");Serial.print(gX);Serial.print(", gY: ");Serial.print(gY);  Serial.print(", gZ: ");Serial.print(gZ);
  Serial.print(", aX: ");Serial.print(aX);Serial.print(", aY: ");Serial.print(aY);Serial.print(", aZ: ");Serial.print(aZ);Serial.print(", aSqrt: ");Serial.print(aSqrt);
  Serial.print(", mX: ");Serial.print(mX);Serial.print(", mY: ");Serial.print(mY);Serial.print(", mZ: ");Serial.print(mZ);Serial.print(", mDir: ");Serial.print(mDirection);  
  Serial.print(", gas: ");Serial.print(x);   
    LoRa.print(", ");
  LoRa.print(temp);
  LoRa.print(", ");
 LoRa.print(hum);
  LoRa.print(", ");
   LoRa.print(pres);
     LoRa.print(", ");  
      LoRa.print(uv.readVisible());
    LoRa.print(", ");
LoRa.print(uv.readIR()); 
 LoRa.print(", ");
LoRa.print(UVindex); 
LoRa.print(", ");LoRa.print(gX);LoRa.print(", ");LoRa.print(gY);  LoRa.print(", ");LoRa.print(gZ);
LoRa.print(", ");LoRa.print(aX);LoRa.print(", ");LoRa.print(aY);LoRa.print(", ");LoRa.print(aZ);LoRa.print(", ");LoRa.print(aSqrt);
  //LoRa.print(", ");LoRa.print(mX);LoRa.print(", ");LoRa.print(mY);LoRa.print(", ");LoRa.print(mZ);LoRa.print(", ");LoRa.println(mDirection);

    LoRa.endPacket();
    delay(395);
  }
}
void loop() {
  //updateSerial();
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.print(F("No GPS detected"));
        LoRa.beginPacket();
    float hum=bme.readHumidity();
      float temp=bme.readTemperature();
  float pres=bme.readPressure();
  float UVindex = uv.readUV();
  float x=analogRead(2);
  UVindex /= 100.0;
   Serial.print(", H: ");
    Serial.print(hum);
       Serial.print(", T: ");
    Serial.print(temp);
       Serial.print(", P: ");
    Serial.print(pres);
      Serial.print(", Vis:"); Serial.print(uv.readVisible());
  Serial.print(", IR:"); Serial.print(uv.readIR());
  Serial.print(", UV:");  Serial.print(UVindex);
    mySensor.gyroUpdate();
  gX = mySensor.gyroX();gY = mySensor.gyroY();gZ = mySensor.gyroZ();
  mySensor.accelUpdate();
  aX = mySensor.accelX();aY = mySensor.accelY();aZ = mySensor.accelZ();aSqrt = mySensor.accelSqrt();
  mySensor.magUpdate();
  mX = mySensor.magX();mY = mySensor.magY();mZ = mySensor.magZ();mDirection = mySensor.magHorizDirection();
  
  Serial.print(", gX: ");Serial.print(gX);Serial.print(", gY: ");Serial.print(gY);  Serial.print(", gZ: ");Serial.print(gZ);
  Serial.print(", aX: ");Serial.print(aX);Serial.print(", aY: ");Serial.print(aY);Serial.print(", aZ: ");Serial.print(aZ);Serial.print(", aSqrt: ");Serial.print(aSqrt);
  Serial.print(", mX: ");Serial.print(mX);Serial.print(", mY: ");Serial.print(mY);Serial.print(", mZ: ");Serial.print(mZ);Serial.print(", mDir: ");Serial.println(mDirection);  
  Serial.print(", gas: ");Serial.println(x);   
     LoRa.print(" ");
  LoRa.print(temp);
  LoRa.print(", ");
 LoRa.print(hum);
  LoRa.print(", ");
   LoRa.print(pres); 
     LoRa.print(", ");
      LoRa.print(uv.readVisible());
    LoRa.print(", ");
LoRa.print(uv.readIR()); 
 LoRa.print(", ");
LoRa.print(UVindex);  
LoRa.print(", ");LoRa.print(gX);LoRa.print(", ");LoRa.print(gY);  LoRa.print(", ");LoRa.print(gZ);
  LoRa.print(", ");LoRa.print(aX);LoRa.print(", ");LoRa.print(aY);LoRa.print(", ");LoRa.print(aZ);LoRa.print(", ");LoRa.print(aSqrt);
  //LoRa.print(", ");LoRa.print(mX);LoRa.print(", ");LoRa.print(mY);LoRa.print(", ");LoRa.print(mZ);LoRa.print(", ");LoRa.println(mDirection);

  LoRa.endPacket();
    delay(395);
  }
}