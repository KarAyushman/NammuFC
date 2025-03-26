#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>

SoftwareSerial ss(16,17);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

/*

Serial -> UART -> RX, TX -> (16,17)
I2C -> MPU6050 & BMP280 -> SDA, SCL -> (21, 22)
SPI -> SD -> MISO, MOSI, SCK, CS -> (19, 23, 18, 5)

*/

float lat, longi, utcTime, utcDate, gpsSats, gpsAlt, gpsSpd;

void gpsInit(){
  Serial.println("Starting GPS Sensor...");
  ss.begin(9600);
  Serial.println("GPS Sensor Started Succesfully!");
}

void gpsData(){
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      lat = gps.location.lat();
      longi = gps.location.lng();
      utcTime = gps.time.value();
      utcDate = gps.date.value();
      gpsSats = gps.satellites.value();
      gpsAlt = gps.altitude.meters();
      gpsSpd = gps.speed.mps();
    }
  }
}

/*
  Acceleration Units - m/s^2, G
  1G = 9.816m/s^2

  Gyroscope Units - dps, rps
*/

float aX, aY, aZ, gX, gY, gZ;
float pitch, yaw;
float mpuTemp;

void mpuInit(){
  Serial.println("Starting MPU6050 Sensor...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 Failed. Retrying...");
    mpuInit();
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Started Succesfully!");
}

void mpuData(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  aX = a.acceleration.x;
  aY = a.acceleration.y;
  aZ = a.acceleration.z;
  gX = g.gyro.x;
  gY = g.gyro.y;
  gZ = g.gyro.z;
  pitch = atan2(-aX,sqrt(aY*aY + aZ*aZ));
  yaw = atan2(aY,sqrt(aX*aX + aZ*aZ));
  mpuTemp = temp.temperature;
}

/*
  Pressure Units - Pa (SI), psi, atm, bar, mmHg
  1 atm = 101325 Pa = 1013.25 bar = 29.92mmHg
*/

float pressure, altMSL, altAGL, bmpTemp;

void bmpInit(){
  Serial.println("Starting BMP280 Sensor...");
  if (!bmp.begin()) {
    Serial.println("BMP280 Failed. Retrying...");
    bmpInit();
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,                   /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,                  /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,                    /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_1);                 /* Standby time. */

  pressure = 0.0;
  altMSL = 0.0;
  for(int i=0;i<20;i++){
    pressure += bmp.readPressure()/20;
    delay(50);
  }
  for(int i=0;i<20;i++){
    altMSL += bmp.readAltitude(pressure/100)/20;
    delay(50);
  }
  Serial.println("BMP280 Started Succesfully!");
}

void bmpData(){
  altAGL = bmp.readAltitude(pressure/100) - altMSL;
  bmpTemp = bmp.readTemperature();
  pressure = bmp.readPressure();
}

void setup(){
  Serial.begin(9600);
  Wire.begin();
  gpsInit();
  mpuInit();
  bmpInit();
}

void loop(){
  gpsData();
  mpuData();
  bmpData();
  Serial.print("Pitch: ");
  Serial.println(pitch);
  Serial.print("Yaw: ");
  Serial.println(yaw);
}