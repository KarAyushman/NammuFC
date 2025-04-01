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

float lati, longi, utcTime, utcDate, gpsSats, gpsAlt, gpsSpd;

void gpsInit(){
  Serial.println("Starting GPS Sensor...");
  ss.begin(9600);
  Serial.println("GPS Sensor Started Succesfully!");
}

void gpsData(){
  unsigned long startTime = millis();
  
  // Process available GPS data for up to 1000ms
  while (ss.available() > 0 && millis() - startTime < 1000) {
    char c = ss.read();
    // Uncomment this line to see raw NMEA data
    // Serial.write(c); 
    gps.encode(c);
  }
  
  // Update variables regardless of whether location is updated
  lati = gps.location.isValid() ? gps.location.lat() : 0.0;
  longi = gps.location.isValid() ? gps.location.lng() : 0.0;
  utcTime = gps.time.isValid() ? gps.time.value() : 0.0;
  utcDate = gps.date.isValid() ? gps.date.value() : 0.0;
  gpsSats = gps.satellites.isValid() ? gps.satellites.value() : 0.0;
  gpsAlt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  gpsSpd = gps.speed.isValid() ? gps.speed.mps() : 0.0;
  
  // Add diagnostic information
  Serial.print("GPS chars processed: ");
  Serial.println(gps.charsProcessed());
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
  pitch = atan2(-aX,sqrt(aY*aY + aZ*aZ))/3.14*180;
  yaw = atan2(aY,sqrt(aX*aX + aZ*aZ))/3.14*180;
  mpuTemp = temp.temperature;
}

/*
  Pressure Units - Pa (SI), psi, atm, bar, mmHg
  1 atm = 101325 Pa = 1013.25 bar = 29.92mmHg
*/

float pressure, altMSL, altAGL, bmpTemp;

void bmpInit(){
  Serial.println("Starting BMP280 Sensor...");
  if (!bmp.begin(0x76)) {
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
  Serial.begin(115200);  // Increased baud rate for faster serial communication
  Wire.begin();
  
  Serial.println("Initializing all sensors...");
  gpsInit();
  mpuInit();
  bmpInit();
  
  Serial.println("All sensors initialized. Beginning data collection...");
  Serial.println("-------------------");
}

void loop(){
  gpsData();
  mpuData();
  bmpData();

  Serial.println("Mehak Rustagi 22MID0026");
  Serial.println("Pallavi Yadav 22MID0028");
  
  
  // MPU6050 data
  Serial.print("Pitch & Yaw: ");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);
  Serial.print("MPU TEMP: ");
  Serial.println(mpuTemp);
  Serial.print("Gyro x: ");
  Serial.println(gX);
  Serial.print("Gyro y: ");
  Serial.println(gY);
  Serial.print("Gyro z: ");
  Serial.println(gZ);
  Serial.print("Acc x: ");
  Serial.println(aX);
  Serial.print("Acc y: ");
  Serial.println(aY);
  Serial.print("Acc z: ");
  Serial.println(aZ);

  // BMP280 data
  Serial.print("Pressure (Pa): ");
  Serial.println(pressure);
  Serial.print("Altitude AGL (m): ");
  Serial.println(altAGL);
  Serial.print("BMP TEMP: ");
  Serial.println(bmpTemp);
  
  // GPS data
  Serial.print("GPS Location: ");
  Serial.print(lati);
  Serial.print(",");
  Serial.println(longi);
  Serial.print("GPS Altitude (m): ");
  Serial.println(gpsAlt);
  Serial.print("GPS Speed (m/s): ");
  Serial.println(gpsSpd);
  Serial.print("Satellites: ");
  Serial.println(gpsSats);
  Serial.print("UTC Time/Date: ");
  Serial.print(utcTime);
  Serial.print("/");
  Serial.println(utcDate);
  
  Serial.println("-------------------");
  delay(1000);  // Adding a small delay to make the output readable
}
