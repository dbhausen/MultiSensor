#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include <Adafruit_LSM6DS3.h>

#include <QMC5883LCompass.h>
#include <TeleplotHelper.h>
#include <TinyGPS++.h>

void printCompass(char direction[3], int x, int y, int z, int azimuth);
void myPrintAccelerometer(sensors_event_t &accel, sensors_event_t &gyro,
                          sensors_event_t &temp);
void printGPSData();

Adafruit_LSM6DS3 lsm6ds3;
QMC5883LCompass compass;

TinyGPSPlus gps;

unsigned long gpsCount = 0;
unsigned long compassCount = 0;
unsigned long previousMillis = 0;
unsigned long previousLedMillis = 0;
const long interval = 250;
const long ledInterval = 1000;
bool ledState = LOW;
bool compassInitialized = false;
bool declinationSet = false;

TeleplotHelper tp = TeleplotHelper(Serial, false);

SoftwareSerial gpsSerial(4, 3); // RX, TX

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println("LSM6DS3 Test + GPS Test + Compass Test ");

  // Initialize sensor
  if (lsm6ds3.begin_I2C(0x6B)) {
    Serial.println("LSM6DS3 Found!");
  } else {
    Serial.println("Failed to find LSM6DS3 chip");
  }

  // Initialize compass
  compass.init();
  compass.setCalibration(-2813, 81, -1861, 1038, -1342, 737);
  compass.setMagneticDeclination(11, 3); // 11° 3' W
  compass.setSmoothing(5, true);
  Serial.println("QMC5883L Compass initialized!");
}

void loop() {

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isUpdated()) {
        // printGPSData();
        gpsCount++;
      }
    }
  }

  unsigned long currentMillis = millis();
  float i = 0.0;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sensors_event_t accel, gyro, temp;
    if (lsm6ds3.getEvent(&accel, &gyro, &temp)) {
      // myPrintAccelerometer(accel, gyro, temp);
    } else {
      Serial.println("Failed to read from LSM6DS3 sensor!");
    }

    // Read compass data
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();
    int azimuth = compass.getAzimuth();
    char direction[3];
    compass.getDirection(direction, azimuth);
    Serial.print("Compass Direction: ");
    Serial.println(direction);

    i += 1;

    // Serial.print(">3D|my_super_cube2:S:cube:W:5:D:5:H:5:");
    // Serial.print(":R:");
    // // Serial.print(x);
    // Serial.print(":");
    // Serial.print(y);
    // Serial.print(":");
    // // Serial.print(z);
    // Serial.println(":C:red|");
    // tp.plotXY("Compass", x, y);
    // tp.plotCube("Compass3D", x, y, z);

    compassCount++;
  }

  if (currentMillis - previousLedMillis >= ledInterval) {
    previousLedMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    Serial.print("GPS Reads: ");
    Serial.print(gpsCount);
    Serial.print(" | Compass Reads: ");
    Serial.println(compassCount);
  }
}

void printCompass(char direction[3], int x, int y, int z, int azimuth) {
  Serial.print("Direction: ");
  Serial.print(direction);
  Serial.print(" | ");
  Serial.print("Compass X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.print(" Azimuth: ");
  Serial.print(azimuth);
  Serial.println("°");
}

void myPrintAccelerometer(sensors_event_t &accel, sensors_event_t &gyro,
                          sensors_event_t &temp) {

  float fahrenheit = (temp.temperature * 9.0 / 5.0) + 32.0;

  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" Z: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temp: ");
  Serial.print(temp.temperature);
  Serial.println(" C");

  Serial.print("Temp: ");
  Serial.print(fahrenheit);
  Serial.println(" F");

  Serial.println();
}
void printGPSData() {
  if (gps.location.isValid()) {

    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Lng: ");
    Serial.println(gps.location.lng(), 6);

    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("GPS: Waiting for fix...");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print("Date: ");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.year());
    Serial.print(" Time: ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  }

  Serial.println();
}
