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
// CFG-RATE packet template (little-endian periods/ratios)
static const uint8_t CFG_RATE[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // Class = CFG, ID = RATE
    0x06, 0x00, // Length = 6 bytes
    0xE8, 0x03, // measRate (1000 ms default)
    0x01, 0x00, // navRate (1 cycle)
    0x01, 0x00, // timeRef (0=UTC,1=GPS,2=GLONASS)
    0x00, 0x00  // checksum placeholder
};

void sendCfgRate(Stream &gps, uint16_t desired_ms) {
  uint8_t packet[sizeof(CFG_RATE)];
  memcpy(packet, CFG_RATE, sizeof(CFG_RATE));

  // measRate: little-endian milliseconds
  packet[6] = desired_ms & 0xFF;
  packet[7] = desired_ms >> 8;

  // recompute Fletcher-8 checksum over class+ID+length+payload
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < sizeof(packet) - 2; ++i) {
    ckA += packet[i];
    ckB += ckA;
  }
  packet[sizeof(packet) - 2] = ckA;
  packet[sizeof(packet) - 1] = ckB;

  gps.write(packet, sizeof(packet));
}
// UBX-CFG-CFG template (length = 13 bytes payload)
static uint8_t CFG_CFG[] = {
    0xB5, 0x62, // sync chars
    0x06, 0x09, // class = CFG, id = CFG
    0x0D, 0x00, // payload length = 13

    // clearMask (none)
    0x00, 0x00, 0x00, 0x00,
    // saveMask: save all configuration blocks
    0xFF, 0xFF, 0x00, 0x00,
    // loadMask (don't load anything)
    0x00, 0x00, 0x00, 0x00,
    // deviceMask: bit0=BBR, bit1=Flash, bit2=EEPROM, bit3=SPI Flash
    0x03, // save to both BBR and Flash when available

    0x00, 0x00 // checksum placeholders
};

void sendCfgCfgSave(Stream &gps) {
  uint8_t packet[sizeof(CFG_CFG)];
  memcpy(packet, CFG_CFG, sizeof(CFG_CFG));

  // recompute checksum over class..payload
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < sizeof(packet) - 2; ++i) {
    ckA += packet[i];
    ckB += ckA;
  }
  packet[sizeof(packet) - 2] = ckA;
  packet[sizeof(packet) - 1] = ckB;

  gps.write(packet, sizeof(packet));
}
static const uint8_t CFG_RATE_POLL[] = {
    0xB5, 0x62, // sync
    0x06, 0x08, // class, id
    0x00, 0x00, // payload length = 0 (poll request)
    0x0E, 0x30  // checksum over class+id+len (precomputed)
};

bool readCfgRate(Stream &gps, uint16_t &measRateMs, uint16_t &navRate,
                 uint16_t &timeRef, unsigned long timeoutMs = 200) {
  gps.write(CFG_RATE_POLL, sizeof(CFG_RATE_POLL));

  static const uint8_t header[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00};
  const size_t headerLen = sizeof(header);
  const size_t expected = headerLen + 6 + 2; // header + payload + checksum
  uint8_t buffer[expected];
  size_t index = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (!gps.available()) {
      continue;
    }

    uint8_t byte = gps.read();
    if (index < headerLen) {
      if (byte == header[index]) {
        buffer[index++] = byte;
      } else {
        index = (byte == header[0]) ? 1 : 0;
        if (index == 1) {
          buffer[0] = byte;
        }
      }
      continue;
    }

    buffer[index++] = byte;
    if (index == expected) {
      break;
    }
  }

  if (index != expected) {
    Serial.println("Timeout waiting for CFG-RATE response");
    return false; // timed out before full packet arrived
  }

  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < expected - 2; ++i) {
    ckA += buffer[i];
    ckB += ckA;
  }
  if (ckA != buffer[expected - 2] || ckB != buffer[expected - 1]) {
    Serial.println("CFG-RATE checksum mismatch");
    return false;
  }

  measRateMs = buffer[6] | (buffer[7] << 8);
  navRate = buffer[8] | (buffer[9] << 8);
  timeRef = buffer[10] | (buffer[11] << 8);
  return true;
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  delay(2000);

  // sendCfgRate(gpsSerial, 1000); // Set GPS to 1 Hz updates
  // sendCfgCfgSave(gpsSerial);    // Save configuration to flash
  Serial.println("LSM6DS3 Test + GPS Test + Compass Test ");
  uint16_t measRate, navRate, timeRef;
  if (readCfgRate(gpsSerial, measRate, navRate, timeRef)) {
    Serial.print("measRate(ms): ");
    Serial.print(measRate);
    Serial.print(" navRate: ");
    Serial.print(navRate);
    Serial.print(" timeRef: ");
    Serial.println(timeRef == 0 ? "UTC" : "GPS");
  } else {
    Serial.println("Failed to read CFG-RATE from GPS");
  }

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

  // uint16_t measRate, navRate, timeRef;
  // if (readCfgRate(gpsSerial, measRate, navRate, timeRef)) {
  //   Serial.print("measRate(ms): ");
  //   Serial.print(measRate);
  //   Serial.print(" navRate: ");
  //   Serial.print(navRate);
  //   Serial.print(" timeRef: ");
  //   Serial.println(timeRef == 0 ? "UTC" : "GPS");
  // } else {
  //   Serial.println("Failed to read CFG-RATE from GPS");
  // }
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
    Serial.println(azimuth);
    compass.getBearing(azimuth);

    i += 1;
    // bad change
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
