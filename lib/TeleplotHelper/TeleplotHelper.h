#ifndef TELEPLOTHELPER_H
#define TELEPLOTHELPER_H
#include <Arduino.h>

// https://github.com/nesnes/teleplot/blob/main/README.md

// Helper class for sending data to Teleplot
// Use to make basic Teleplot functionality easier
// by wrapping common tasks in functions
// You can always send raw Teleplot commands using Serial.print()
class TeleplotHelper {
private:
  bool noPrint = false;
  HardwareSerial *serial;

  // text-based UDP packet format:
public:
  TeleplotHelper(HardwareSerial &s, bool noPrint = false);
  void plot(const char *teleplotUDPString);
  void plotXY(const char *variableName, float x, float y);
  void plotCube(const char *variableName, float w, float h, float d);
  void setNoPrint(bool value);
  bool getNoPrint();
};
#endif // TELEPLOTHELPER_H