#include "TeleplotHelper.h"

TeleplotHelper::TeleplotHelper(HardwareSerial &s, bool noPrint) {
  serial = &s;
  this->noPrint = noPrint;
}

void TeleplotHelper::setNoPrint(bool value) { this->noPrint = value; }
bool TeleplotHelper::getNoPrint() { return this->noPrint; }

void TeleplotHelper::plot(const char *teleplotUDPString) {
  if (!noPrint) {
    serial->println(teleplotUDPString);
  }
}
void TeleplotHelper::plotXY(const char *variableName, float x, float y) {
  if (!noPrint) {
    serial->print(">");
    serial->print(variableName);
    serial->print(":");
    serial->print(x);
    serial->print(",");
    serial->print(y);
    serial->println("|xy");
  }
}
void TeleplotHelper::plotCube(const char *variableName, float w, float h,
                              float d) {
  if (!noPrint) {
    serial->print(">3D|");
    serial->print(variableName);
    serial->print(":S:cube:W:10:D:10:H:10:Q:");
    serial->print(w);
    serial->print(":");
    serial->print(h);
    serial->print(":");
    serial->print(d);
    Serial.println(":C:red|");
  }
}
