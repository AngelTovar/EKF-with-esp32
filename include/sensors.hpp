#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

namespace Sensors {
  bool begin();

  void imuEnableReports();
  bool imuGetEvent(sh2_SensorValue_t &evt);

  bool flowOK();
  bool tofOK();

  bool tofReadMeters(float &m);
  void flowReadCounts(int16_t &dx, int16_t &dy);
}
