#include "sensors.hpp"
#include "config.h"
#include "calib.hpp"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <Bitcraze_PMW3901.h>
#include <VL53L1X.h>

namespace {
  Adafruit_BNO08x bno08x;
  Bitcraze_PMW3901 flow(PMW_CS);
  VL53L1X tof;

  bool g_flow_ok=false, g_tof_ok=false;
}

namespace Sensors {

//BNO08x
static void imuBegin(){
  // I2C robusto
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);      // 400 kHz
  Wire.setTimeOut(50);        // evita bloqueos largos en ESP32

  if (!bno08x.begin_I2C(BNO08X_ADDR, &Wire)) {
    // Se puede reintentar, pero dejamos que begin() global falle
    return;
  }
  imuEnableReports();
}

void imuEnableReports(){
  // Habilitar SOLO lo necesario y a tasa razonable
  const uint32_t us = (uint32_t)(1e6f / CAL.imu_report_rate_hz);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, us);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, us);
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, us); // w,x,y,z (sin magnetómetro)
}

bool imuGetEvent(sh2_SensorValue_t &evt){
  if(bno08x.wasReset()){
    imuEnableReports();  // reconfigurar tras reset interno
  }
  return bno08x.getSensorEvent(&evt);
}

//PMW3901
static void flowBegin(){
  SPI.begin();
  g_flow_ok = flow.begin();   // la lib devuelve bool (true si OK)
}

//VL53L1X
static void tofBegin(){
  Wire.beginTransmission(TOF_ADDR);
  if(Wire.endTransmission()!=0){ g_tof_ok=false; return; }

  if(!tof.init()){
    g_tof_ok=false; return;
  }
  tof.setTimeout(50);                       // abortar lecturas trabadas
  tof.setDistanceMode(VL53L1X::Short);      // indoor
  tof.setMeasurementTimingBudget(50000);    // 50 ms
  tof.startContinuous(50);                  // cada 50 ms
  g_tof_ok=true;
}

bool begin(){
  imuBegin();
  flowBegin();
  tofBegin();
  return g_flow_ok /*flow*/; // BNO puede no estar aún; se reintenta en loop
}

bool flowOK(){ return g_flow_ok; }
bool tofOK(){ return g_tof_ok; }

bool tofReadMeters(float &m){
  if(!g_tof_ok) return false;
  uint16_t mm = tof.read(); // mm
  bool to = tof.timeoutOccurred();
  if(to || mm==0){ return false; }
  m = mm * 0.001f;
  // clamp básico por sanidad (deja el clamp fino al EKF/usuario)
  if(!(isfinite(m)) || m<0.0f) return false;
  return true;
}

void flowReadCounts(int16_t &dx, int16_t &dy){
  if(!g_flow_ok){ dx=dy=0; return; }
  flow.readMotionCount(&dx,&dy); // delta desde última lectura (void)
}

} // namespace
