#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "calib.hpp"
#include "ekf.hpp"
#include "sensors.hpp"
#include "util.hpp"

ESKF ekf;
sh2_SensorValue_t evt;

volatile float imuAcc[3]={0,0,0};     // linear accel (m/s^2) en cuerpo
volatile float imuGyro[3]={0,0,0};    // gyro (rad/s) en cuerpo
volatile float imuQuat[4]={1,0,0,0};  // w,x,y,z

// Acumulador de flow en ventana
struct {
  long sx=0, sy=0;
  float dt = 0.0f;
  void reset(){ sx=0; sy=0; dt=0.0f; }
} g_flowAcc;

static const float FLOW_WIN_S = 0.040f; // 40 ms de ventana (20–60 ms es buen rango)

static float clampf(float v, float lo, float hi){
  return v<lo? lo : (v>hi? hi : v);
}

// ===== Helpers de lectura IMU (BNO085) =====
static void readIMU(){
  // Dranea la cola de eventos del BNO08x
  while(Sensors::imuGetEvent(evt)){
    switch(evt.sensorId){
      case SH2_LINEAR_ACCELERATION:
        imuAcc[0]=evt.un.linearAcceleration.x;
        imuAcc[1]=evt.un.linearAcceleration.y;
        imuAcc[2]=evt.un.linearAcceleration.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        imuGyro[0]=evt.un.gyroscope.x - CAL.gyro_bias[0];
        imuGyro[1]=evt.un.gyroscope.y - CAL.gyro_bias[1];
        imuGyro[2]=evt.un.gyroscope.z - CAL.gyro_bias[2];
        break;

      case SH2_GAME_ROTATION_VECTOR:
        imuQuat[0]=evt.un.gameRotationVector.real;
        imuQuat[1]=evt.un.gameRotationVector.i;
        imuQuat[2]=evt.un.gameRotationVector.j;
        imuQuat[3]=evt.un.gameRotationVector.k;
        ekf.setQuaternion(imuQuat);
        break;

      default: break;
    }
    // Evita monopolizar CPU (alimentar RTOS/wdt)
    yield();
  }
}

// ===== Estado altura por ToF =====
static float alt_tof_m = 0.15f;   // valor razonable por defecto
static uint32_t lastPredictUs=0, lastPrintUs=0, lastFlowUs=0;
static uint32_t lastTofMs=0;

void setup(){
  Serial.begin(115200);
  delay(200);

  if(!Sensors::begin()){
    Serial.println("Error inicializando sensores (flow/ToF). BNO08x puede tardar.");
    // seguimos; el loop reintenta BNO08x vía wasReset()
  }

  // Altura inicial (si hay ToF)
  float m;
  if(Sensors::tofReadMeters(m)){
    alt_tof_m = clampf(m, CAL.alt_min_m, CAL.alt_max_m);
  }

  ekf.begin(0,0,alt_tof_m, quatYaw(imuQuat));
  lastPredictUs = micros();
  lastPrintUs   = micros();
  lastFlowUs    = micros();
  lastTofMs     = millis();
}

void loop(){
  // ===== 1) Leer IMU y mantener q =====
  readIMU();

  // ===== 2) ToF (altura) =====
  float z;
  bool hadToF = false;
  if(Sensors::tofReadMeters(z)){
    hadToF = true;
    alt_tof_m = clampf(z, CAL.alt_min_m, CAL.alt_max_m);
    // std adaptativa
    float std_z = CAL.tof_std_base_m + CAL.tof_std_k_h * alt_tof_m;
    if(isfinite(std_z) && std_z>0){
      ekf.updateAltitude(alt_tof_m, std_z);
      if(LOG_PRINT_TOF){ Serial.printf("[TOF] z=%.3f std=%.3f\n", alt_tof_m, std_z); }
    }
    lastTofMs = millis();
  }

  // ===== 3) Flow PMW3901 (ventana) =====
  uint32_t now = micros();
  float dt_flow = (now - lastFlowUs)*1e-6f;
  if(dt_flow > 0 && dt_flow < 0.2f){
    int16_t dx=0, dy=0;
    Sensors::flowReadCounts(dx,dy);
    // Recorte por muestra para robustez
    dx = (int16_t)clampf(dx, -CAL.flow_max_abs_ct, CAL.flow_max_abs_ct);
    dy = (int16_t)clampf(dy, -CAL.flow_max_abs_ct, CAL.flow_max_abs_ct);
    g_flowAcc.sx += dx;
    g_flowAcc.sy += dy;
    g_flowAcc.dt += dt_flow;
  }
  lastFlowUs = now;

  bool hadFlow = false;
  if(g_flowAcc.dt >= FLOW_WIN_S){
    // intercambiar ejes si el montaje lo requiere
    long fx = CAL.flow_swap_xy ? g_flowAcc.sy : g_flowAcc.sx;
    long fy = CAL.flow_swap_xy ? g_flowAcc.sx : g_flowAcc.sy;

    // counts -> rad
    float rx = (float)fx * CAL.flow_counts_to_rad_x;
    float ry = (float)fy * CAL.flow_counts_to_rad_y;

    float h = clampf(alt_tof_m, CAL.alt_min_m, CAL.alt_max_m);

    // velocidad cuerpo m/s ≈ flow_rad * h / dt_acum
    float vxb = CAL.flow_sign_x * (ry * h / g_flowAcc.dt);
    float vyb = CAL.flow_sign_y * (rx * h / g_flowAcc.dt);

    // std adaptativa con altura
    float std_v = CAL.flow_std_base_ms + CAL.flow_std_k_h * h;

    if(isfinite(vxb) && isfinite(vyb) && std_v>0 && isfinite(std_v)){
      ekf.updateFlowVelBody(vxb, vyb, std_v);
      hadFlow = true;
      if(LOG_PRINT_FLOW){
        Serial.printf("[FLOW] fx=%ld fy=%ld dt=%.3f h=%.2f vxb=%.3f vyb=%.3f std=%.3f\n",
                      fx, fy, g_flowAcc.dt, h, vxb, vyb, std_v);
      }
    }
    g_flowAcc.reset();
  }

  // ===== 4) ZUPT XY si estamos quietos y no hubo flow en esta ventana =====
  if(!hadFlow){
    // Criterio simple: tilt moderado, gyro bajo, accel pequeña
    float Rwb[9]; ekf.getRwb(Rwb);
    float cosTilt = fabsf( Rwb[8] );    // Rwb[8] = proyección de Z_cuerpo sobre Z_mundo
    float gyro_norm = sqrtf(imuGyro[0]*imuGyro[0]+imuGyro[1]*imuGyro[1]+imuGyro[2]*imuGyro[2]);
    float acc_norm  = sqrtf(imuAcc[0]*imuAcc[0]+imuAcc[1]*imuAcc[1]+imuAcc[2]*imuAcc[2]);
    if(cosTilt > CAL.zupt_tilt_cos_min && gyro_norm < CAL.zupt_gyro_th && acc_norm < CAL.zupt_acc_th){
      ekf.updateVelZeroXY(CAL.zupt_std_xy);
      if(LOG_PRINT_ZUPT){ Serial.println("[ZUPT]"); }
    }
  }

  // ===== 5) Predict con dt estable =====
  uint32_t nowPred = micros();
  float dtp = (nowPred - lastPredictUs)*1e-6f;
  if(dtp > 0 && dtp <= 0.05f){
    float acc_copy[3]  = { imuAcc[0], imuAcc[1], imuAcc[2] };
    float gyro_copy[3] = { imuGyro[0], imuGyro[1], imuGyro[2] };
    if(isfinite(acc_copy[0]) && isfinite(acc_copy[1]) && isfinite(acc_copy[2])){
      ekf.predict(acc_copy, gyro_copy, dtp);
    }
    lastPredictUs = nowPred;
  }else{
    // si el dt es anómalo (por bloqueo I2C/RTOS), re-sincroniza sin integrar
    lastPredictUs = nowPred;
  }

  // ===== 6) Salida a ~50 Hz =====
  uint32_t nowPrint = micros();
  if((nowPrint - lastPrintUs) >= (1000000/LOOP_HZ_PRINT)){
    lastPrintUs = nowPrint;
    float p[3]; ekf.getPosition(p);
    Serial.printf("px,py,pz\n%.6f,%.6f,%.6f\n", p[0],p[1],p[2]);
  }
}
