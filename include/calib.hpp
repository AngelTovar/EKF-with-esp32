#pragma once
#include <Arduino.h>

// Parámetros de calibración y ruido
struct Cal {
  // --- Bias IMU
  float gyro_bias[3]      = { 0.0f, 0.0f, 0.0f };   // rad/s
  float acc_bias_body[3]  = { 0.0f, 0.0f, 0.0f };   // m/s^2 (BNO08x linear accel)

  // --- Física ---
  float g = 9.80665f;                 // (NO sumamos g porque usamos Linear Accel)
  float deadband_acc  = 0.004f;       // m/s^2 para ZUPT (cuerpo)

  // --- Flow PMW3901 ---
  // Escalas típicas ~1/500 rad/cuenta, falta testear
  float flow_counts_to_rad_x = 1.0f/500.0f;
  float flow_counts_to_rad_y = 1.0f/500.0f;

  // Signo, swap y saturaciones
  int8_t flow_sign_x = +1;            // vxb ← +ry*h/dt (convención)
  int8_t flow_sign_y = +1;            // vyb ← +rx*h/dt
  bool   flow_swap_xy = false;
  int16_t flow_max_abs_ct = 400;      // recorte de dx/dy por sample

  // Std de flow (m/s), base + dependiente de altura
  float flow_std_base_ms = 0.05f;
  float flow_std_k_h     = 0.05f;     // +0.05 m/s por cada metro

  // --- Altura (ToF) ---
  float alt_min_m = 0.03f;
  float alt_max_m = 4.00f;
  float tof_std_base_m = 0.02f;       // 2 cm base
  float tof_std_k_h    = 0.01f;       // +1 cm/m

  // --- ZUPT ---
  float zupt_gyro_th  = 0.05f;        // rad/s
  float zupt_acc_th   = 0.05f;        // m/s^2
  float zupt_tilt_cos_min = 0.85f;    // |cos(tilt)|>0.85 (tilt < ~31°)
  float zupt_std_xy   = 0.03f;        // std m/s para ZUPT

  // --- Filtro (ruidos proceso) ---
  float q_pos = 5e-4f;
  float q_vel = 3e-2f;
  float q_att = 1e-3f;

  // --- Gating (rechazo de outliers, N-σ) ---
  float gate_N   = 3.0f;              // general
  float gate_N_z = 8.0f;              // z más permisivo

  // --- BNO08x ---
  uint32_t imu_report_rate_hz = 200;  // pide 200 Hz si el bus lo permite
};

extern Cal CAL;
