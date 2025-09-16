#pragma once
#include <Arduino.h>

// ===== Pines y direcciones =====
#ifndef SDA
  #define SDA 21
#endif
#ifndef SCL
  #define SCL 22
#endif

#define BNO08X_ADDR 0x4B   // 0x4A si AD0 a GND
#define TOF_ADDR    0x29   // VL53L1X I2C

// SPI PMW3901
#ifndef PMW_CS
  #define PMW_CS 5        // ajusta según tu placa
#endif

// ===== Frecuencias de lazo / impresión =====
#define LOOP_HZ_PRINT 50

// ===== Logs (0/1) =====
#define LOG_PRINT_FLOW   0
#define LOG_PRINT_TOF    0
#define LOG_PRINT_ZUPT   0

// ===== Util =====
#ifndef PI
  #define PI 3.14159265358979323846f
#endif
#ifndef DEG_TO_RAD
  #define DEG_TO_RAD 0.017453292519943295f
#endif
#ifndef RAD_TO_DEG
  #define RAD_TO_DEG 57.29577951308232f
#endif
