#pragma once
#include <Arduino.h>
#include <math.h>

// acepta buffers normales y volátiles (solo lectura)
static inline float quatYaw(const volatile float q[4]){
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float siny_cosp = 2.0f*(w*z + x*y);
  const float cosy_cosp = 1.0f - 2.0f*(y*y + z*z);
  return atan2f(siny_cosp, cosy_cosp);
}

static inline void quatToRotMat3(const volatile float q[4], float R[9]){
  const float w=q[0], x=q[1], y=q[2], z=q[3];
  const float xx=x*x, yy=y*y, zz=z*z;
  const float xy=x*y, xz=x*z, yz=y*z;
  const float wx=w*x, wy=w*y, wz=w*z;
  R[0] = 1 - 2*(yy + zz);  R[1] = 2*(xy - wz);     R[2] = 2*(xz + wy);
  R[3] = 2*(xy + wz);      R[4] = 1 - 2*(xx + zz); R[5] = 2*(yz - wx);
  R[6] = 2*(xz - wy);      R[7] = 2*(yz + wx);     R[8] = 1 - 2*(xx + yy);
}

static inline void R_mul_v(const float R[9], const float v[3], float o[3]){
  o[0]=R[0]*v[0]+R[1]*v[1]+R[2]*v[2];
  o[1]=R[3]*v[0]+R[4]*v[1]+R[5]*v[2];
  o[2]=R[6]*v[0]+R[7]*v[1]+R[8]*v[2];
}
