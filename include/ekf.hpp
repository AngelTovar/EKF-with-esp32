#pragma once
#include <Arduino.h>

class ESKF {
public:
  ESKF();

  // x = [px py pz  vx vy vz  dθx dθy dθz]
  void begin(float x0=0, float y0=0, float z0=0, float yaw0=0);

  // orientación (externa, del BNO) para rotar acc_b -> mundo
  void setQuaternion(const volatile float q_meas[4]);
  void predict(const float acc_b[3], const float gyro_b[3], float dt);

  // Medidas
  void updateAltitude(float z_meas, float std_z);
  void updateFlowVelBody(float vxb, float vyb, float std_v);
  void updateVelZeroXY(float std_v);

  // Lectura de estado
  void getPosition(float p[3]) const { p[0]=x[0]; p[1]=x[1]; p[2]=x[2]; }
  void getRwb(float R_out[9]) const { for(int i=0;i<9;i++) R_out[i]=Rwb[i]; }

private:
  float x[9];   // estado
  float P[81];  // 9x9
  float q[4], Rwb[9]; // orientación externa (mundo = Rwb * cuerpo)

  // helpers
  bool scalarUpdate(const float *H9, float err, float std_meas, float gateN, float &S_out);
  void clampSymm();
};
