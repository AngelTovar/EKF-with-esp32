#include <math.h>
#include <string.h>
#include "ekf.hpp"
#include "calib.hpp"
#include "util.hpp"

static constexpr float MAX_COV = 100.0f;
static constexpr float MIN_COV = 1e-8f;

ESKF::ESKF(){ begin(); }

void ESKF::begin(float x0, float y0, float z0, float /*yaw0*/){
  memset(x, 0, sizeof(x));
  x[0]=x0; x[1]=y0; x[2]=z0;

  // P inicial (diagonal)
  memset(P, 0, sizeof(P));
  P[0*9+0]=1e-3f; P[1*9+1]=1e-3f; P[2*9+2]=1e-3f;
  P[3*9+3]=1e-2f; P[4*9+4]=1e-2f; P[5*9+5]=1e-2f;
  P[6*9+6]=1e-2f; P[7*9+7]=1e-2f; P[8*9+8]=1e-2f;

  // orientación neutra
  q[0]=1; q[1]=q[2]=q[3]=0;
  quatToRotMat3(q, Rwb);
}

void ESKF::setQuaternion(const volatile float q_meas[4]){
  q[0]=q_meas[0]; q[1]=q_meas[1]; q[2]=q_meas[2]; q[3]=q_meas[3];
  quatToRotMat3(q, Rwb);
}

void ESKF::predict(const float acc_b_[3], const float /*gyro_b*/[3], float dt){
  if(!(dt>0) || dt>0.1f) return;

  // bias + deadband sobre linear accel
  float acc_b[3] = {
    acc_b_[0] - CAL.acc_bias_body[0],
    acc_b_[1] - CAL.acc_bias_body[1],
    acc_b_[2] - CAL.acc_bias_body[2]
  };
  for(int i=0;i<3;i++){
    if(fabsf(acc_b[i]) < CAL.deadband_acc) acc_b[i]=0.0f;
  }

  // a_mundo = Rwb * a_cuerpo
  float a_w[3]; R_mul_v(Rwb, acc_b, a_w);

  // Integración simple pos-vel
  x[0] += x[3]*dt + 0.5f*a_w[0]*dt*dt;
  x[1] += x[4]*dt + 0.5f*a_w[1]*dt*dt;
  x[2] += x[5]*dt + 0.5f*a_w[2]*dt*dt;
  x[3] += a_w[0]*dt;
  x[4] += a_w[1]*dt;
  x[5] += a_w[2]*dt;

  // P = F P F^T + Q  (aprox diagonal con q_pos/vel/att)
  // Aquí usamos un modelo muy simple: aumenta cov en cada componente
  P[0*9+0] += CAL.q_pos*dt; P[1*9+1] += CAL.q_pos*dt; P[2*9+2] += CAL.q_pos*dt;
  P[3*9+3] += CAL.q_vel*dt; P[4*9+4] += CAL.q_vel*dt; P[5*9+5] += CAL.q_vel*dt;
  P[6*9+6] += CAL.q_att*dt; P[7*9+7] += CAL.q_att*dt; P[8*9+8] += CAL.q_att*dt;

  clampSymm();
}

bool ESKF::scalarUpdate(const float *H, float err, float std_meas, float gateN, float &S_out){
  if(!(std_meas>0)) return false;

  // S = H P H^T + R  (escalar)
  float HP[9]; for(int i=0;i<9;i++){ HP[i]=0; for(int j=0;j<9;j++) HP[i]+=H[j]*P[j*9+i]; }
  float PH = 0; for(int i=0;i<9;i++) PH += HP[i]*H[i];
  float Rm = std_meas*std_meas;
  float S  = PH + Rm;
  if(!(S>0) || !isfinite(S)) return false;

  // Gating N-σ
  if(gateN>0){
    float N = gateN*sqrtf(S);
    if(fabsf(err) > N) { S_out=S; return false; }
  }

  // K = P H^T / S   (9x1)
  float K[9];
  for(int i=0;i<9;i++){
    K[i] = (P[i*9+0]*H[0] + P[i*9+1]*H[1] + P[i*9+2]*H[2] +
            P[i*9+3]*H[3] + P[i*9+4]*H[4] + P[i*9+5]*H[5] +
            P[i*9+6]*H[6] + P[i*9+7]*H[7] + P[i*9+8]*H[8]) / S;
  }

  // x = x + K * err
  for(int i=0;i<9;i++) x[i] += K[i]*err;

  // Joseph form: P = (I-KH)P(I-KH)^T + K R K^T
  float KH[81]={0};
  for(int i=0;i<9;i++) for(int j=0;j<9;j++) KH[i*9+j]=K[i]*H[j];

  float I_KH[81]={0};
  for(int i=0;i<9;i++){
    for(int j=0;j<9;j++){
      float v = (i==j?1.0f:0.0f) - KH[i*9+j];
      I_KH[i*9+j]=v;
    }
  }
  // T1 = (I-KH) * P
  float T1[81]={0};
  for(int i=0;i<9;i++){
    for(int j=0;j<9;j++){
      float s=0;
      for(int k=0;k<9;k++) s += I_KH[i*9+k]*P[k*9+j];
      T1[i*9+j]=s;
    }
  }
  // P_new = T1 * (I-KH)^T + K*R*K^T
  float Pnew[81]={0};
  for(int i=0;i<9;i++){
    for(int j=0;j<9;j++){
      float s=0;
      for(int k=0;k<9;k++) s += T1[i*9+k]*I_KH[j*9+k]; // (I-KH)^T
      Pnew[i*9+j]=s;
    }
  }
  // + K R K^T (escalar Rm)
  for(int i=0;i<9;i++) for(int j=0;j<9;j++) Pnew[i*9+j] += K[i]*Rm*K[j];

  memcpy(P, Pnew, sizeof(P));
  clampSymm();
  S_out = S;
  return true;
}

void ESKF::updateAltitude(float z_meas, float std_z){
  float H[9] = {0,0,1, 0,0,0, 0,0,0};
  float S; (void)scalarUpdate(H, (z_meas - x[2]), std_z, CAL.gate_N_z, S);
}

void ESKF::updateFlowVelBody(float vxb, float vyb, float std_v){
  // v_b = R^T v_w  ⇒  vxb = col0(Rwb)·[vx vy vz],  vyb = col1(Rwb)·[vx vy vz]
  const float r0 = Rwb[0], r1 = Rwb[3], r2 = Rwb[6]; // primera columna
  const float c0[9] = {0,0,0, r0,r1,r2, 0,0,0};      // H para vxb
  const float r3 = Rwb[1], r4 = Rwb[4], r5 = Rwb[7]; // segunda columna
  const float c1[9] = {0,0,0, r3,r4,r5, 0,0,0};      // H para vyb

  float errx = (vxb - (r0*x[3] + r1*x[4] + r2*x[5]));
  float Sx; (void)scalarUpdate(c0, errx, std_v, CAL.gate_N, Sx);

  float erry = (vyb - (r3*x[3] + r4*x[4] + r5*x[5]));
  float Sy; (void)scalarUpdate(c1, erry, std_v, CAL.gate_N, Sy);
}

void ESKF::updateVelZeroXY(float std_v){
  // vxb=0 y vyb=0 (en reposo) → mismas H que arriba
  const float r0 = Rwb[0], r1 = Rwb[3], r2 = Rwb[6];
  const float c0[9] = {0,0,0, r0,r1,r2, 0,0,0};
  const float r3 = Rwb[1], r4 = Rwb[4], r5 = Rwb[7];
  const float c1[9] = {0,0,0, r3,r4,r5, 0,0,0};

  float Sdummy;
  (void)scalarUpdate(c0, -(r0*x[3]+r1*x[4]+r2*x[5]), std_v, CAL.gate_N, Sdummy);
  (void)scalarUpdate(c1, -(r3*x[3]+r4*x[4]+r5*x[5]), std_v, CAL.gate_N, Sdummy);
}

void ESKF::clampSymm(){
  // simetriza y acota P
  for(int i=0;i<9;i++){
    for(int j=0;j<i;j++){
      float a = 0.5f*(P[i*9+j] + P[j*9+i]);
      P[i*9+j]=P[j*9+i]=a;
    }
  }
  for(int i=0;i<9;i++){
    if(P[i*9+i] < MIN_COV) P[i*9+i]=MIN_COV;
    if(P[i*9+i] > MAX_COV) P[i*9+i]=MAX_COV;
  }
}
