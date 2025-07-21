#include "kalman.h"

float x[2] = {0.0, 0.0};
float P[2][2] = {{1, 0}, {0, 1}};
float Q[2][2] = {{0.1, 0}, {0, 1.0}};
float R = 2.0;
float H[2] = {1, 0};
float I[2][2] = {{1, 0}, {0, 1}};
float dt = 0.01;

float smoothedAccel = 0;
float alpha = 0.1;

void initKalman() {
  x[0] = 0.0;
  x[1] = 0.0;
  P[0][0] = 1;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 1;
}

void filterAccel(float rawAccel) {
  smoothedAccel = alpha * rawAccel + (1 - alpha) * smoothedAccel;
}

void kalmanUpdate(float z, float accel) {
  float x_pred[2];
  x_pred[0] = x[0] + x[1] * dt + 0.5 * accel * dt * dt;
  x_pred[1] = x[1] + accel * dt;

  float A[2][2] = {
    {1, dt},
    {0, 1}
  };

  float P_pred[2][2];
  P_pred[0][0] = A[0][0]*P[0][0]*A[0][0] + A[0][1]*P[1][0]*A[0][0] + Q[0][0];
  P_pred[0][1] = A[0][0]*P[0][1]*A[1][1] + A[0][1]*P[1][1]*A[1][1] + Q[0][1];
  P_pred[1][0] = A[1][0]*P[0][0]*A[0][0] + A[1][1]*P[1][0]*A[0][0] + Q[1][0];
  P_pred[1][1] = A[1][0]*P[0][1]*A[1][1] + A[1][1]*P[1][1]*A[1][1] + Q[1][1];

  float y = z - (H[0]*x_pred[0] + H[1]*x_pred[1]);
  float S = H[0]*P_pred[0][0]*H[0] + H[1]*P_pred[1][1]*H[1] + R;

  float K[2];
  K[0] = (P_pred[0][0]*H[0] + P_pred[0][1]*H[1]) / S;
  K[1] = (P_pred[1][0]*H[0] + P_pred[1][1]*H[1]) / S;

  x[0] = x_pred[0] + K[0] * y;
  x[1] = x_pred[1] + K[1] * y;

  float KH[2][2] = {
    {K[0]*H[0], K[0]*H[1]},
    {K[1]*H[0], K[1]*H[1]}
  };

  P[0][0] = (I[0][0] - KH[0][0]) * P_pred[0][0];
  P[0][1] = (I[0][1] - KH[0][1]) * P_pred[0][1];
  P[1][0] = (I[1][0] - KH[1][0]) * P_pred[1][0];
  P[1][1] = (I[1][1] - KH[1][1]) * P_pred[1][1];
}

float getAltitude() {
  return x[0];
}

float getVelocity() {
  return x[1];
}
