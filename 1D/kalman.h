#ifndef KALMAN_H
#define KALMAN_H

extern float x[2];
extern float P[2][2];
extern float dt;
extern float R;
extern float Q[2][2];
extern float I[2][2];
extern float H[2];
extern float smoothedAccel;
extern float alpha;

void initKalman();
void filterAccel(float rawAccel);
void kalmanUpdate(float z_measured, float accel);
float getAltitude();
float getVelocity();

#endif
