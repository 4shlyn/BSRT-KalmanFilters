#ifndef EKF_H
#define EKF_H

extern float x[3];         // [altitude, velocity, accel bias]
extern float P[3][3];
extern float Q[3][3];
extern float R;
extern float H[3];
extern float I[3][3];
extern float dt;
extern float smoothedAccel;
extern float alpha;

void initEKF();
void filterAccel(float rawAccel);
void ekfUpdate(float z_measured, float a_measured);
float getAltitude();
float getVelocity();
float getBias();

#endif
