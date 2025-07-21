/* 
  One-Dimensional Altitude Estimation via Kalman Filtering

  This implementation estimates the rocket’s vertical position and velocity using a discrete-time linear Kalman filter. 
  The system models the state vector x = [altitude; vertical velocity], with updates driven by IMU-derived acceleration data and corrected using barometric altitude measurements.

  Note: The state transition Jacobian 'A' is constant for this linear system and is thus hardcoded. 
  This allows implementation to be suitable for resource-constrained microcontrollers like standard Arduinos (hence .ino); 
  however, for Extended Kalman Filters (EKF), where a Jacobian must be recomputed at each instance due to nonlinearity, higher performance hardward such as a Teensy or STM32 is recommended.

  This is solely intended as an introductory implementation for new members. 
  Email 4shlynl@gmail.com if you have any questions!
*/



#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>   // IMU
#include <SparkFun_MPL3115A2.h>  // baro

Adafruit_LSM6DSOX imu;    
MPL3115A2 baro;               
bool imuConnected = false;       // flag to track IMU connectivity


// thrust curve data: time (s) and thrust (N)
// from https://www.rocketreviews.com/cti-n1100-moonburner.html
const int thrustCurveSize = 15;
float thrustTimes[thrustCurveSize] = {0, 0.04, 0.065, 0.077, 0.13, 0.35, 1, 1.5, 2, 2.3, 2.6, 2.9, 3.25, 3.4, 3.48};
float thrustValues[thrustCurveSize] = {0, 133 ,1200, 1510, 1250, 1400, 1530, 1595, 1630, 1600, 1510, 1350,1032, 120, 0};
float rocketMass = 12.4; // kg

// pyro pins
#define DROGUE_PIN 6
#define MAIN_PIN 7

// launch and flight state tracking
bool launched = false;
unsigned long launchTime = 0;

bool apogeeDetected = false;
bool drogueFired = false;
bool mainFired = false;

float previousVelocity = 0;
unsigned long apogeeTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // start I2C bus

  //  imu imu
  while (!imu.begin_I2C()) {
    Serial.println("no imu detected, retrying..");
    delay(500);
  }
  imuConnected = true;
  Serial.println("IMU connected.");

  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G); // set to ±4g range for flight

// baro stuff
  baro.begin();
  baro.setModeAltimeter();
  baro.setOversampleRate(7);  // better resolution
  baro.enableEventFlags();    // enable data-ready flags

  // setup output pins for pyro channels
  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(MAIN_PIN, OUTPUT);
  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(MAIN_PIN, LOW);
}

void loop() {
  // imu imu imu
  if (!imu.begin_I2C()) {
    imuConnected = false;
    Serial.println("imu lost. retrying..");
    delay(100);
    return;
  } else if (!imuConnected) {
    Serial.println("imu reconnected.");
    imuConnected = true;
  }

  float baroAlt = baro.readAltitude();
  float accelInput;                    // net vertical acceleration
  unsigned long now = millis();        // current time

  if (!launched) {
    sensors_event_t accel;
    imu.getAccelerometerSensor()->getEvent(&accel);
    float aZ = accel.acceleration.z - 9.81;

    if (aZ > 10.0) { // crude threshold to detect launch event
      launched = true;
      launchTime = now;
      Serial.println("laauunnncheeddd!");
    }
    accelInput = aZ;
  } else {
    // after launch, compute acceleration input
    float t_since_launch = (now - launchTime) / 1000.0;

    if (t_since_launch < thrustTimes[thrustCurveSize - 1]) {
      // during burn: compute net acceleration from thrust curve
      float thrust = getThrust(t_since_launch);
      accelInput = thrust / rocketMass - 9.81; // net vertical accel
    } else {
      // after motor burn, fall back to IMU
      sensors_event_t accel;
      imu.getAccelerometerSensor()->getEvent(&accel);
      accelInput = accel.acceleration.z - 9.81;
    }
  }

  // low-pass filter acc to reduce noise
  filterAccel(accelInput);
  kalmanUpdate(baroAlt, smoothedAccel);

  // -------- detect apogee --------
  if (!apogeeDetected && previousVelocity > 0 && x[1] <= 0) {
    apogeeDetected = true;
    apogeeTime = now;
    digitalWrite(DROGUE_PIN, HIGH);  // fire drogue charge
    delay(500);
    digitalWrite(DROGUE_PIN, LOW);
    drogueFired = true;
  }

  // -------- main para --------
  if (apogeeDetected && !mainFired) {
    // after 3 seconds and under 300m altitude, fire main
    if (now - apogeeTime > 3000 && x[0] < 300) {
      digitalWrite(MAIN_PIN, HIGH);
      delay(500);
      digitalWrite(MAIN_PIN, LOW);
      mainFired = true;
    }
  }

  previousVelocity = x[1];

  // debugging
  Serial.print("Altitude: "); Serial.print(x[0]);
  Serial.print(" m, Velocity: "); Serial.print(x[1]);
  Serial.print(" m/s, Accel: "); Serial.println(accelInput);

  delay(10); // run at ~100 Hz
}


float getThrust(float t) {
  if (t < thrustTimes[0] || t > thrustTimes[thrustCurveSize - 1]) return 0;
  for (int i = 0; i < thrustCurveSize - 1; i++) {
    if (t >= thrustTimes[i] && t < thrustTimes[i + 1]) {
      float t0 = thrustTimes[i];
      float t1 = thrustTimes[i + 1];
      float f0 = thrustValues[i];
      float f1 = thrustValues[i + 1];
      float frac = (t - t0) / (t1 - t0);
      return f0 + frac * (f1 - f0);
    }
  }
  return 0;
}


