/*******************************************************
  3 Reaction Wheel Attitude Controller (ESP32 DevKit v1 + 3 ESC + MPU6050)

  - estimator -> attitude PID -> wheel command (ESC pulse)
  - targets default to (0,0,0) deg; you can set via Serial:  r p y
  - ESC outputs are centered at 1500us (neutral), +/- command adds/subtracts
*******************************************************/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;
Madgwick filter;

/* ESC signal pins */
static const int ESC_X_PIN = 25;
static const int ESC_Y_PIN = 26;
static const int ESC_Z_PIN = 27;

/* Servo objects (ESC uses servo pulses) */
Servo escX, escY, escZ;

/* ESC pulse config (most ESCs: 1000-2000us, neutral 1500us) */
static const int ESC_MIN_US     = 1000;
static const int ESC_MAX_US     = 2000;
static const int ESC_NEUTRAL_US = 1500;

/* Controller gains (start here; you WILL tune) */
float Kp_att = 3.0f;
float Ki_att = 0.0f;
float Kd_att = 0.35f;

/* Targets (deg) */
volatile float targetRollDeg  = 0.0f;
volatile float targetPitchDeg = 0.0f;
volatile float targetYawDeg   = 0.0f;

/* Estimated attitude (rad) */
float roll = 0, pitch = 0, yaw = 0;

/* Body rates (rad/s) */
float pRate = 0, qRate = 0, rRate = 0;

/* Integrators */
float intRoll = 0, intPitch = 0, intYaw = 0;

/* Safety / scaling */
float intClamp = 0.35f;          // rad*s clamp for integral
float maxCmdUs = 220.0f;         // max +/- around neutral
float cmdSlewUsPerSec = 600.0f;  // how fast command can change (us/sec)

/* Output command (us delta from neutral), after slew limiting */
float outX_us = 0, outY_us = 0, outZ_us = 0;

unsigned long lastUs = 0;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static inline float deg2rad(float d) { return d * 0.01745329252f; }
static inline float rad2deg(float r) { return r * 57.2957795131f; }

static inline float wrapPi(float a) {
  while (a >  3.14159265f) a -= 2.0f * 3.14159265f;
  while (a < -3.14159265f) a += 2.0f * 3.14159265f;
  return a;
}

/* Read IMU + update attitude estimate */
void updateIMU(float dt) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pRate = g.gyro.x;
  qRate = g.gyro.y;
  rRate = g.gyro.z;

  filter.updateIMU(
    pRate, qRate, rRate,
    a.acceleration.x,
    a.acceleration.y,
    a.acceleration.z
  );

  roll  = filter.getRollRadians();
  pitch = filter.getPitchRadians();
  yaw   = filter.getYawRadians();
}

/* Attitude PID -> wheel command deltas (us) */
void attitudeController(float dt, float &ux, float &uy, float &uz) {
  float rRef = deg2rad(targetRollDeg);
  float pRef = deg2rad(targetPitchDeg);
  float yRef = deg2rad(targetYawDeg);

  float er = wrapPi(rRef - roll);
  float ep = wrapPi(pRef - pitch);
  float ey = wrapPi(yRef - yaw);

  intRoll  = clampf(intRoll  + er * dt, -intClamp, intClamp);
  intPitch = clampf(intPitch + ep * dt, -intClamp, intClamp);
  intYaw   = clampf(intYaw   + ey * dt, -intClamp, intClamp);

  // derivative: use gyro rates (negative feedback on rate)
  float dr = -pRate;
  float dp = -qRate;
  float dy = -rRate;

  float tx = Kp_att * er + Ki_att * intRoll  + Kd_att * dr;
  float ty = Kp_att * ep + Ki_att * intPitch + Kd_att * dp;
  float tz = Kp_att * ey + Ki_att * intYaw   + Kd_att * dy;

  // Map "torque-like" command -> ESC microseconds delta
  // The scale factor is embedded in Kp/Kd; we just clamp to safe pulse range.
  ux = clampf(-tx * 400.0f, -maxCmdUs, maxCmdUs);
  uy = clampf(-ty * 400.0f, -maxCmdUs, maxCmdUs);
  uz = clampf(-tz * 400.0f, -maxCmdUs, maxCmdUs);
}

/* Slew limit command in us */
float slewLimit(float prev, float target, float dt, float rateUsPerSec) {
  float maxStep = rateUsPerSec * dt;
  float step = clampf(target - prev, -maxStep, maxStep);
  return prev + step;
}

/* Write pulses to ESCs */
void writeESC(float ux, float uy, float uz) {
  int px = (int)clampf(ESC_NEUTRAL_US + ux, ESC_MIN_US, ESC_MAX_US);
  int py = (int)clampf(ESC_NEUTRAL_US + uy, ESC_MIN_US, ESC_MAX_US);
  int pz = (int)clampf(ESC_NEUTRAL_US + uz, ESC_MIN_US, ESC_MAX_US);

  escX.writeMicroseconds(px);
  escY.writeMicroseconds(py);
  escZ.writeMicroseconds(pz);
}

/* Set targets via Serial:  r p y   (degrees) */
void handleSerialTargets() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  float r, p, y;
  int n = sscanf(s.c_str(), "%f %f %f", &r, &p, &y);
  if (n == 3) {
    targetRollDeg  = r;
    targetPitchDeg = p;
    targetYawDeg   = y;
  }
}

void armESCs() {
  // Most ESCs arm at low throttle. For bidirectional ESCs, neutral is often 1500us.
  // This keeps neutral for a while so they initialize.
  writeESC(0, 0, 0);
  delay(3000);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 on DevKit v1

  if (!mpu.begin()) {
    while (true) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  filter.begin(200.0f);

  // ESP32Servo: attach with explicit min/max to match ESC range
  escX.attach(ESC_X_PIN, ESC_MIN_US, ESC_MAX_US);
  escY.attach(ESC_Y_PIN, ESC_MIN_US, ESC_MAX_US);
  escZ.attach(ESC_Z_PIN, ESC_MIN_US, ESC_MAX_US);

  armESCs();

  lastUs = micros();
}

void loop() {
  handleSerialTargets();

  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  if (dt < 0.002f) return;     // ~500 Hz cap
  if (dt > 0.05f) dt = 0.05f;  // clamp on stalls
  lastUs = nowUs;

  updateIMU(dt);

  float ux_cmd, uy_cmd, uz_cmd;
  attitudeController(dt, ux_cmd, uy_cmd, uz_cmd);

  outX_us = slewLimit(outX_us, ux_cmd, dt, cmdSlewUsPerSec);
  outY_us = slewLimit(outY_us, uy_cmd, dt, cmdSlewUsPerSec);
  outZ_us = slewLimit(outZ_us, uz_cmd, dt, cmdSlewUsPerSec);

  writeESC(outX_us, outY_us, outZ_us);

  static float tPrint = 0;
  tPrint += dt;
  if (tPrint > 0.1f) {
    tPrint = 0;
    Serial.print("Att(deg) R/P/Y: ");
    Serial.print(rad2deg(roll), 1); Serial.print(" ");
    Serial.print(rad2deg(pitch), 1); Serial.print(" ");
    Serial.print(rad2deg(yaw), 1);
    Serial.print(" | Target: ");
    Serial.print(targetRollDeg, 1); Serial.print(" ");
    Serial.print(targetPitchDeg, 1); Serial.print(" ");
    Serial.print(targetYawDeg, 1);
    Serial.print(" | ESC dUs: ");
    Serial.print(outX_us, 0); Serial.print(" ");
    Serial.print(outY_us, 0); Serial.print(" ");
    Serial.println(outZ_us, 0);
  }
}