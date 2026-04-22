#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Complementary filter
float angle = 0;
float alpha = 0.98;
unsigned long last_time = 0;

// Fixed normalization (adjust later after data)
float angle_min = -30, angle_max = 30;
float vel_min   = -200, vel_max   = 200;

void setup() {
  Serial.begin(115200);
  Wire.begin(3, 4);

  if (!mpu.begin(0x68)) {
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  last_time = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;

  // ── Accelerometer angle ──
  float acc_angle = atan2(a.acceleration.y,
                          a.acceleration.z) * 180.0 / PI;

  // ── Gyro rate ── (CHANGE axis if needed)
  float gyro_rate = g.gyro.x * 180.0 / PI;

  // ── Complementary filter ──
  angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * acc_angle;

  float velocity = gyro_rate;

  // ── Normalize ──
  float norm_angle = 2.0 * (angle - angle_min) / (angle_max - angle_min) - 1.0;
  float norm_vel   = 2.0 * (velocity - vel_min) / (vel_max - vel_min) - 1.0;

  // clamp
  norm_angle = constrain(norm_angle, -1, 1);
  norm_vel   = constrain(norm_vel, -1, 1);

  // ── Phase ──
  float phase_deg = atan2(norm_vel, norm_angle) * 180.0 / PI;
  if (phase_deg < 0) phase_deg += 360.0;

  // ── Print CLEAN CSV ──
  Serial.print(now); Serial.print(",");
  Serial.print(angle, 2); Serial.print(",");
  Serial.print(velocity, 2); Serial.print(",");
  Serial.println(phase_deg, 2);

  delay(20); // ~
  
}
