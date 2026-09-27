// Minimal driver for the M5Stack FIRE's internal IMU (MPU6886; MPU6050/9250
// compatible register map) on the shared I2C bus.
#pragma once
#include <Wire.h>

class Imu {
 public:
  static constexpr uint8_t kAddr = 0x68;
  static constexpr float kAccScale = 9.80665f / 8192.0f;         // +-4 g  -> m/s^2
  static constexpr float kGyroScale = DEG_TO_RAD / 32.8f;        // +-1000 dps -> rad/s

  explicit Imu(TwoWire& wire) : wire_(wire) {}

  bool begin() {
    if (!write(0x6B, 0x80)) return false;  // PWR_MGMT_1: reset
    delay(50);
    write(0x6B, 0x01);  // clock: auto PLL
    delay(10);
    write(0x19, 0x00);  // SMPLRT_DIV: 1 kHz
    write(0x1A, 0x02);  // CONFIG: gyro DLPF ~92 Hz
    write(0x1B, 0x10);  // GYRO_CONFIG: +-1000 dps
    write(0x1C, 0x08);  // ACCEL_CONFIG: +-4 g
    write(0x1D, 0x02);  // ACCEL_CONFIG2: accel DLPF ~99 Hz
    write(0x6A, 0x00);  // USER_CTRL
    write(0x23, 0x00);  // FIFO_EN
    delay(10);
    return true;
  }

  uint8_t whoAmI() {
    uint8_t v = 0;
    read(0x75, &v, 1);
    return v;
  }

  // acc [m/s^2], gyro [rad/s] in sensor axes.
  bool readAll(float acc[3], float gyro[3]) {
    uint8_t b[14];
    if (!read(0x3B, b, sizeof(b))) return false;
    for (int i = 0; i < 3; ++i) {
      acc[i] = static_cast<int16_t>((b[2 * i] << 8) | b[2 * i + 1]) * kAccScale;
      gyro[i] = static_cast<int16_t>((b[8 + 2 * i] << 8) | b[9 + 2 * i]) * kGyroScale;
    }
    return true;
  }

 private:
  bool write(uint8_t reg, uint8_t v) {
    wire_.beginTransmission(kAddr);
    wire_.write(reg);
    wire_.write(v);
    return wire_.endTransmission() == 0;
  }

  bool read(uint8_t reg, uint8_t* data, size_t len) {
    wire_.beginTransmission(kAddr);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(kAddr, static_cast<uint8_t>(len)) != len) return false;
    for (size_t i = 0; i < len; ++i) data[i] = wire_.read();
    return true;
  }

  TwoWire& wire_;
};
