// Minimal I2C driver for M5Stack Unit Roller485 (Lite).
// Register map: https://github.com/m5stack/M5Unit-Roller (unit_roller_common.hpp)
#pragma once
#include <Wire.h>

namespace roller {

enum Reg : uint8_t {
  kOutput = 0x00,        // u8  0: off, 1: on
  kMode = 0x01,          // u8  1: speed, 2: position, 3: current, 4: encoder
  kSysStatus = 0x0C,     // u8
  kErrorCode = 0x0D,     // u8
  kStallProtect = 0x0F,  // u8
  kVin = 0x34,           // i32 [0.01 V]
  kTemp = 0x38,          // i32 [degC]
  kDialCounter = 0x3C,   // i32
  kSpeed = 0x40,         // i32 [0.01 rpm]
  kSpeedMaxCurrent = 0x50,  // i32 [0.01 mA]
  kSpeedReadback = 0x60,    // i32 [0.01 rpm]
  kSpeedPid = 0x70,         // u32 x3
  kPosReadback = 0x90,      // i32 [0.01 deg], multi-turn
  kCurrent = 0xB0,          // i32 [0.01 mA]
  kCurrentReadback = 0xC0,  // i32 [0.01 mA]
  kSaveFlash = 0xF0,
  kFirmwareVersion = 0xFE,
  kI2cAddress = 0xFF,
};

enum Mode : uint8_t { kModeSpeed = 1, kModePosition = 2, kModeCurrent = 3, kModeEncoder = 4 };

class Roller {
 public:
  Roller(TwoWire& wire, uint8_t addr) : wire_(wire), addr_(addr) {}

  uint8_t addr() const { return addr_; }

  bool ping() {
    wire_.beginTransmission(addr_);
    return wire_.endTransmission() == 0;
  }

  bool write(uint8_t reg, const void* data, size_t len) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(static_cast<const uint8_t*>(data), len);
    return wire_.endTransmission() == 0;
  }

  bool read(uint8_t reg, void* data, size_t len) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(addr_, static_cast<uint8_t>(len)) != len) return false;
    auto* p = static_cast<uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) p[i] = wire_.read();
    return true;
  }

  bool write8(uint8_t reg, uint8_t v) { return write(reg, &v, 1); }
  bool writeI32(uint8_t reg, int32_t v) { return write(reg, &v, 4); }  // ESP32 is little-endian

  bool read8(uint8_t reg, uint8_t& v) { return read(reg, &v, 1); }
  bool readI32(uint8_t reg, int32_t& v) { return read(reg, &v, 4); }

 private:
  TwoWire& wire_;
  uint8_t addr_;
};

}  // namespace roller
