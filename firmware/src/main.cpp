// Two-wheeled inverted pendulum on M5Stack FIRE + 2x Unit Roller485 Lite (I2C).
//   left  roller (0x64) / right roller (0x65): speed mode, one per wheel
//   body tilt: FIRE internal IMU (MPU6886), complementary filter
//
// State x = [th, psi, dth, dpsi]  (rad, rad, rad/s, rad/s)
//   th  : body tilt from upright, + = leaning forward
//   psi : wheel angle relative to the body (mean of both), + = rolls forward
// Control u = -K x  [rad/s], sent to both rollers as a speed command.
//
// Serial protocol (line based, 921600 baud): see README.md.
#include <M5Unified.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_heap_caps.h>

#include "imu.h"
#include "remote.h"
#include "roller.h"

namespace {

constexpr uint8_t kLeftAddr = 0x64;
constexpr uint8_t kRightAddr = 0x65;
constexpr int kSda = 21;  // Port A / internal bus
constexpr int kScl = 22;
constexpr uint32_t kI2cFreq = 400000;
constexpr uint32_t kBaud = 921600;
constexpr uint32_t kDtUs = 10000;  // control period; keep in sync with params.toml [control].dt
constexpr float kDt = kDtUs * 1e-6f;

constexpr uint32_t kParamsVersion = 2;
constexpr char kWifiPassword[] = "pendulum";  // smartphone remote (8+ chars)
// Calibration is rejected when the gyro spread exceeds this (i.e. the robot is
// being rotated). Averaging keeps the bias error ~sd/sqrt(n), so a few dps of
// hand-held jitter is fine.
constexpr float kCalMaxSdDps = 3.0f;

struct Params {
  uint32_t version = kParamsVersion;
  float K[4] = {0, 0, 0, 0};    // u = -K x
  float wheel_r = 0.029f;       // m
  int32_t sgn_l = 1;            // roller direction -> forward
  int32_t sgn_r = -1;
  // IMU axes as signed 1-based indices (e.g. -2 = minus sensor y)
  int32_t imu_gyro = 1;         // pitch rate, + when tipping forward
  int32_t imu_fwd = 2;          // accel axis: th_acc = atan2(fwd, up)
  int32_t imu_up = 3;
  float trim_deg = 0.0f;        // th_acc reading when balanced upright
  float gyro_bias[3] = {0, 0, 0};
  float tc = 0.5f;              // complementary filter time constant [s]
  float tf = 0.02f;             // derivative filter time constant [s]
  float u_max = 30.0f;          // |u| limit [rad/s]
  float th_limit_deg = 30.0f;   // abort when |th| exceeds this
  float x_limit = 1.0f;         // abort when wheel travel exceeds this [m]
  float arm_window_deg = 3.0f;  // start balancing when |th| gets within this
  int32_t max_current_ma = 1200;
  // Roller speed PID (raw register values), sent at boot. Only takes effect when
  // the roller's own menu has SPEED PID = User-Def. Default: Qiita article values
  // (P2 D850) plus a small I, which removes the stiction dead band at low speed.
  uint32_t speed_pid[3] = {200000, 30000, 85000000};
  // Smartphone driving: full-stick forward speed, acceleration limit (the body
  // must lean ~13 deg per m/s^2), and full-stick wheel speed difference for turning.
  float drive_vmax = 0.3f;  // m/s
  float drive_amax = 0.3f;  // m/s^2
  // rad/s added to the 0x64 roller and subtracted from 0x65; the sign depends on
  // which side each roller is mounted (negative on the original robot).
  float drive_yaw = -4.0f;
};

enum class State : uint8_t { kIdle = 0, kArmed = 1, kRun = 2, kStep = 3 };
const char* stateName(State s) {
  switch (s) {
    case State::kIdle: return "IDLE";
    case State::kArmed: return "ARMED";
    case State::kRun: return "RUN";
    case State::kStep: return "STEP";
  }
  return "?";
}

// Filtered derivative s/(tf s + 1), bilinear transform.
struct Deriv {
  float prev = 0, y = 0;
  void reset(float x) { prev = x; y = 0; }
  float update(float x, float tf, float h) {
    y = (2.0f * (x - prev) + (2.0f * tf - h) * y) / (2.0f * tf + h);
    prev = x;
    return y;
  }
};

Params P;
Preferences prefs;
roller::Roller left(Wire, kLeftAddr);
roller::Roller right(Wire, kRightAddr);
Imu imu(Wire);

State state = State::kIdle;
bool logging = false;
uint32_t i2c_errors = 0;

float acc[3], gyro[3];          // latest IMU sample (gyro bias removed)
float th = 0, th_acc = 0;       // filtered / accelerometer-only tilt [rad]
float psi_raw = 0, psi0 = 0;    // wheel angle before / offset [rad]
float x[4] = {0, 0, 0, 0};
float u = 0;
Deriv dpsi;

// Consecutive samples with the command at its limit. The wheel-angle feedback
// is positive, so a body that cannot tilt (held in the hand, wheels off the
// floor) makes the wheels run away; stop that instead of driving off.
constexpr int kSatAbortSamples = 30;
int sat_count = 0;

float vin = 0;  // left roller supply voltage, refreshed every 100 ms
float v_ref = 0, psi_ref = 0, yaw = 0;  // driving reference (see updateReference)
char ssid[24] = "";

// On-board recorder: keeps the most recent kRecCap samples taken while not
// IDLE (plus 1 s after), so runs without the USB cable can be dumped later.
struct Rec {
  uint32_t t_ms;
  uint8_t state;
  uint16_t exec_us;
  float th, psi, dth, dpsi, u, th_acc, vin, v_ref, yaw;
};
constexpr size_t kRecCap = 6000;  // 60 s at 100 Hz, in PSRAM
Rec* rec_buf = nullptr;
size_t rec_cap = 0, rec_head = 0, rec_count = 0;
uint32_t last_active_ms = 0;

void recAlloc() {
  rec_buf = static_cast<Rec*>(heap_caps_malloc(kRecCap * sizeof(Rec), MALLOC_CAP_SPIRAM));
  rec_cap = kRecCap;
  if (!rec_buf) {  // no PSRAM: fall back to 10 s in internal RAM
    rec_cap = 1000;
    rec_buf = static_cast<Rec*>(malloc(rec_cap * sizeof(Rec)));
    if (!rec_buf) rec_cap = 0;
  }
}

void recPush(const Rec& r) {
  if (!rec_cap) return;
  rec_buf[rec_head] = r;
  rec_head = (rec_head + 1) % rec_cap;
  if (rec_count < rec_cap) ++rec_count;
}

void printRec(const Rec& r) {
  Serial.printf("D,%lu,%u,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%u,%.2f,%.3f,%.2f\n", (unsigned long)r.t_ms,
                (unsigned)r.state, r.th, r.psi, r.dth, r.dpsi, r.u, r.th_acc, (unsigned)r.exec_us, r.vin, r.v_ref,
                r.yaw);
}

float step_u = 0, step_dur = 0;
uint32_t step_t0 = 0;

float axis(const float v[3], int32_t a) { return a > 0 ? v[a - 1] : -v[-a - 1]; }

void loadParams() {
  prefs.begin("pendulum", true);
  // Fields are only ever appended, so a shorter blob from an older build is
  // loaded as a prefix and the new fields keep their defaults.
  Params tmp;
  const size_t len = prefs.getBytesLength("p");
  if (len >= sizeof(uint32_t) && len <= sizeof(Params)) {
    prefs.getBytes("p", &tmp, len);
    if (tmp.version == kParamsVersion) P = tmp;
  }
  prefs.end();
}

void saveParams() {
  prefs.begin("pendulum", false);
  prefs.putBytes("p", &P, sizeof(Params));
  prefs.end();
}

void setupRollers() {
  for (auto* r : {&left, &right}) {
    r->write8(roller::kOutput, 0);
    r->write8(roller::kMode, roller::kModeSpeed);
    r->writeI32(roller::kSpeedMaxCurrent, P.max_current_ma * 100);
    r->writeI32(roller::kSpeed, 0);
    r->write(roller::kSpeedPid, P.speed_pid, sizeof(P.speed_pid));
  }
}

void setOutput(bool on) {
  for (auto* r : {&left, &right}) {
    r->writeI32(roller::kSpeed, 0);
    r->write8(roller::kOutput, on ? 1 : 0);
  }
}

// u: common wheel speed, yaw_rad_s: added to the left wheel and subtracted from the right.
void sendSpeed(float u_rad_s, float yaw_rad_s = 0) {
  constexpr float kToRpm100 = 60.0f / (2.0f * PI) * 100.0f;
  const float l = constrain(u_rad_s + yaw_rad_s, -P.u_max, P.u_max) * kToRpm100;
  const float r = constrain(u_rad_s - yaw_rad_s, -P.u_max, P.u_max) * kToRpm100;
  if (!left.writeI32(roller::kSpeed, lroundf(P.sgn_l * l))) ++i2c_errors;
  if (!right.writeI32(roller::kSpeed, lroundf(P.sgn_r * r))) ++i2c_errors;
}

void enter(State s) {
  if (s == State::kIdle) setOutput(false);
  if (state == State::kIdle && s != State::kIdle) i2c_errors = 0;
  state = s;
  Serial.printf("# state %s\n", stateName(s));
}

bool readImu() {
  if (!imu.readAll(acc, gyro)) {
    ++i2c_errors;
    return false;
  }
  for (int i = 0; i < 3; ++i) gyro[i] -= P.gyro_bias[i];
  return true;
}

float accelTilt() { return atan2f(axis(acc, P.imu_fwd), axis(acc, P.imu_up)); }

// Read IMU and both wheel angles, update x. Returns false on I2C failure.
bool measure() {
  int32_t pl, pr;
  if (!readImu() || !left.readI32(roller::kPosReadback, pl) || !right.readI32(roller::kPosReadback, pr)) {
    ++i2c_errors;
    return false;
  }
  const float w = axis(gyro, P.imu_gyro);
  th_acc = accelTilt() - P.trim_deg * DEG_TO_RAD;
  const float a = P.tc / (P.tc + kDt);
  th = a * (th + w * kDt) + (1.0f - a) * th_acc;
  psi_raw = 0.5f * (P.sgn_l * pl + P.sgn_r * pr) * 0.01f * DEG_TO_RAD;
  x[0] = th;
  x[1] = psi_raw - psi0;
  x[2] = w;
  x[3] = dpsi.update(psi_raw, P.tf, kDt);
  return true;
}

void startFromHere() {
  psi0 = psi_raw;
  x[1] = 0;
  dpsi.reset(psi_raw);
  x[3] = 0;
  v_ref = psi_ref = yaw = 0;
}

// Moves the reference with the smartphone command: v_ref is rate-limited, the
// wheel-angle reference is its integral, and the yaw difference is rate-limited too.
void updateReference(uint32_t now_ms) {
  const remote::Cmd c = remote::get(now_ms);
  const float dv = constrain(c.v * P.drive_vmax - v_ref, -P.drive_amax * kDt, P.drive_amax * kDt);
  v_ref += dv;
  psi_ref += v_ref / P.wheel_r * kDt;
  constexpr float kYawRate = 20.0f;  // rad/s^2
  yaw += constrain(c.w * P.drive_yaw - yaw, -kYawRate * kDt, kYawRate * kDt);
}

void control(uint32_t now_ms) {
  u = 0;
  switch (state) {
    case State::kIdle:
      return;
    case State::kArmed:
      if (fabsf(th) < P.arm_window_deg * DEG_TO_RAD) {
        startFromHere();
        sat_count = 0;
        setOutput(true);
        enter(State::kRun);
      }
      return;
    case State::kRun: {
      updateReference(now_ms);
      // travel is measured from the moving reference, so driving does not trip XLIM
      const float travel = P.wheel_r * (x[0] + x[1] - psi_ref);
      if (fabsf(th) > P.th_limit_deg * DEG_TO_RAD || fabsf(travel) > P.x_limit || i2c_errors > 5) {
        Serial.printf("# abort th=%.1fdeg travel=%.3fm i2c_err=%u\n", th * RAD_TO_DEG, travel, i2c_errors);
        enter(State::kIdle);
        return;
      }
      const float dpsi_ref = v_ref / P.wheel_r;
      const float e[4] = {x[0], x[1] - psi_ref, x[2], x[3] - dpsi_ref};
      float s = dpsi_ref;  // feed-forward: the speed loop needs the reference speed
      for (int i = 0; i < 4; ++i) s -= P.K[i] * e[i];
      u = constrain(s, -P.u_max, P.u_max);
      sat_count = fabsf(s) >= P.u_max ? sat_count + 1 : 0;
      if (sat_count > kSatAbortSamples) {
        Serial.printf("# abort: command saturated for %d ms (body held or wheels off the floor?)\n",
                      kSatAbortSamples * static_cast<int>(kDtUs / 1000));
        enter(State::kIdle);
        return;
      }
      sendSpeed(u, yaw);
      return;
    }
    case State::kStep: {
      const float t = (now_ms - step_t0) * 1e-3f;
      if (t < step_dur) {
        u = step_u;
      } else if (t > step_dur + 0.5f) {
        enter(State::kIdle);
        return;
      }
      sendSpeed(u);
      return;
    }
  }
}

// Average the IMU for `sec` seconds while the robot is held still.
// upright=true also sets trim so that the current pose reads th = 0.
void calibrate(float sec, bool upright) {
  const float saved[3] = {P.gyro_bias[0], P.gyro_bias[1], P.gyro_bias[2]};
  for (float& b : P.gyro_bias) b = 0;
  double g[3] = {0, 0, 0}, g2 = 0, tilt = 0;
  int n = 0;
  const uint32_t t_end = millis() + static_cast<uint32_t>(sec * 1000);
  while (millis() < t_end) {
    if (readImu()) {
      for (int i = 0; i < 3; ++i) g[i] += gyro[i];
      g2 += gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2];
      tilt += accelTilt();
      ++n;
    }
    delay(5);
  }
  if (n == 0) {
    memcpy(P.gyro_bias, saved, sizeof(saved));
    Serial.println("# ERR calibration: no IMU data");
    return;
  }
  double var = g2 / n;
  for (int i = 0; i < 3; ++i) {
    g[i] /= n;
    var -= g[i] * g[i];
  }
  const float sd_dps = sqrtf(fmaxf(var, 0)) * RAD_TO_DEG;
  if (sd_dps > kCalMaxSdDps) {
    memcpy(P.gyro_bias, saved, sizeof(saved));
    Serial.printf("# calibration skipped: moving (gyro sd %.2f dps)\n", sd_dps);
    return;
  }
  for (int i = 0; i < 3; ++i) P.gyro_bias[i] = g[i];
  if (upright) P.trim_deg = tilt / n * RAD_TO_DEG;
  th = accelTilt() - P.trim_deg * DEG_TO_RAD;
  Serial.printf("# gyro bias %.3f %.3f %.3f dps (sd %.2f), trim %.3f deg\n", P.gyro_bias[0] * RAD_TO_DEG,
                P.gyro_bias[1] * RAD_TO_DEG, P.gyro_bias[2] * RAD_TO_DEG, sd_dps, P.trim_deg);
}

void printParams() {
  Serial.printf("# K %g %g %g %g\n", P.K[0], P.K[1], P.K[2], P.K[3]);
  Serial.printf("# R %g\n# SGN %ld %ld\n# IMU %ld %ld %ld\n# TRIM %g\n# TC %g\n# TF %g\n", P.wheel_r,
                (long)P.sgn_l, (long)P.sgn_r, (long)P.imu_gyro, (long)P.imu_fwd, (long)P.imu_up, P.trim_deg,
                P.tc, P.tf);
  Serial.printf("# UMAX %g\n# THLIM %g\n# XLIM %g\n# ARMW %g\n# IMAX %ld\n# DT %g\n", P.u_max, P.th_limit_deg,
                P.x_limit, P.arm_window_deg, (long)P.max_current_ma, kDt);
  Serial.printf("# DRIVE %g %g %g\n", P.drive_vmax, P.drive_amax, P.drive_yaw);
  Serial.printf("# SPID %lu %lu %lu\n", (unsigned long)P.speed_pid[0], (unsigned long)P.speed_pid[1],
                (unsigned long)P.speed_pid[2]);
}

void printRollerInfo(roller::Roller& r, const char* name) {
  if (!r.ping()) {
    Serial.printf("# %s(0x%02X): not found\n", name, r.addr());
    return;
  }
  uint8_t mode = 0, out = 0, fw = 0, st = 0, err = 0;
  int32_t vin = 0, pos = 0, spd = 0;
  r.read8(roller::kMode, mode);
  r.read8(roller::kOutput, out);
  r.read8(roller::kFirmwareVersion, fw);
  r.read8(roller::kSysStatus, st);
  r.read8(roller::kErrorCode, err);
  r.readI32(roller::kVin, vin);
  r.readI32(roller::kPosReadback, pos);
  r.readI32(roller::kSpeedReadback, spd);
  static const char* kModes[] = {"?", "speed", "position", "current", "encoder"};
  Serial.printf("# %s(0x%02X): fw=%u mode=%u(%s) output=%u status=%u err=%u vin=%.2fV pos=%.2fdeg speed=%.2frpm\n",
                name, r.addr(), fw, mode, mode <= 4 ? kModes[mode] : "?", out, st, err, vin * 0.01f, pos * 0.01f,
                spd * 0.01f);
}

// Speed PID raw register values (library scaling: P/1e5, I/1e7, D/1e5).
void printSpeedPid(roller::Roller& r, const char* name) {
  uint32_t pid[3] = {0, 0, 0};
  if (!r.read(roller::kSpeedPid, pid, sizeof(pid))) {
    Serial.printf("# %s: speed PID read failed\n", name);
    return;
  }
  Serial.printf("# %s speed PID raw %lu %lu %lu (P=%.5f I=%.7f D=%.5f)\n", name, (unsigned long)pid[0],
                (unsigned long)pid[1], (unsigned long)pid[2], pid[0] / 1e5, pid[1] / 1e7, pid[2] / 1e5);
}

void printImuInfo() {
  readImu();
  Serial.printf("# imu(0x68): whoami=0x%02X acc=%.2f %.2f %.2f m/s2 gyro=%.2f %.2f %.2f dps th_acc=%.2fdeg th=%.2fdeg\n",
                imu.whoAmI(), acc[0], acc[1], acc[2], gyro[0] * RAD_TO_DEG, gyro[1] * RAD_TO_DEG,
                gyro[2] * RAD_TO_DEG, th_acc * RAD_TO_DEG, th * RAD_TO_DEG);
}

// Returns true if the command was recognised.
bool handleCommand(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return true;
  for (char* c = cmd; *c; ++c) *c = toupper(*c);
  auto next = []() -> float {
    const char* t = strtok(nullptr, " \t");
    return t ? atof(t) : NAN;
  };
  auto set = [&](float& dst) {
    const float v = next();
    if (!isnan(v)) dst = v;
  };
  auto err = [](const char* msg) {
    Serial.printf("# ERR %s\n", msg);
    return true;
  };
  const bool idle = state == State::kIdle;

  if (!strcmp(cmd, "K")) {
    float k[4];
    for (auto& ki : k) ki = next();
    if (isnan(k[3])) return err("K needs 4 values");
    memcpy(P.K, k, sizeof(k));
  } else if (!strcmp(cmd, "ARM")) {
    if (idle) enter(State::kArmed);
  } else if (!strcmp(cmd, "STOP")) {
    enter(State::kIdle);
  } else if (!strcmp(cmd, "STEP")) {
    const float v = next(), d = next();
    if (!idle || isnan(d)) return err("STEP u[rad/s] dur[s] (IDLE only)");
    step_u = constrain(v, -P.u_max, P.u_max);
    step_dur = d;
    step_t0 = millis();
    startFromHere();
    setOutput(true);
    enter(State::kStep);
  } else if (!strcmp(cmd, "LOG")) {
    logging = next() != 0;
  } else if (!strcmp(cmd, "CAL")) {
    if (!idle) return err("CAL is IDLE only");
    calibrate(3.0f, true);
  } else if (!strcmp(cmd, "GBIAS")) {
    if (!idle) return err("GBIAS is IDLE only");
    calibrate(2.0f, false);
  } else if (!strcmp(cmd, "SGN")) {
    const float l = next(), r = next();
    if (isnan(r) || !idle) return err("SGN left right (IDLE only)");
    P.sgn_l = l < 0 ? -1 : 1;
    P.sgn_r = r < 0 ? -1 : 1;
  } else if (!strcmp(cmd, "IMU")) {
    const float g = next(), f = next(), up = next();
    auto ok = [](float a) { return !isnan(a) && fabsf(a) >= 1 && fabsf(a) <= 3; };
    if (!ok(g) || !ok(f) || !ok(up)) return err("IMU gyro fwd up (signed axis 1..3)");
    P.imu_gyro = g;
    P.imu_fwd = f;
    P.imu_up = up;
  } else if (!strcmp(cmd, "R")) { set(P.wheel_r);
  } else if (!strcmp(cmd, "TRIM")) { set(P.trim_deg);
  } else if (!strcmp(cmd, "TC")) { set(P.tc);
  } else if (!strcmp(cmd, "TF")) { set(P.tf);
  } else if (!strcmp(cmd, "UMAX")) { set(P.u_max);
  } else if (!strcmp(cmd, "DRIVE")) {  // DRIVE vmax[m/s] amax[m/s^2] yaw[rad/s]
    set(P.drive_vmax);
    set(P.drive_amax);
    set(P.drive_yaw);
  } else if (!strcmp(cmd, "THLIM")) { set(P.th_limit_deg);
  } else if (!strcmp(cmd, "XLIM")) { set(P.x_limit);
  } else if (!strcmp(cmd, "ARMW")) { set(P.arm_window_deg);
  } else if (!strcmp(cmd, "IMAX")) {
    const float v = next();
    if (!isnan(v)) {
      P.max_current_ma = static_cast<int32_t>(v);
      for (auto* r : {&left, &right}) r->writeI32(roller::kSpeedMaxCurrent, P.max_current_ma * 100);
    }
  } else if (!strcmp(cmd, "DUMP")) {
    if (!idle) return err("DUMP is IDLE only");
    Serial.printf("# dump %u\n", (unsigned)rec_count);
    const size_t start = (rec_head + rec_cap - rec_count) % (rec_cap ? rec_cap : 1);
    for (size_t i = 0; i < rec_count; ++i) printRec(rec_buf[(start + i) % rec_cap]);
    Serial.println("# dump end");
  } else if (!strcmp(cmd, "CLEARLOG")) {
    rec_head = rec_count = 0;
  } else if (!strcmp(cmd, "GET")) {
    printParams();
  } else if (!strcmp(cmd, "SAVE")) {
    saveParams();
  } else if (!strcmp(cmd, "INFO")) {
    if (!idle) return err("INFO is IDLE only");
    printRollerInfo(left, "left");
    printRollerInfo(right, "right");
    printSpeedPid(left, "left");
    printSpeedPid(right, "right");
    printImuInfo();
  } else if (!strcmp(cmd, "SETADDR")) {
    // Connect only the roller to be changed. SETADDR <old> <new>, decimal or 0x..
    const char* a = strtok(nullptr, " \t");
    const char* b = strtok(nullptr, " \t");
    if (!idle || !a || !b) return err("SETADDR old new (IDLE only)");
    const uint8_t from = strtol(a, nullptr, 0), to = strtol(b, nullptr, 0);
    roller::Roller r(Wire, from);
    if (!r.ping()) return err("no device at old address");
    r.write8(roller::kI2cAddress, to);
    delay(100);
    roller::Roller(Wire, to).write8(roller::kSaveFlash, 1);
    delay(100);
    Serial.printf("# address 0x%02X -> 0x%02X, now %s\n", from, to,
                  roller::Roller(Wire, to).ping() ? "responding" : "NOT responding");
  } else if (!strcmp(cmd, "SPID")) {
    // SPID            : print both rollers' speed PID
    // SPID p i d      : write raw values to both; kept in Params (SAVE) and re-sent at boot
    const char* a = strtok(nullptr, " \t");
    if (a) {
      const char* b = strtok(nullptr, " \t");
      const char* c = strtok(nullptr, " \t");
      if (!idle || !b || !c) return err("SPID p i d (raw, IDLE only)");
      uint32_t pid[3];
      const char* tok[3] = {a, b, c};
      for (int i = 0; i < 3; ++i) {
        char* end;
        pid[i] = strtoul(tok[i], &end, 0);
        if (end == tok[i] || *end) return err("SPID values must be integers (raw register values)");
      }
      memcpy(P.speed_pid, pid, sizeof(pid));
      for (auto* r : {&left, &right}) r->write(roller::kSpeedPid, pid, sizeof(pid));
    }
    printSpeedPid(left, "left");
    printSpeedPid(right, "right");
  } else if (!strcmp(cmd, "SETUP")) {
    if (idle) setupRollers();
  } else {
    Serial.printf("# ERR unknown command %s\n", cmd);
    return false;
  }
  Serial.println("# OK");
  return true;
}

void pollSerial() {
  static char buf[128];
  static size_t n = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) {
        buf[n] = 0;
        handleCommand(buf);
        n = 0;
      }
    } else if (c < 0x20 || c > 0x7e) {
      n = 0;  // noise (e.g. from a port reset): drop the partial line
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void pollButtons() {
  M5.update();
  if (M5.BtnA.wasPressed() && state == State::kIdle) enter(State::kArmed);
  if (M5.BtnB.wasPressed()) enter(State::kIdle);
  if (M5.BtnC.wasPressed() && state == State::kIdle) {
    Serial.println("# CAL: hold the robot upright and still for 3 s");
    calibrate(3.0f, true);
  }
}

void drawLcd() {
  auto& d = M5.Display;
  d.setCursor(0, 0);
  d.printf("%-6s  i2c_err %-4u\n", stateName(state), i2c_errors);
  d.printf("th   %+8.2f deg\n", th * RAD_TO_DEG);
  d.printf("thacc%+8.2f deg\n", th_acc * RAD_TO_DEG);
  d.printf("dth  %+8.1f dps\n", x[2] * RAD_TO_DEG);
  d.printf("psi  %+8.1f deg\n", x[1] * RAD_TO_DEG);
  d.printf("u    %+8.2f rad/s\n", u);
  d.printf("vin  %6.2f V  rec %4u\n", vin, (unsigned)rec_count);
  d.printf("wifi %s\n", ssid);
  d.printf("\n A:ARM  B:STOP  C:CAL");
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = false;  // the IMU is read directly over Wire below
  M5.begin(cfg);
  Serial.setTxBufferSize(4096);
  Serial.begin(kBaud);
  M5.In_I2C.release();
  Wire.begin(kSda, kScl, kI2cFreq);

  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.fillScreen(TFT_BLACK);

  loadParams();
  recAlloc();
  {
    snprintf(ssid, sizeof(ssid), "pendulum-%04X", static_cast<unsigned>(ESP.getEfuseMac() >> 32) & 0xFFFF);
    remote::begin(ssid, kWifiPassword);
    Serial.printf("# wifi AP %s / %s -> http://192.168.4.1\n", ssid, kWifiPassword);
  }
  delay(500);  // let the rollers boot
  if (!left.ping() || !right.ping()) Serial.println("# ERR roller not found; check wiring/addresses (INFO)");
  if (!imu.begin()) Serial.println("# ERR IMU not found");
  setupRollers();
  calibrate(1.0f, false);  // refresh gyro bias if the robot is still
  measure();
  th = th_acc;
  startFromHere();
  Serial.println("# ready");
  printParams();
}

void loop() {
  static uint32_t next_us = micros();
  static uint32_t lcd_ms = 0;

  const uint32_t t0 = micros();
  const uint32_t now_ms = millis();
  measure();
  control(now_ms);
  const uint32_t exec_us = micros() - t0;

  static uint8_t vin_div = 0;
  if (++vin_div >= 10) {
    vin_div = 0;
    int32_t v;
    if (left.readI32(roller::kVin, v)) vin = v * 0.01f;
  }

  const Rec r{now_ms, static_cast<uint8_t>(state), static_cast<uint16_t>(min<uint32_t>(exec_us, 65535)),
              x[0], x[1], x[2], x[3], u, th_acc, vin, v_ref, yaw};
  if (state != State::kIdle) last_active_ms = now_ms;
  if (state != State::kIdle || now_ms - last_active_ms < 1000) recPush(r);
  if (logging) printRec(r);

  pollSerial();
  pollButtons();
  if (remote::takeStop()) enter(State::kIdle);
  if (remote::takeArm() && state == State::kIdle) enter(State::kArmed);
  static uint32_t pub_ms = 0;
  if (now_ms - pub_ms > 200) {
    pub_ms = now_ms;
    remote::publish(stateName(state), th * RAD_TO_DEG, vin, v_ref);
  }
  // LCD drawing takes several ms over SPI, so only refresh it while not balancing.
  const bool busy = state == State::kRun || state == State::kStep;
  if (!busy && now_ms - lcd_ms > 200) {
    lcd_ms = now_ms;
    drawLcd();
  }

  next_us += kDtUs;
  if (static_cast<int32_t>(next_us - micros()) < 0) {
    next_us = micros();  // overrun: resync
  } else {
    while (static_cast<int32_t>(next_us - micros()) > 0) {
    }
  }
}
