// Cart inverted pendulum on M5Stack FIRE + 2x Unit Roller485 Lite (I2C).
//   cart  roller (0x64): speed mode, drives the wheels
//   pend  roller (0x65): encoder mode, measures the pendulum angle
//
// State x = [p, th, v, dth]  (m, rad, m/s, rad/s)
//   p  : cart position, + = direction the cart moves for a positive command
//   th : pendulum angle from upright, + = leaning toward +p
// Control u = -K x  [m/s], sent to the cart roller as a speed command.
//
// Serial protocol (line based, 921600 baud): see README.md.
#include <M5Unified.h>
#include <Preferences.h>
#include <Wire.h>

#include "roller.h"

namespace {

constexpr uint8_t kCartAddr = 0x64;
constexpr uint8_t kPendAddr = 0x65;
constexpr int kSda = 21;  // Port A
constexpr int kScl = 22;
constexpr uint32_t kI2cFreq = 400000;
constexpr uint32_t kBaud = 921600;
constexpr uint32_t kDtUs = 10000;  // control period; keep in sync with params.toml [control].dt
constexpr float kDt = kDtUs * 1e-6f;
// Register the pendulum angle is read from. If it does not change in encoder
// mode, check INFO output and try roller::kDialCounter.
constexpr uint8_t kPendPosReg = roller::kPosReadback;

constexpr uint32_t kParamsVersion = 1;

struct Params {
  uint32_t version = kParamsVersion;
  float K[4] = {0, 0, 0, 0};    // u = -K x
  float wheel_r = 0.029f;       // m
  int32_t sgn_cart = 1;         // motor direction -> +p
  int32_t sgn_pend = 1;         // encoder direction -> +th
  float pend_zero_deg = 0.0f;   // raw encoder reading while hanging straight down
  float trim_deg = 0.0f;        // upright offset
  float tf = 0.02f;             // derivative filter time constant [s]
  float u_max = 1.0f;           // |u| limit [m/s]
  float th_limit_deg = 30.0f;   // abort when |th| exceeds this
  float p_limit = 0.5f;         // abort when |p| exceeds this [m]
  float arm_window_deg = 3.0f;  // start balancing when |th| gets within this
  int32_t max_current_ma = 1200;
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
roller::Roller cart(Wire, kCartAddr);
roller::Roller pend(Wire, kPendAddr);

State state = State::kIdle;
bool logging = false;
bool i2c_ok = true;
uint32_t i2c_errors = 0;

float p_raw = 0, p0 = 0;       // cart position before / offset [m]
float th = 0, th_hang_deg = 0;
float x[4] = {0, 0, 0, 0};
float u = 0;
Deriv dp, dth;

float step_u = 0, step_dur = 0;
uint32_t step_t0 = 0;

float wrapDeg(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

void loadParams() {
  prefs.begin("pendulum", true);
  Params tmp;
  if (prefs.getBytesLength("p") == sizeof(Params)) {
    prefs.getBytes("p", &tmp, sizeof(Params));
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
  cart.write8(roller::kOutput, 0);
  cart.write8(roller::kMode, roller::kModeSpeed);
  cart.writeI32(roller::kSpeedMaxCurrent, P.max_current_ma * 100);
  cart.writeI32(roller::kSpeed, 0);
  pend.write8(roller::kOutput, 0);
  pend.write8(roller::kMode, roller::kModeEncoder);
}

void motorOn() {
  cart.writeI32(roller::kSpeed, 0);
  cart.write8(roller::kOutput, 1);
}

void motorOff() {
  cart.writeI32(roller::kSpeed, 0);
  cart.write8(roller::kOutput, 0);
}

void sendSpeed(float u_mps) {
  const float rpm = P.sgn_cart * u_mps / P.wheel_r * 60.0f / (2.0f * PI);
  if (!cart.writeI32(roller::kSpeed, static_cast<int32_t>(lroundf(rpm * 100.0f)))) {
    ++i2c_errors;
  }
}

void enter(State s) {
  if (s == State::kIdle) motorOff();
  if (state == State::kIdle && s != State::kIdle) i2c_errors = 0;
  state = s;
  Serial.printf("# state %s\n", stateName(s));
}

// Read both encoders and update x. Returns false on I2C failure.
bool measure() {
  int32_t cp, pp;
  if (!cart.readI32(roller::kPosReadback, cp) || !pend.readI32(kPendPosReg, pp)) {
    ++i2c_errors;
    return false;
  }
  p_raw = P.sgn_cart * (cp * 0.01f) * DEG_TO_RAD * P.wheel_r;
  th_hang_deg = wrapDeg(P.sgn_pend * (pp * 0.01f - P.pend_zero_deg));
  th = wrapDeg(th_hang_deg - 180.0f - P.trim_deg) * DEG_TO_RAD;
  x[0] = p_raw - p0;
  x[1] = th;
  x[2] = dp.update(p_raw, P.tf, kDt);
  x[3] = dth.update(th, P.tf, kDt);
  return true;
}

void resetFilters() {
  dp.reset(p_raw);
  dth.reset(th);
  x[2] = x[3] = 0;
}

void control(uint32_t now_ms) {
  u = 0;
  switch (state) {
    case State::kIdle:
      return;
    case State::kArmed:
      if (fabsf(th) < P.arm_window_deg * DEG_TO_RAD) {
        p0 = p_raw;
        x[0] = 0;
        resetFilters();
        motorOn();
        enter(State::kRun);
      }
      return;
    case State::kRun: {
      if (fabsf(th) > P.th_limit_deg * DEG_TO_RAD || fabsf(x[0]) > P.p_limit || i2c_errors > 5) {
        Serial.printf("# abort th=%.1fdeg p=%.3fm i2c_err=%u\n", th * RAD_TO_DEG, x[0], i2c_errors);
        enter(State::kIdle);
        return;
      }
      float s = 0;
      for (int i = 0; i < 4; ++i) s -= P.K[i] * x[i];
      u = constrain(s, -P.u_max, P.u_max);
      sendSpeed(u);
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

void printParams() {
  Serial.printf("# K %g %g %g %g\n", P.K[0], P.K[1], P.K[2], P.K[3]);
  Serial.printf("# R %g\n# SGN %ld %ld\n# PZERO %g\n# TRIM %g\n# TF %g\n", P.wheel_r, (long)P.sgn_cart,
                (long)P.sgn_pend, P.pend_zero_deg, P.trim_deg, P.tf);
  Serial.printf("# UMAX %g\n# THLIM %g\n# PLIM %g\n# ARMW %g\n# IMAX %ld\n# DT %g\n", P.u_max,
                P.th_limit_deg, P.p_limit, P.arm_window_deg, (long)P.max_current_ma, kDt);
}

void printRollerInfo(roller::Roller& r, const char* name) {
  if (!r.ping()) {
    Serial.printf("# %s(0x%02X): not found\n", name, r.addr());
    return;
  }
  uint8_t mode = 0, out = 0, fw = 0, st = 0, err = 0;
  int32_t vin = 0, pos = 0, dial = 0, spd = 0;
  r.read8(roller::kMode, mode);
  r.read8(roller::kOutput, out);
  r.read8(roller::kFirmwareVersion, fw);
  r.read8(roller::kSysStatus, st);
  r.read8(roller::kErrorCode, err);
  r.readI32(roller::kVin, vin);
  r.readI32(roller::kPosReadback, pos);
  r.readI32(roller::kDialCounter, dial);
  r.readI32(roller::kSpeedReadback, spd);
  static const char* kModes[] = {"?", "speed", "position", "current", "encoder"};
  Serial.printf("# %s(0x%02X): fw=%u mode=%u(%s) output=%u status=%u err=%u vin=%.2fV pos=%.2fdeg dial=%ld speed=%.2frpm\n",
                name, r.addr(), fw, mode, mode <= 4 ? kModes[mode] : "?", out, st, err, vin * 0.01f,
                pos * 0.01f, (long)dial, spd * 0.01f);
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
  const bool idle = state == State::kIdle;

  if (!strcmp(cmd, "K")) {
    float k[4];
    for (auto& ki : k) ki = next();
    if (isnan(k[3])) return Serial.println("# ERR K needs 4 values"), true;
    memcpy(P.K, k, sizeof(k));
  } else if (!strcmp(cmd, "ARM")) {
    if (idle) enter(State::kArmed);
  } else if (!strcmp(cmd, "STOP")) {
    enter(State::kIdle);
  } else if (!strcmp(cmd, "STEP")) {
    const float v = next(), d = next();
    if (!idle || isnan(d)) return Serial.println("# ERR STEP u[m/s] dur[s] (IDLE only)"), true;
    step_u = constrain(v, -P.u_max, P.u_max);
    step_dur = d;
    step_t0 = millis();
    p0 = p_raw;
    resetFilters();
    motorOn();
    enter(State::kStep);
  } else if (!strcmp(cmd, "LOG")) {
    logging = next() != 0;
  } else if (!strcmp(cmd, "ZERO")) {
    int32_t pp;
    if (idle && pend.readI32(kPendPosReg, pp)) P.pend_zero_deg = pp * 0.01f;
  } else if (!strcmp(cmd, "R")) { set(P.wheel_r);
  } else if (!strcmp(cmd, "SGN")) {
    const float c = next(), p = next();
    if (!isnan(p) && idle) { P.sgn_cart = c < 0 ? -1 : 1; P.sgn_pend = p < 0 ? -1 : 1; }
  } else if (!strcmp(cmd, "TRIM")) { set(P.trim_deg);
  } else if (!strcmp(cmd, "TF")) { set(P.tf);
  } else if (!strcmp(cmd, "UMAX")) { set(P.u_max);
  } else if (!strcmp(cmd, "THLIM")) { set(P.th_limit_deg);
  } else if (!strcmp(cmd, "PLIM")) { set(P.p_limit);
  } else if (!strcmp(cmd, "ARMW")) { set(P.arm_window_deg);
  } else if (!strcmp(cmd, "IMAX")) {
    const float v = next();
    if (!isnan(v)) {
      P.max_current_ma = static_cast<int32_t>(v);
      cart.writeI32(roller::kSpeedMaxCurrent, P.max_current_ma * 100);
    }
  } else if (!strcmp(cmd, "GET")) {
    printParams();
  } else if (!strcmp(cmd, "SAVE")) {
    saveParams();
  } else if (!strcmp(cmd, "INFO")) {
    if (!idle) return Serial.println("# ERR INFO is IDLE only"), true;
    printRollerInfo(cart, "cart");
    printRollerInfo(pend, "pend");
  } else if (!strcmp(cmd, "SETADDR")) {
    // Connect only the roller to be changed. SETADDR <old> <new>, decimal or 0x..
    const char* a = strtok(nullptr, " \t");
    const char* b = strtok(nullptr, " \t");
    if (!idle || !a || !b) return Serial.println("# ERR SETADDR old new (IDLE only)"), true;
    const uint8_t from = strtol(a, nullptr, 0), to = strtol(b, nullptr, 0);
    roller::Roller r(Wire, from);
    if (!r.ping()) return Serial.printf("# ERR no device at 0x%02X\n", from), true;
    r.write8(roller::kI2cAddress, to);
    delay(100);
    roller::Roller(Wire, to).write8(roller::kSaveFlash, 1);
    delay(100);
    Serial.printf("# address 0x%02X -> 0x%02X, now %s\n", from, to,
                  roller::Roller(Wire, to).ping() ? "responding" : "NOT responding");
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
    int32_t pp;
    if (pend.readI32(kPendPosReg, pp)) P.pend_zero_deg = pp * 0.01f;
    Serial.println("# pendulum zero set (hanging)");
  }
}

void drawLcd() {
  auto& d = M5.Display;
  d.setCursor(0, 0);
  d.printf("%-6s  i2c_err %-4u\n", stateName(state), i2c_errors);
  d.printf("th  %+8.2f deg\n", th * RAD_TO_DEG);
  d.printf("hang%+8.2f deg\n", th_hang_deg);
  d.printf("p   %+8.3f m\n", x[0]);
  d.printf("u   %+8.3f m/s\n", u);
  d.printf("\n A:ARM  B:STOP  C:ZERO");
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.setTxBufferSize(4096);
  Serial.begin(kBaud);
  Wire.begin(kSda, kScl, kI2cFreq);

  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.fillScreen(TFT_BLACK);

  loadParams();
  delay(500);  // let the rollers boot
  i2c_ok = cart.ping() && pend.ping();
  if (!i2c_ok) {
    Serial.println("# ERR roller not found; check wiring/addresses (INFO)");
  }
  setupRollers();
  measure();
  resetFilters();
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

  if (logging) {
    Serial.printf("D,%lu,%u,%.5f,%.5f,%.4f,%.4f,%.4f,%lu\n", (unsigned long)now_ms, (unsigned)state, x[0],
                  x[1], x[2], x[3], u, (unsigned long)exec_us);
  }

  pollSerial();
  pollButtons();
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
