#include "motor_config.h"

#include <Preferences.h>
#include <cmath>

namespace motor_config
{

namespace
{

constexpr float kMinTpr = 10.0f;
constexpr float kMaxTpr = 10000.0f;
constexpr float kMinMaxRpm = 1.0f;
constexpr float kMaxMaxRpm = 500.0f;
constexpr float kMinIntegralLimit = 0.0f;
constexpr float kMaxIntegralLimit = 1000.0f;
constexpr float kMinKp = 0.0f;
constexpr float kMaxKp = 1000.0f;
constexpr float kMinKi = 0.0f;
constexpr float kMaxKi = 10000.0f;
constexpr float kMinFfK = 0.0f;
constexpr float kMaxFfK = 100.0f;
constexpr float kMinDuty = 0.0f;
constexpr float kMaxDuty = 1023.0f;

bool finite(float v) { return std::isfinite(v); }

bool clamp_float(float & v, float min_v, float max_v)
{
  if (!finite(v)) {
    return false;
  }
  if (v < min_v) {
    v = min_v;
  } else if (v > max_v) {
    v = max_v;
  }
  return true;
}

#pragma pack(push, 1)
struct StoredConfig {
  uint32_t magic;
  uint16_t version;
  RobotConfig config;
};
#pragma pack(pop)

}  // namespace

const RobotConfig & compiled_defaults()
{
  static const RobotConfig defaults = []() {
    RobotConfig c{};
    c.version = kVersion;
    c.ticks_per_revolution = 1399.2f;
    c.max_abs_rpm = 150.0f;
    c.integral_limit = 100.0f;
    for (int s = 0; s < 2; ++s) {
      c.motors[s].ff_k[kFwd] = 7.522f;
      c.motors[s].ff_k[kRev] = 7.522f;
      c.motors[s].ff_d0[kFwd] = 0.0f;
      c.motors[s].ff_d0[kRev] = 0.0f;
      c.motors[s].min_duty[kFwd] = 98.0f;
      c.motors[s].min_duty[kRev] = 98.0f;
      c.motors[s].kp = 2.051f;
      c.motors[s].ki = 20.515f;
    }
    return c;
  }();
  return defaults;
}

bool validate_and_clamp(RobotConfig & cfg)
{
  bool ok = true;
  ok &= clamp_float(cfg.ticks_per_revolution, kMinTpr, kMaxTpr);
  ok &= clamp_float(cfg.max_abs_rpm, kMinMaxRpm, kMaxMaxRpm);
  ok &= clamp_float(cfg.integral_limit, kMinIntegralLimit, kMaxIntegralLimit);

  for (int s = 0; s < 2; ++s) {
    for (int d = 0; d < 2; ++d) {
      ok &= clamp_float(cfg.motors[s].ff_k[d], kMinFfK, kMaxFfK);
      ok &= clamp_float(cfg.motors[s].ff_d0[d], kMinDuty, kMaxDuty);
      ok &= clamp_float(cfg.motors[s].min_duty[d], kMinDuty, kMaxDuty);
    }
    ok &= clamp_float(cfg.motors[s].kp, kMinKp, kMaxKp);
    ok &= clamp_float(cfg.motors[s].ki, kMinKi, kMaxKi);
  }

  return ok;
}

bool load(RobotConfig & cfg)
{
  cfg = compiled_defaults();

  Preferences prefs;
  if (!prefs.begin("motor_cfg", true)) {
    return false;
  }

  size_t len = prefs.getBytesLength("data");
  if (len != sizeof(StoredConfig)) {
    prefs.end();
    return false;
  }

  StoredConfig stored{};
  if (prefs.getBytes("data", &stored, sizeof(stored)) != sizeof(stored)) {
    prefs.end();
    return false;
  }
  prefs.end();

  if (stored.magic != kMagic || stored.version != kVersion) {
    return false;
  }

  cfg = stored.config;
  validate_and_clamp(cfg);
  return true;
}

bool save(const RobotConfig & cfg)
{
  RobotConfig tmp = cfg;
  if (!validate_and_clamp(tmp)) {
    return false;
  }

  StoredConfig stored{};
  stored.magic = kMagic;
  stored.version = kVersion;
  stored.config = tmp;

  Preferences prefs;
  if (!prefs.begin("motor_cfg", false)) {
    return false;
  }
  const size_t written = prefs.putBytes("data", &stored, sizeof(stored));
  prefs.end();
  return written == sizeof(stored);
}

void print(const RobotConfig & cfg)
{
  Serial.printf("ticks_per_revolution=%.3f max_abs_rpm=%.1f integral_limit=%.1f\n",
                cfg.ticks_per_revolution, cfg.max_abs_rpm, cfg.integral_limit);
  for (int s = 0; s < 2; ++s) {
    const char *name = (s == kLeft) ? "left" : "right";
    Serial.printf("%s: kp=%.3f ki=%.3f\n", name, cfg.motors[s].kp, cfg.motors[s].ki);
    Serial.printf("%s: ff_k=[fwd %.3f rev %.3f] d0=[fwd %.1f rev %.1f] min=[fwd %.1f rev %.1f]\n",
                  name,
                  cfg.motors[s].ff_k[kFwd], cfg.motors[s].ff_k[kRev],
                  cfg.motors[s].ff_d0[kFwd], cfg.motors[s].ff_d0[kRev],
                  cfg.motors[s].min_duty[kFwd], cfg.motors[s].min_duty[kRev]);
  }
}

}  // namespace motor_config
