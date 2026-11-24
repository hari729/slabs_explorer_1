// =============================================================================
// autotune.cpp — boot-time motor identification + PI auto-tuner
//
// LIFT THE WHEELS OFF THE GROUND before running. Motors spin up to full duty.
//
// Called from setup() when TUNE_MODE_PIN is LOW at boot. Uses the same pins,
// encoder signs, PWM and TPR as the normal control loop. After printing the
// xacro-ready parameters the firmware halts; power cycle to run normally.
//
// Tuning procedure (per motor, both directions):
//   1. Breakaway duty : smallest duty that makes the wheel turn.
//   2. Duty sweep     : steady-state RPM at 5 duties -> fit  duty = d0 + k*rpm.
//   3. Step response  : duty step (forward only) -> K, tau, L -> IMC PI gains.
// =============================================================================

#include "autotune.h"

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

#include "board_config.h"
#include "motor_config.h"

extern motor_config::RobotConfig g_config;
extern ESP32Encoder g_left_encoder;
extern ESP32Encoder g_right_encoder;

namespace
{

constexpr int PWM_PIN[2] = {LEFT_PWM_PIN, RIGHT_PWM_PIN};
constexpr int DIR_PIN[2] = {LEFT_DIR_PIN, RIGHT_DIR_PIN};
constexpr int ENC_A[2] = {LEFT_ENC_A_PIN, RIGHT_ENC_A_PIN};
constexpr int ENC_B[2] = {LEFT_ENC_B_PIN, RIGHT_ENC_B_PIN};
constexpr int ENC_SIGN[2] = {LEFT_ENCODER_SIGN, RIGHT_ENCODER_SIGN};
constexpr int PWM_CH[2] = {LEFT_PWM_CHANNEL, RIGHT_PWM_CHANNEL};
constexpr const char *NAME[2] = {"LEFT", "RIGHT"};

constexpr float LAMBDA_FACTOR = 1.0f;
static const int STEP_N = 250;  // 2.5 s at 100 Hz
static const int W = 5;         // 50 ms RPM window

struct Fit {
  bool ok = false;
  bool wrong_sign = false;
  float d0 = 0, k = 0, maxrpm = 0;
};
struct StepFit {
  bool ok = false;
  float K = 0, tau = 0, L = 0;
};
struct MotorResult {
  float breakaway[2] = {-1, -1};  // [0]=forward, [1]=reverse
  Fit fit[2];
  StepFit step;
};

static MotorResult res[2];
static int32_t st_ticks[STEP_N + 1];
static uint32_t st_us[STEP_N + 1];
static float st_y[STEP_N + 1];
static float st_t[STEP_N + 1];

ESP32Encoder &encoder(int m)
{
  return (m == 0) ? g_left_encoder : g_right_encoder;
}

void set_motor(int m, int duty)
{
  set_motor_pwm(PWM_CH[m], DIR_PIN[m], duty);
}

void stop_all()
{
  set_motor(0, 0);
  set_motor(1, 0);
}

bool estop_pressed()
{
  return (ESTOP_PIN >= 0) && (digitalRead(ESTOP_PIN) == LOW);
}

void abort_now()
{
  stop_all();
  Serial.println("# ABORTED by user");
  while (true) {
    delay(1000);
  }
}

void check_abort()
{
  if (estop_pressed()) {
    abort_now();
  }
  if (Serial.available() && Serial.read() == 'x') {
    abort_now();
  }
}

void wait_ms(uint32_t ms)
{
  const uint32_t start = millis();
  while (millis() - start < ms) {
    check_abort();
    delay(1);
  }
}

int32_t ticks(int m)
{
  return static_cast<int32_t>(encoder(m).getCount()) * ENC_SIGN[m];
}

float measure_rpm(int m, uint32_t ms)
{
  const int32_t t0 = ticks(m);
  const uint32_t u0 = micros();
  wait_ms(ms);
  const int32_t t1 = ticks(m);
  const uint32_t u1 = micros();
  const float dt = static_cast<float>(static_cast<uint32_t>(u1 - u0)) / 1e6f;
  return (static_cast<float>(t1 - t0) / g_config.ticks_per_revolution) / dt * 60.0f;
}

float find_breakaway(int m, int dir)
{
  for (int duty = 80; duty <= PWM_MAX_DUTY; duty += 10) {
    set_motor(m, dir * duty);
    const float rpm = fabsf(measure_rpm(m, 150));
    if (rpm > 3.0f) {
      stop_all();
      wait_ms(800);
      return static_cast<float>(duty);
    }
  }
  stop_all();
  wait_ms(800);
  return -1.0f;
}

Fit run_sweep(int m, int dir)
{
  static const float FRACTIONS[] = {0.45f, 0.60f, 0.75f, 0.90f, 1.00f};
  const int N = sizeof(FRACTIONS) / sizeof(FRACTIONS[0]);

  Fit f;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  int n = 0;

  for (int i = 0; i < N; i++) {
    const int duty = static_cast<int>(FRACTIONS[i] * PWM_MAX_DUTY);
    set_motor(m, dir * duty);
    wait_ms(1500);
    const float rpm = measure_rpm(m, 800) * dir;
    Serial.printf("#   duty %c%4d -> %7.2f rpm\n", dir > 0 ? '+' : '-', duty, rpm);

    if (rpm < -2.0f) {
      f.wrong_sign = true;
    }
    if (rpm > 2.0f) {
      sx += rpm;
      sy += duty;
      sxx += static_cast<double>(rpm) * rpm;
      sxy += static_cast<double>(rpm) * duty;
      n++;
    }
    if (i == N - 1) {
      f.maxrpm = rpm;
    }
  }
  stop_all();
  wait_ms(1000);

  const double den = n * sxx - sx * sx;
  if (n >= 3 && fabs(den) > 1e-6) {
    f.k = static_cast<float>((n * sxy - sx * sy) / den);
    f.d0 = static_cast<float>((sy - f.k * sx) / n);
    f.ok = f.k > 0.0f;
  }
  return f;
}

float window_rpm(int i)
{
  const float dt =
      static_cast<float>(static_cast<uint32_t>(st_us[i] - st_us[i - W])) / 1e6f;
  return (static_cast<float>(st_ticks[i] - st_ticks[i - W]) / g_config.ticks_per_revolution) /
         dt * 60.0f;
}

float crossing(float level)
{
  for (int i = W + 1; i <= STEP_N; i++) {
    if (st_y[i - 1] < level && st_y[i] >= level) {
      const float frac = (level - st_y[i - 1]) / (st_y[i] - st_y[i - 1]);
      return st_t[i - 1] + frac * (st_t[i] - st_t[i - 1]);
    }
  }
  return -1.0f;
}

StepFit run_step(int m, int d_lo, int d_hi)
{
  StepFit s;

  set_motor(m, d_lo);
  wait_ms(2000);
  const float rpm_lo = measure_rpm(m, 500);

  st_ticks[0] = ticks(m);
  st_us[0] = micros();
  set_motor(m, d_hi);
  uint32_t next = st_us[0];
  for (int i = 1; i <= STEP_N; i++) {
    next += 10000;  // 10 ms
    while (static_cast<int32_t>(micros() - next) < 0) {
    }
    st_ticks[i] = ticks(m);
    st_us[i] = micros();
  }
  stop_all();
  wait_ms(1000);

  float fin = 0;
  for (int i = STEP_N - 49; i <= STEP_N; i++) {
    fin += window_rpm(i);
  }
  fin /= 50.0f;
  const float delta = fin - rpm_lo;
  Serial.printf("#   step %d -> %d duty: %.1f -> %.1f rpm\n", d_lo, d_hi, rpm_lo, fin);
  if (delta < 5.0f) {
    Serial.println("#   step produced too little speed change");
    return s;
  }

  for (int i = W; i <= STEP_N; i++) {
    st_y[i] = (window_rpm(i) - rpm_lo) / delta;
    st_t[i] = (static_cast<float>(st_us[i] - st_us[0]) +
               static_cast<float>(st_us[i - W] - st_us[0])) /
              2e6f;
  }

  const float t28 = crossing(0.283f);
  const float t63 = crossing(0.632f);
  Serial.printf("#   t28=%.3f s  t63=%.3f s\n", t28, t63);
  if (t28 < 0 || t63 <= t28) {
    Serial.println(
        "#   could not extract time constant (plant too fast or noisy)");
    return s;
  }

  s.tau = 1.5f * (t63 - t28);
  s.L = fmaxf(t63 - s.tau, 0.0f);
  s.K = delta / static_cast<float>(d_hi - d_lo);
  s.ok = s.tau > 0.005f && s.K > 0.0f;
  return s;
}

void tune_motor(int m)
{
  Serial.printf("# ---------- %s motor ----------\n", NAME[m]);
  encoder(m).clearCount();
  stop_all();
  wait_ms(500);

  for (int d = 0; d < 2; d++) {
    const int dir = (d == 0) ? 1 : -1;
    Serial.printf("# %s %s: breakaway ramp...\n", NAME[m],
                  d == 0 ? "forward" : "reverse");
    res[m].breakaway[d] = find_breakaway(m, dir);
    Serial.printf("#   breakaway duty = %.0f\n", res[m].breakaway[d]);
    Serial.printf("# %s %s: duty sweep...\n", NAME[m],
                  d == 0 ? "forward" : "reverse");
    res[m].fit[d] = run_sweep(m, dir);
    if (res[m].fit[d].wrong_sign) {
      Serial.printf(
          "# !!! %s: rpm sign is OPPOSITE to the commanded direction -> "
          "flip ENC_SIGN for this wheel (or swap the motor wires)\n",
          NAME[m]);
    }
  }

  const float bk = res[m].breakaway[motor_config::kFwd];
  if (bk > 0) {
    float d_lo = fmaxf(0.5f * PWM_MAX_DUTY, 1.25f * bk);
    d_lo = fminf(d_lo, 0.7f * PWM_MAX_DUTY);
    const float d_hi = fminf(d_lo + 0.25f * PWM_MAX_DUTY, 0.95f * PWM_MAX_DUTY);
    Serial.printf("# %s: step response...\n", NAME[m]);
    res[m].step = run_step(m, static_cast<int>(d_lo), static_cast<int>(d_hi));
  } else {
    Serial.printf("# %s: motor never started, skipping step test\n", NAME[m]);
  }
}

struct Gains {
  bool ok = false;
  float k = 0, d0 = 0, kp = 0, ki = 0;
};

Gains compute_gains(int m)
{
  Gains g;
  const MotorResult &r = res[m];

  float k = 0, d0 = 0;
  int c = 0;
  for (int d = 0; d < 2; d++) {
    if (r.fit[d].ok) {
      k += r.fit[d].k;
      d0 += r.fit[d].d0;
      c++;
    }
  }
  if (c == 0) {
    return g;
  }
  if (c == 2) {
    const float ratio = r.fit[0].k / r.fit[1].k;
    if (ratio > 1.25f || ratio < 0.8f) {
      Serial.printf(
          "// WARNING %s: forward/reverse gain differ by %.0f%% "
          "(check encoder wiring / pull-ups)\n",
          NAME[m], fabsf(ratio - 1.0f) * 100.0f);
    }
  }
  g.k = k / c;
  g.d0 = d0 / c;

  float K = r.step.K, tau = r.step.tau, L = r.step.L;
  if (!r.step.ok) {
    K = 1.0f / g.k;
    tau = 0.03f;
    L = 0.01f;
    Serial.printf("// NOTE %s: step fit failed, assuming tau = 30 ms\n", NAME[m]);
  }

  const float lambda = fmaxf(LAMBDA_FACTOR * fmaxf(tau, L), 0.10f);
  const float ti = fmaxf(tau, 0.10f);
  g.kp = tau / (K * (lambda + L));
  g.ki = g.kp / ti;
  g.ok = true;
  return g;
}

void print_xacro_params()
{
  Serial.println();
  Serial.println("// ================= TUNING RESULTS =================");

  Gains g[2];
  float min_max = 1e9f;
  bool any_valid = false;

  for (int m = 0; m < 2; m++) {
    g[m] = compute_gains(m);
    Serial.printf(
      "// %s: breakaway fwd %.0f / rev %.0f | max rpm fwd %.1f / rev %.1f | "
      "plant K=%.4f rpm/duty tau=%.3f s L=%.3f s\n",
      NAME[m], res[m].breakaway[0], res[m].breakaway[1],
      res[m].fit[0].maxrpm, res[m].fit[1].maxrpm,
      res[m].step.K, res[m].step.tau, res[m].step.L);

    for (int d = 0; d < 2; d++) {
      if (res[m].fit[d].ok && res[m].fit[d].maxrpm > 0.0f) {
        min_max = fminf(min_max, res[m].fit[d].maxrpm);
        any_valid = true;
      }
    }
    if (!g[m].ok) {
      Serial.printf("// %s: FAILED to produce new gains (see log above)\n", NAME[m]);
    }
  }

  if (any_valid) {
    Serial.printf("<param name=\"max_abs_rpm\">%.1f</param>\n", 0.9f * min_max);
  } else {
    Serial.printf(
      "<param name=\"max_abs_rpm\">%.1f</param>  <!-- no valid sweep, kept current -->\n",
      g_config.max_abs_rpm);
  }

  for (int m = 0; m < 2; m++) {
    const char *side = (m == 0) ? "left" : "right";
    for (int d = 0; d < 2; d++) {
      const char *dir_name = (d == 0) ? "fwd" : "rev";
      float ff_k = g_config.motors[m].ff_k[d];
      float ff_d0 = g_config.motors[m].ff_d0[d];
      float min_duty = g_config.motors[m].min_duty[d];

      if (res[m].fit[d].ok) {
        ff_k = res[m].fit[d].k;
        ff_d0 = res[m].fit[d].d0;
      } else {
        Serial.printf(
          "// WARNING %s %s sweep failed, keeping current ff_k/ff_d0\n",
          side, dir_name);
      }
      if (res[m].breakaway[d] > 0.0f) {
        min_duty = res[m].breakaway[d];
      } else {
        Serial.printf(
          "// WARNING %s %s breakaway failed, keeping current min_duty\n",
          side, dir_name);
      }

      Serial.printf(
        "<param name=\"%s_ff_k_%s\">%.3f</param>\n",
        side, dir_name, ff_k);
      Serial.printf(
        "<param name=\"%s_ff_d0_%s\">%.1f</param>\n",
        side, dir_name, ff_d0);
      Serial.printf(
        "<param name=\"%s_min_duty_%s\">%.1f</param>\n",
        side, dir_name, min_duty);
    }

    float kp = g_config.motors[m].kp;
    float ki = g_config.motors[m].ki;
    if (g[m].ok) {
      kp = g[m].kp;
      ki = g[m].ki;
    }
    Serial.printf("<param name=\"%s_kp\">%.3f</param>\n", side, kp);
    Serial.printf("<param name=\"%s_ki\">%.3f</param>\n", side, ki);
  }

  Serial.println("// ==================================================");
  Serial.println("# Power cycle to run normally.");
}

}  // namespace

void run_autotune()
{
  delay(1000);
  Serial.println("# PID auto-tune mode. LIFT THE WHEELS.");
  Serial.println("# Type 'g' + Enter to start, 'x' to abort, estop to stop.");
  while (true) {
    check_abort();
    if (Serial.available() && Serial.read() == 'g') {
      break;
    }
    delay(10);
  }

  Serial.println("# Starting in 3 s...");
  wait_ms(3000);

  tune_motor(0);
  tune_motor(1);
  stop_all();
  print_xacro_params();

  while (true) {
    delay(1000);
  }
}
