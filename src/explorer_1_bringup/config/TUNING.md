# Explorer 1 Nav2 / Controller Tuning Notes

These values were calibrated on the real robot after observing that Nav2 could not
reliably turn in place while keyboard teleop worked. The root cause is the large
lateral scrub of the 4-wheel skid-steer drivetrain: the wheel odometry over-reads
yaw by roughly 3× compared to the EKF/IMU.

> **Last calibration:** 2026-11-24  
> **Surface:** hard floor (tile/wood/concrete) — values will differ on carpet.

---

## 1. Effective wheel separation (the only measured value)

A 90° in-place spin was commanded through Nav2 `/spin`:

| Source | Yaw delta |
|---|---|
| EKF / IMU (`/odometry/filtered`) | +1.576 rad (90.3°) |
| Wheel odometry (`/diff_drive_controller/odom`) | +4.827 rad (276.5°) |

The wheel odom computes yaw from the wheel-speed difference and the configured
separation `b`:

```
wheel_yaw = (v_right - v_left) / b * t
```

The real robot turns as if the same wheel-speed difference were divided by a
larger **effective separation** `b_eff` that lumps lateral scrubbing:

```
b_eff = b * (wheel_yaw / ekf_yaw)
      = 0.165 * (4.827 / 1.576)
      = 0.505 m
```

So `diff_drive_controller.wheel_separation` was changed from `0.165` to `0.505`.

This is **not** the physical track width; it is a calibration constant. It fixes
both:

* the wheel-odometry yaw reporting, and
* the conversion from `cmd_vel.angular.z` into wheel commands.

---

## 2. Physical limits from motor data

From `mobile_base.ros2_control.xacro`:

```
max_abs_rpm = 112.5 RPM
wheel_radius = 0.035 m
```

Convert to linear/wheel limits:

```
max_wheel_speed  = 112.5 * 2π / 60 = 11.78 rad/s
max_linear_speed = 11.78 * 0.035   = 0.412 m/s
max_spin_rate    = 2 * 0.412 / 0.505 = 1.633 rad/s
```

These numbers set the hard ceiling for every velocity limit downstream.

---

## 3. Parameter derivations

### `explorer_1_controllers.yaml`

| Parameter | Old | New | Derivation |
|---|---|---|---|
| `wheel_separation` | 0.165 | **0.505** | Measured slip ratio 3.06 (Section 1) |
| `linear.x.max_velocity` | 0.5 | **0.4** | 97% of physical max 0.412 m/s. Old 0.5 exceeded it. |
| `linear.x.min_velocity` | -0.5 | **-0.4** | Symmetric to forward limit |
| `angular.z.max_velocity` | 3.0 | **1.5** | 92% of physical max 1.633 rad/s. Old 3.0 needed 207 RPM wheels. |
| `angular.z.min_velocity` | -3.0 | **-1.5** | Symmetric to spin limit |

### `nav2_params.yaml` — MPPI controller

| Parameter | Old | New | Derivation |
|---|---|---|---|
| `vx_max` | 0.35 | **0.20** | Common cruise speed, 50% of physical max, leaves braking/steering headroom |
| `vx_min` | -0.15 | **-0.15** | unchanged |
| `wz_max` | 2.0 | **0.8** | ~50% of 1.633 rad/s. Path following does not need max spin; keeps the skid steer in a tractable region. At 0.2 m/s this gives a 0.25 m min turn radius. |
| `wz_std` | 0.6 | **0.4** | Sampling heuristic: `wz_max / 2`, so ~95% of samples stay within ±wz_max |

### `nav2_params.yaml` — Regulated Pure Pursuit (RPP)

| Parameter | Old | New | Derivation |
|---|---|---|---|
| `desired_linear_vel` | 0.35 | **0.20** | Same cruise speed as MPPI |
| `rotate_to_heading_angular_vel` | 1.8 | **1.2** | 73% of 1.633 rad/s. Old 1.8 exceeded the physical max. |

### `nav2_params.yaml` — behavior server

| Parameter | Old | New | Derivation |
|---|---|---|---|
| `max_rotational_vel` | 2.0 | **1.5** | Match the controller's `angular.z.max_velocity` so behavior requests are not silently clipped |
| `min_rotational_vel` | 0.30 | **0.40** | Low-speed floor to stay above the firmware `min_duty` stick-slip region (see below) |
| `rotational_acc_lim` | 8.0 | **6.0** | Reach 1.5 rad/s in 0.25 s; softer torque step for a stick-slip drivetrain |

#### Why `min_rotational_vel = 0.40`

With `b_eff = 0.505`, a 0.4 rad/s in-place spin requires:

```
v_wheel      = wz * b_eff / 2
             = 0.4 * 0.505 / 2 = 0.101 m/s
wheel_rpm    = v_wheel / r * 60 / 2π
             = 0.101 / 0.035 * 60 / 2π = 27.6 RPM
```

Compare the firmware feed-forward duty with the `min_duty` floor:

```
left  FF = 141.3 + 7.048 * 27.6 = 336  > 270  ✓
right FF = 142.5 + 7.019 * 27.6 = 336  > 330  ✓
```

At the old floor of 0.30 rad/s the right-wheel feed-forward is below its 330
floor, so the PI controller has to wind up against the floor — the observed
lurching behavior.

The exact crossing for the right wheel is:

```
rpm_cross   = (330 - 142.5) / 7.019 = 26.7 RPM
wz_cross    = rpm_cross * 2π/60 * r * 2 / b_eff
            = 26.7 * 0.10472 * 0.035 * 2 / 0.505
            = 0.388 rad/s
```

0.40 was chosen as the round value just above 0.388.

### `nav2_params.yaml` — velocity smoother

| Parameter | Old | New | Derivation |
|---|---|---|---|
| `max_velocity` | `[0.35, 0.0, 2.0]` | **`[0.20, 0.0, 1.5]`** | Tightest upstream limits: MPPI `vx_max` and controller angular cap |
| `min_velocity` | `[-0.15, 0.0, -2.0]` | **`[-0.15, 0.0, -1.5]`** | Matches MPPI `vx_min` and controller angular cap |
| `max_accel` angular | `8.0` | **6.0** | Match `behavior_server.rotational_acc_lim` |
| `max_decel` angular | `-8.0` | **-6.0** | Match `behavior_server.rotational_acc_lim` |

Linear accel/decel (`[2.0, 0.0, 8.0]` / `[-2.0, 0.0, -8.0]`) was left unchanged.

### `mobile_base.ros2_control.xacro`

| Parameter | Old | New | Derivation |
|---|---|---|---|
| command interface `velocity` limits | `±1.0` | **`±12.0`** | Physical max wheel speed is 11.78 rad/s. `±1.0` rad/s would have limited linear speed to 0.035 m/s if enforced. The firmware clamps to `max_abs_rpm` anyway. |

---

## 4. Validation procedure

After restarting the robot stack, verify in this order:

1. **Constant spin test**  
   Publish `cmd_vel.angular.z = 0.5 rad/s` for 5 s.  
   Expected EKF yaw delta: ≈ 2.5 rad.  
   If actual > 2.5 rad, `wheel_separation` is too high; if less, too low.

2. **90° spin test**  
   Send a Nav2 `/spin` goal of 90°.  
   Expected: wheel-odom yaw delta ≈ EKF yaw delta (≈ 1.57 rad), completion in ~3–4 s.

3. **Straight-line test**  
   Command `cmd_vel.linear.x = 0.2 m/s` for 2 m.  
   Compare wheel-odom distance vs. a tape measure or lidar landmark.  
   If it is off, `wheel_radius` needs its own calibration (unchanged here).

---

## 5. When to re-tune

Re-run the 90° spin calibration if any of these change:

* Floor surface (carpet vs. hard floor changes scrub)
* Wheel/tire type or wear
* Payload weight or CG
* Motor parameters (`max_abs_rpm`, `min_duty`, PI gains)

If the constant-spin test tracks the right yaw but the motion is still
stuttery/lurching at low speed, the remaining cause is the firmware's
unconditional `min_duty` floor in `esp32_firmware/src/main.cpp` (Phase 2).
