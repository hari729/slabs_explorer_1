# slabs_explorer

ROS 2 Jazzy stack for the Explorer 1 differential-drive robot.

- [YDLidar SDK build/install](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)
- [ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)

## Running the robot

On the robot:

```bash
# Start the robot: hardware, controllers, EKF, and lidar.
ros2 launch explorer_1_bringup explorer_1.launch.py
```

```bash
# Start online SLAM + Nav2. The controller plugin defaults to MPPI;
# use controller:=rpp to switch to Regulated Pure Pursuit.
ros2 launch explorer_1_bringup autonomy.launch.py
ros2 launch explorer_1_bringup autonomy.launch.py controller:=rpp
```

On a laptop or development machine:

```bash
# Start the Nav2 RViz view and use the Nav2 Goal tool to send goals.
ros2 launch explorer_1_bringup rviz.launch.py
```

## Motor configuration and tuning

Motor parameters live in
`src/explorer_1_description/urdf/mobile_base.ros2_control.xacro`.
When `explorer_1.launch.py` starts, `esp32_hardware_interface` reads those
parameters, sends them to the ESP32 over the serial protocol, and the ESP32
stores them in NVS flash. After the first launch the values persist across
power cycles, so you can edit them without reflashing the firmware.

Editable parameters:

| Parameter | Meaning |
|---|---|
| `ticks_per_revolution` | Encoder ticks per wheel revolution |
| `max_abs_rpm` | Maximum commanded wheel RPM |
| `integral_limit` | PI integrator anti-windup clamp |
| `<side>_ff_k_fwd` / `_rev` | Feed-forward slope: duty per RPM |
| `<side>_ff_d0_fwd` / `_rev` | Feed-forward deadband offset (duty) |
| `<side>_min_duty_fwd` / `_rev` | Smallest duty that starts the wheel turning |
| `<side>_kp` | PI proportional gain |
| `<side>_ki` | PI integral gain |

Nav2 and controller limits, plus the skid-steer calibration that produced the
current `wheel_separation`, are documented in
[`src/explorer_1_bringup/config/TUNING.md`](src/explorer_1_bringup/config/TUNING.md).

### Onboard tuning mode

The main firmware contains the same motor identification routine as
`esp32_pid/`. To use it:

1. **Lift the wheels off the ground.** Motors will spin up to full duty.
2. Connect a USB cable and open the serial monitor at 115200 baud.
3. Pull **GPIO 27** to **GND** and power-cycle (or reset) the ESP32.
4. The firmware prints the tuning menu. Type `g` + Enter to start.
   - Type `x` at any time to abort.
   - Press the estop switch to stop immediately.
5. After the sweep and step tests finish, the firmware prints the new
   parameters as paste-ready xacro `<param>` lines and then halts.
6. Copy the printed lines into
   `src/explorer_1_description/urdf/mobile_base.ros2_control.xacro`,
   replacing the old values.
7. Power-cycle the ESP32 (with GPIO 27 **open**) and relaunch the robot.
   The new values are pushed to NVS on startup.

> The tuner only measures PI gains from a forward step response, so `kp` and
> `ki` are per-side; feed-forward and breakaway values are per-direction.

## Saving the map

While `autonomy.launch.py` is running:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

This writes `my_map.pgm` and `my_map.yaml`.

## Safety notes

- Do not run teleop and autonomous navigation at the same time; both publish
  velocity commands on `/diff_drive_controller/cmd_vel`.
- Tuned Nav2 limits: 0.20 m/s forward, 0.8 rad/s path-following yaw, 1.5 rad/s
  in-place spin. Validate in a clear, supervised space.

