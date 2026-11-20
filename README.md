# slabs_explorer

ROS 2 Jazzy stack for the Explorer 1 differential-drive robot.

- [YDLidar SDK build/install](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)
- [ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)

## Running the robot

```bash
# Start the robot: hardware, controllers, EKF, and lidar.
ros2 launch explorer_1_bringup explorer_1.launch.py
```

```bash
# In a second terminal, start online SLAM + Nav2.
ros2 launch explorer_1_bringup autonomy.launch.py
```

Add `use_rviz:=false` if you do not want the Nav2 RViz view. In RViz, use the
**Nav2 Goal** tool to send a `2D Goal Pose`.

## Saving the map

While `autonomy.launch.py` is running:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

This writes `my_map.pgm` and `my_map.yaml`.

## Safety notes

- Do not run teleop and autonomous navigation at the same time; both publish
  velocity commands on `/diff_drive_controller/cmd_vel`.
- Initial Nav2 limits are conservative: ~0.15 m/s and ~0.75 rad/s. Tune on
  your hardware after validating in a clear, supervised space.

