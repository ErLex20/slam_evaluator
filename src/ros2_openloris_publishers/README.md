# ros2_openloris_publishers

ROS 2 adapters for converted OpenLORIS-Scene bags:

- aligned D400 depth image and static calibration to reliable XYZI cloud;
- split D400 accelerometer and gyroscope streams to one `sensor_msgs/Imu`;
- wheel odometry normalization and `base_odom -> base_link` TF;
- normalized ground-truth pose, path, and TF;
- native ROS 2 static D400 transforms and a reliable simulation-clock bridge;
- TUM trajectory recording for SLAM output.

Build and launch the adapters with a converted bag:

```bash
colcon build --packages-select ros2_openloris_publishers
source install/setup.bash
ros2 launch ros2_openloris_publishers openloris_publishers.launch.py \
  play_bag:=true bag_path:=/home/neo/workspace/office1-1
```

The player waits two seconds before publishing because OpenLORIS records
camera calibration and static transforms only once.

The `slam_evaluator` OpenLORIS launch always uses `ekf_global` as LIO-SAM's
motion prior. The EKF is independent of LIO-SAM and fuses normalized wheel
pose/twist with D400 yaw rate. LIO-SAM's pose-fix input is disabled, and `/gt`
is used only for visualization and evaluation.
