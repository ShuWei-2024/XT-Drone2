# astra_gz_sensors

Stage-2 Gazebo Harmonic sensor boundary for Astra. The model merges the PX4
`x500` model and adds native Gazebo Sim sensors: a regular 16 x 720 Mid-360
equivalent `gpu_lidar`, a 640 x 480 RGB camera, a 640 x 480 depth camera, and
a 100 Hz IMU. The lidar is explicitly an equivalent model; it does not model
Livox's non-repetitive scan timing.

The model is held static for this sensor-only validation stage. It does not
start PX4, Micro XRCE Agent, FAST-LIVO2, or EGO-Planner.

## Run

```bash
source /opt/ros/jazzy/setup.bash
source /home/cyf/dev/astra_ros2_ws/install/setup.bash
ros2 launch astra_gz_sensors gz_sensors.launch.py rviz:=false
```

Gazebo checks:

```bash
gz topic -l
gz topic -f -t /sensors/lidar/points
gz topic -f -t /sensors/camera/color/image_raw
gz topic -f -t /sensors/camera/depth/image_raw
gz topic -f -t /sensors/imu/data
```

Stable ROS 2 interfaces:

| ROS 2 topic | Type | Expected rate |
| --- | --- | --- |
| `/sensors/lidar/points` | `sensor_msgs/msg/PointCloud2` | 10 Hz |
| `/sensors/camera/color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| `/sensors/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| `/sensors/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz |
| `/sensors/imu/data` | `sensor_msgs/msg/Imu` | 100 Hz |

The external calibration is recorded in `config/static_sensor_tf.yaml`. The
published tree is `map -> odom -> base_link` followed by the sensor frames.
