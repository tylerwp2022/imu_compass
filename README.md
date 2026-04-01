# imu_compass

A ROS2 Jazzy node that publishes a calibrated compass heading by fusing IMU orientation with GPS-derived bearing. The IMU provides smooth, high-rate output; GPS provides drift-free absolute heading reference whenever the robot is moving in a straight line.

## How It Works

### Coordinate Convention

The MicroStrain EKF IMU publishes orientation in **ENU** (East-North-Up) convention — yaw = 0° means facing East, and positive yaw is counter-clockwise. This node converts to **compass/NED** convention throughout:

- **0° = North, 90° = East, clockwise-positive**
- Conversion: `compass_heading = 90° - ENU_yaw_deg`

This conversion is applied once inside `yaw_from_quaternion()`. Everything downstream — the calibration offset, the published heading — uses compass convention.

### Calibration

GPS bearing between two successive fixes gives a ground-truth heading (direction of travel). The difference between that bearing and the IMU's compass heading is the mounting misalignment offset:

```
offset = normalize(gps_heading - imu_compass_heading)   → [-180°, 180°]
```

This offset is stored and applied every IMU tick:

```
calibrated_heading = normalize(imu_compass_heading + offset)   → [0°, 360°)
```

The offset is updated continuously during straight-line travel and smoothed with an exponential moving average (EMA) to reject GPS noise.

### Calibration Conditions

All four conditions must be true simultaneously for a calibration update to occur:

1. `gps_speed >= min_calibration_speed_m_s` — robot is actually translating
2. `|yaw_rate| <= max_calibration_yaw_rate_rad_s` — moving straight, not turning
3. GPS fix is valid (`STATUS_FIX` or better)
4. Sufficient time and distance since the last GPS sample used for bearing

If the robot is turning or stopped, the GPS anchor is reset so the next bearing is computed between two points both taken during straight-line travel.

---

## Dependencies

- ROS2 Jazzy
- `rclcpp`
- `sensor_msgs` (`Imu`, `NavSatFix`)
- `std_msgs` (`Float64`, `String`)
- [`gps_speed`](../gps_speed) node — must be running for the same robot, as this node subscribes to `/{robot_name}/gps_speed`

---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/{robot_name}/sensors/microstrain/ekf/imu/data` | `sensor_msgs/Imu` | Quaternion orientation (ENU) + angular velocity |
| `/{robot_name}/sensors/ublox/fix` | `sensor_msgs/NavSatFix` | GPS fix used for calibration bearing |
| `/{robot_name}/gps_speed` | `std_msgs/Float64` | Pre-computed ground speed from `gps_speed` node |

All subscribers use `SensorDataQoS` (best effort, volatile).

### Published

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/{robot_name}/compass` | `std_msgs/Float64` | Reliable, depth 10 | Calibrated heading in degrees `[0, 360)`, CW from North. `-1.0` when uncalibrated. Published at IMU rate. |
| `/{robot_name}/compass/status` | `std_msgs/String` | Reliable, depth 10 | Human-readable calibration state (see below). Published at IMU rate. |

#### Status topic values

```
UNCALIBRATED - Begin moving to calibrate
Calibrated - Last update: 2024-03-15 14:23:07 UTC
```

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_name` | string | *(required)* | Robot namespace, e.g. `warthog1` |
| `min_calibration_speed_m_s` | double | `1.0` | Minimum ground speed (m/s) required to update calibration |
| `max_calibration_yaw_rate_rad_s` | double | `0.1` | Maximum \|yaw rate\| (rad/s) during calibration. Ensures straight-line travel. `0.1 rad/s ≈ 5.7°/s` |
| `min_gps_heading_distance_m` | double | `0.5` | Minimum GPS displacement (m) between fixes for a valid bearing. Lower = more frequent updates but noisier. |
| `gps_heading_min_time_delta_s` | double | `0.1` | Minimum time (s) between GPS fixes used for bearing |
| `calibration_ema_alpha` | double | `0.5` | EMA smoothing weight. `1.0` = no smoothing (instant update). `0.3` = strong smoothing. Lower values require more driving to converge. |

---

## Launch

```bash
# Default parameters
ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1

# Tighter speed gate, more smoothing for noisy GPS environments
ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \
    min_calibration_speed_m_s:=1.5 \
    calibration_ema_alpha:=0.3
```

This node is also included in `stack_launch.xml` and launches automatically with the robot stack:

```bash
# Launch with compass enabled (default)
ros2 launch phoenix_launch stack_launch.xml name:=warthog1

# Disable for debugging
ros2 launch phoenix_launch stack_launch.xml name:=warthog1 imu_compass:=false
```

---

## Monitoring

```bash
# Calibrated heading (float)
ros2 topic echo /warthog1/compass

# Calibration status
ros2 topic echo /warthog1/compass/status

# Publish rate (should match IMU rate, ~100+ Hz)
ros2 topic hz /warthog1/compass
```

Expected output before calibration:

```
# /warthog1/compass
data: -1.0

# /warthog1/compass/status
data: UNCALIBRATED - Begin moving to calibrate
```

Expected output after driving straight at >= 1 m/s:

```
# /warthog1/compass
data: 247.3

# /warthog1/compass/status
data: 'Calibrated - Last update: 2024-03-15 14:23:07 UTC'
```

---

## Troubleshooting

**Heading never calibrates**
- Check that `gps_speed` node is running: `ros2 topic echo /{robot_name}/gps_speed`
- Verify GPS fix: `ros2 topic echo /{robot_name}/sensors/ublox/fix` — `status.status` must be `>= 0`
- Drive faster or lower `min_calibration_speed_m_s`
- Drive straighter or raise `max_calibration_yaw_rate_rad_s`

**Heading drifts after calibration**
- Lower `calibration_ema_alpha` (e.g. `0.3`) to smooth out noisy GPS bearing samples
- Increase `min_gps_heading_distance_m` so bearing is computed over a longer baseline

**Subscriber receives no data**
- `/compass` and `/compass/status` use **reliable** QoS. Subscribers must use a compatible QoS policy (reliable or `rclcpp::QoS(N)`). A best-effort subscriber will silently fail to connect — check `ros2 topic info /{robot_name}/compass --verbose` for QoS mismatches.

---

## Package Structure

```
imu_compass/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── imu_compass/
│       └── imu_compass_node.hpp
├── launch/
│   └── imu_compass.launch.py
└── src/
    └── imu_compass_node.cpp
```
