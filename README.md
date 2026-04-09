# imu_compass

A ROS2 Jazzy node that publishes a calibrated compass heading by fusing IMU orientation with GPS-derived bearing. The IMU provides smooth, high-rate output (~100 Hz); GPS provides drift-free absolute heading reference during straight-line travel.

## PETAAR26 Integration

This node is part of the PETAAR stack and its sensor topic defaults are driven by `petaar26/config/hardware.yaml`. You do not need to pass topic names manually — the launch file reads them from the shared config.

**To change the GPS or IMU topic** (e.g., when deploying on new hardware), edit:
```
petaar26/config/hardware.yaml
  → hardware.gps_topic_suffix
  → hardware.imu_topic_suffix
```

> **Bug fixes (applied):**
>
> 1. **Hardcoded topics** — GPS and IMU subscriber topics were hardcoded in the C++ constructor, making them unreachable by launch file parameters. They are now read from ROS parameters. The startup log confirms the actual subscribed topics.
>
> 2. **EMA smoothing not applied** — `calibration_ema_alpha` was declared in the launch file and header but the C++ was doing a bare assignment (`offset = new_offset`), ignoring alpha entirely. EMA is now correctly applied using angular delta arithmetic to avoid ±180° wrap artefacts:
>    ```cpp
>    const double delta = normalize_180(new_offset - prev_offset);
>    calibration_offset_deg_ = normalize_180(prev_offset + alpha * delta);
>    ```

---

## How It Works

### Coordinate Convention

The IMU publishes orientation in **ENU** (East-North-Up) convention — yaw = 0° means facing East, CCW-positive. This node converts to **compass/NED** convention throughout:

- **0° = North, 90° = East, clockwise-positive**
- Conversion: `compass_heading = normalize_360(90° - ENU_yaw_deg)`

> **⚠ Real robot check:** If your IMU driver publishes in NED convention (yaw = 0° = North, CW-positive), this conversion is wrong. See `petaar26/config/hardware.yaml → imu_topic_suffix` for the full check procedure and the one-line code change needed.

### Calibration

GPS bearing between two successive fixes gives a ground-truth heading. The difference between that bearing and the IMU compass heading is the mounting offset:

```
offset = normalize(gps_heading - imu_compass_heading)   → [-180°, 180°]
```

Applied every IMU tick:
```
calibrated_heading = normalize(imu_compass_heading + offset)   → [0°, 360°)
```

The offset is smoothed with EMA to reject GPS bearing noise:
```
offset = prev_offset + alpha * normalize_180(new_sample - prev_offset)
```

### Calibration Conditions

All four must be true simultaneously:

1. `gps_speed >= min_calibration_speed_m_s` — robot is translating
2. `|yaw_rate| <= max_calibration_yaw_rate_rad_s` — moving straight, not turning
3. GPS fix is valid (`STATUS_FIX` or better)
4. Sufficient time and distance since the last bearing sample

---

## Dependencies

- `rclcpp`, `sensor_msgs`, `std_msgs`
- [`gps_speed`](../gps_speed) node — must be running for the same robot (this node subscribes to `/{robot_name}/sensors/gps_speed`)
- `petaar26` package (for shared config defaults at launch time)

---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/{robot_name}/{imu_topic_suffix}` | `sensor_msgs/Imu` | Quaternion orientation (ENU) + angular velocity. Suffix from `petaar26/config/hardware.yaml` |
| `/{robot_name}/{gps_topic_suffix}` | `sensor_msgs/NavSatFix` | GPS fix for calibration bearing. Suffix from `petaar26/config/hardware.yaml` |
| `/{robot_name}/sensors/gps_speed` | `std_msgs/Float64` | Pre-computed ground speed from `gps_speed_node` |

All subscriptions use `SensorDataQoS` (best-effort, volatile).

Verify at startup — the node logs the actual resolved topics:
```
[INFO] Subscribed to IMU:   /warthog1/sensors/microstrain/ekf/imu/data
[INFO] Subscribed to GPS:   /warthog1/sensors/geofog/gps/fix
[INFO] Subscribed to speed: /warthog1/sensors/gps_speed
```

### Published

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/{robot_name}/compass` | `std_msgs/Float64` | Reliable, depth 10 | Calibrated heading in degrees `[0, 360)`, CW from North. `-1.0` when uncalibrated. Published at IMU rate. |
| `/{robot_name}/compass/status` | `std_msgs/String` | Reliable, depth 10 | Human-readable calibration state. |

Status topic values:
```
UNCALIBRATED - Begin moving to calibrate
Calibrated - Last update: 2024-03-15 14:23:07 UTC
```

---

## Parameters

| Parameter | Type | Default | Source | Description |
|---|---|---|---|---|
| `robot_name` | string | **required** | launch arg | Robot namespace, e.g. `warthog1` |
| `gps_topic_suffix` | string | from `hardware.yaml` | `hardware.gps_topic_suffix` | GPS topic path after `/{robot_name}/` |
| `imu_topic_suffix` | string | from `hardware.yaml` | `hardware.imu_topic_suffix` | IMU topic path after `/{robot_name}/` |
| `min_calibration_speed_m_s` | double | `1.0` | launch arg | Minimum ground speed (m/s) to allow calibration update |
| `max_calibration_yaw_rate_rad_s` | double | `0.1` | launch arg | Maximum yaw rate (rad/s) during calibration (~5.7°/s) |
| `min_gps_heading_distance_m` | double | `0.5` | launch arg | Minimum GPS displacement (m) between fixes for a valid bearing |
| `gps_heading_min_time_delta_s` | double | `0.1` | launch arg | Minimum time (s) between GPS fixes used for bearing |
| `calibration_ema_alpha` | double | `0.5` | launch arg | EMA weight. `1.0` = instant update, `0.3` = strong smoothing |

---

## Launch

```bash
# Default parameters (topics from hardware.yaml)
ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1

# Tighter speed gate, more smoothing for noisy GPS
ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \
    min_calibration_speed_m_s:=1.5 \
    calibration_ema_alpha:=0.3

# Monitor calibration
ros2 topic echo /warthog1/compass         # -1.0 = uncalibrated
ros2 topic echo /warthog1/compass/status
ros2 topic hz /warthog1/compass           # Should match IMU rate (~100 Hz)
```

---

## Troubleshooting

**Heading never calibrates**
- Check that `gps_speed` node is running: `ros2 topic echo /{robot_name}/sensors/gps_speed`
- Verify GPS fix: `status.status` must be `>= 0`
- Drive faster (`min_calibration_speed_m_s`) or straighter (`max_calibration_yaw_rate_rad_s`)
- Check the correct GPS topic is subscribed — look at node startup log

**Heading drifts after calibration**
- Lower `calibration_ema_alpha` (e.g. `0.3`) for stronger smoothing
- Increase `min_gps_heading_distance_m` to compute bearing over a longer baseline

**Subscribers receive no data from `/compass`**
- `/compass` uses **reliable** QoS. Subscribers with best-effort QoS will silently fail to connect. Check: `ros2 topic info /{robot_name}/compass --verbose`

**Heading is consistently offset by ~90°**
- Your IMU is likely publishing in NED convention rather than ENU. See `petaar26/config/hardware.yaml → imu_topic_suffix` for the code change required.

---

## Package Structure

```
imu_compass/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/imu_compass/
│   └── imu_compass_node.hpp
├── launch/
│   └── imu_compass.launch.py
└── src/
    └── imu_compass_node.cpp
```
