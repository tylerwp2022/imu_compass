"""
imu_compass.launch.py — Launch the imu_compass_node for one robot.

USAGE:
    # Default — uses status_speed if available, falls back to gps_speed
    ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1

    # Explicitly disable status_speed gate (legacy gps_speed-only behaviour)
    ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \\
        use_status_speed:=false

    # Override the status_speed topic suffix (default: "status_speed")
    ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \\
        status_speed_topic_suffix:=platform/speed

    # Tighter speed gate, more smoothing for noisy GPS environments
    ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \\
        min_calibration_speed_m_s:=1.5 calibration_ema_alpha:=0.3

SPEED GATE NOTE:
    When use_status_speed=true (default), the node subscribes to
    /{robot_name}/{status_speed_topic_suffix} and uses hardware odometry as the
    calibration gate. This eliminates false calibration updates caused by GPS
    noise at standstill.

    If no status_speed message arrives (e.g. platform doesn't publish the topic),
    the node automatically falls back to gps_speed and emits a throttled warning.
    Flip use_status_speed:=false to silence the warning and lock to gps_speed.

    When status_speed is active, min_calibration_speed_m_s can be set much lower
    (e.g. 0.3 m/s) because odometry doesn't have a GPS noise floor. When using
    gps_speed, it typically needs to be 1.0–2.0 m/s to clear the noise floor —
    use gps_noise_characterizer.py to measure the right value for your hardware.

ARGUMENTS:
    robot_name                     — Robot namespace (required). e.g. warthog1
    use_status_speed               — Use hardware odometry speed as calibration gate.
                                     Falls back to gps_speed if topic is absent.
                                     (default: true)
    status_speed_topic_suffix      — Topic suffix for hardware speed topic, under
                                     /{robot_name}/. (default: "status_speed")
    min_calibration_speed_m_s      — Minimum speed (m/s) to allow calibration update.
                                     With status_speed: can be ~0.3 m/s.
                                     With gps_speed: typically 1.0–2.0 m/s.
                                     (default: 0.3)
    max_calibration_yaw_rate_rad_s — Maximum |yaw rate| (rad/s) during calibration.
                                     (default: 0.1 rad/s ≈ 5.7 deg/s)
    min_gps_heading_distance_m     — Minimum GPS displacement (m) between fixes
                                     for bearing computation. (default: 0.5m)
    gps_heading_min_time_delta_s   — Minimum time (s) between GPS fixes used
                                     for bearing computation. (default: 0.1)
    calibration_ema_alpha          — EMA smoothing weight for calibration offset.
                                     1.0 = no smoothing, 0.3 = strong smoothing.
                                     (default: 0.5)

NOTES:
    This node depends on gps_speed_node being running for the same robot.
    Launch gps_speed_node first, or combine in a higher-level launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def _petaar26(filename: str) -> dict:
    """Load a petaar26 config YAML from the installed share directory."""
    path = os.path.join(
        get_package_share_directory('petaar26'),
        'config', filename
    )
    with open(path) as f:
        return yaml.safe_load(f)


_hw = _petaar26('hardware.yaml')['hardware']


def generate_launch_description():

    # -------------------------------------------------------------------------
    # Declare launch arguments
    # -------------------------------------------------------------------------
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        description="Robot namespace (e.g. warthog1)."
    )

    # use_status_speed — controls which speed signal gates calibration.
    # Default true: node subscribes to status_speed and prefers it over gps_speed.
    # Set false to restore legacy behaviour (gps_speed only, no status_speed sub).
    use_status_speed_arg = DeclareLaunchArgument(
        "use_status_speed",
        default_value="true",
        description=(
            "When true, subscribe to /{robot_name}/{status_speed_topic_suffix} "
            "and use hardware odometry as the calibration speed gate. "
            "Eliminates false calibrations from GPS noise at standstill. "
            "Automatically falls back to gps_speed if the topic is absent. "
            "Set false for gps_speed-only legacy behaviour."
        )
    )

    # status_speed topic suffix — the platform-specific path segment after /{robot_name}/
    status_speed_topic_suffix_arg = DeclareLaunchArgument(
        "status_speed_topic_suffix",
        default_value="status_speed",
        description=(
            "Topic path suffix for hardware speed, under /{robot_name}/. "
            "Only used when use_status_speed=true. "
            "Node subscribes to /{robot_name}/{status_speed_topic_suffix}. "
            "Default: 'status_speed'. Override if your platform publishes "
            "to a different path (e.g. 'platform/speed')."
        )
    )

    # GPS topic suffix — selects between GeoFog and u-blox hardware.
    gps_topic_suffix_arg = DeclareLaunchArgument(
        "gps_topic_suffix",
        default_value=_hw['gps_topic_suffix'],
        description=(
            "GPS topic path suffix within the robot namespace. "
            "Node subscribes to /{robot_name}/{gps_topic_suffix} for calibration. "
            "Options: sensors/geofog/gps/fix (GeoFog, NAI_2) "
            "or sensors/ublox/fix (u-blox, NAI_3/NAI_4)."
        )
    )

    # IMU topic suffix — deployment-specific driver path.
    imu_topic_suffix_arg = DeclareLaunchArgument(
        "imu_topic_suffix",
        default_value=_hw['imu_topic_suffix'],
        description=(
            "IMU topic path suffix within the robot namespace. "
            "Node subscribes to /{robot_name}/{imu_topic_suffix}."
        )
    )

    # min_calibration_speed_m_s — default lowered to 0.3 from the legacy 1.0
    # because with status_speed the noise floor is 0.0 and we can afford to
    # calibrate at slow creep speeds. If use_status_speed=false, override to
    # 1.0–2.0 m/s based on gps_noise_characterizer.py results.
    min_speed_arg = DeclareLaunchArgument(
        "min_calibration_speed_m_s",
        default_value="0.3",
        description=(
            "Minimum speed (m/s) required to update calibration. "
            "With use_status_speed=true: 0.3 m/s is typical (no noise floor). "
            "With use_status_speed=false: set to 99.9th percentile standstill "
            "GPS noise speed × 1.3 from gps_noise_characterizer.py output."
        )
    )

    max_yaw_rate_arg = DeclareLaunchArgument(
        "max_calibration_yaw_rate_rad_s",
        default_value="0.1",
        description=(
            "Maximum |yaw rate| (rad/s) during calibration. "
            "Default 0.1 rad/s ≈ 5.7 deg/s."
        )
    )

    min_distance_arg = DeclareLaunchArgument(
        "min_gps_heading_distance_m",
        default_value="0.5",
        description=(
            "Minimum GPS displacement (m) between fixes for bearing computation. "
            "Larger = less frequent but more accurate calibration updates."
        )
    )

    min_time_delta_arg = DeclareLaunchArgument(
        "gps_heading_min_time_delta_s",
        default_value="0.1",
        description="Minimum time (s) between GPS fixes used for bearing."
    )

    ema_alpha_arg = DeclareLaunchArgument(
        "calibration_ema_alpha",
        default_value="0.5",
        description=(
            "EMA smoothing weight for the calibration offset. "
            "1.0 = no smoothing. 0.3 = strong smoothing (noisy GPS)."
        )
    )

    # -------------------------------------------------------------------------
    # Node
    # -------------------------------------------------------------------------
    imu_compass_node = Node(
        package="imu_compass",
        executable="imu_compass_node",
        name="imu_compass_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            "robot_name":                     LaunchConfiguration("robot_name"),
            "use_status_speed":               LaunchConfiguration("use_status_speed"),
            "status_speed_topic_suffix":      LaunchConfiguration("status_speed_topic_suffix"),
            "gps_topic_suffix":               LaunchConfiguration("gps_topic_suffix"),
            "imu_topic_suffix":               LaunchConfiguration("imu_topic_suffix"),
            "min_calibration_speed_m_s":      LaunchConfiguration("min_calibration_speed_m_s"),
            "max_calibration_yaw_rate_rad_s": LaunchConfiguration("max_calibration_yaw_rate_rad_s"),
            "min_gps_heading_distance_m":     LaunchConfiguration("min_gps_heading_distance_m"),
            "gps_heading_min_time_delta_s":   LaunchConfiguration("gps_heading_min_time_delta_s"),
            "calibration_ema_alpha":          LaunchConfiguration("calibration_ema_alpha"),
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_name_arg,
        use_status_speed_arg,
        status_speed_topic_suffix_arg,
        gps_topic_suffix_arg,
        imu_topic_suffix_arg,
        min_speed_arg,
        max_yaw_rate_arg,
        min_distance_arg,
        min_time_delta_arg,
        ema_alpha_arg,
        imu_compass_node,
    ])
