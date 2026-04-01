"""
imu_compass.launch.py — Launch the imu_compass_node for one robot.

USAGE:
    ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1

    # Tighter speed gate, more smoothing for noisy GPS environments
    ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \\
        min_calibration_speed_m_s:=1.5 calibration_ema_alpha:=0.3

ARGUMENTS:
    robot_name                     — Robot namespace (required). e.g. warthog1
    min_calibration_speed_m_s      — Minimum gps_speed (m/s) to allow calibration
                                     update (default: 1.0)
    max_calibration_yaw_rate_rad_s — Maximum |yaw rate| (rad/s) during calibration.
                                     Ensures straight-line heading reference.
                                     (default: 0.1 rad/s ≈ 5.7 deg/s)
    min_gps_heading_distance_m     — Minimum GPS displacement (m) between fixes
                                     used to compute a calibration bearing.
                                     (default: 0.5m)
    gps_heading_min_time_delta_s   — Minimum time (s) between GPS fixes used
                                     for bearing computation (default: 0.1)
    calibration_ema_alpha          — EMA smoothing weight for calibration offset.
                                     1.0 = no smoothing (instant update).
                                     0.3 = strong smoothing (noise rejection).
                                     (default: 0.5)

NOTES:
    This node depends on gps_speed_node being running for the same robot.
    Launch gps_speed_node first, or combine in a higher-level launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------------------------------------------------------------
    # Declare launch arguments
    # -------------------------------------------------------------------------
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        description="Robot namespace (e.g. warthog1)."
    )

    min_speed_arg = DeclareLaunchArgument(
        "min_calibration_speed_m_s",
        default_value="1.0",
        description="Minimum gps_speed (m/s) required to update calibration."
    )

    max_yaw_rate_arg = DeclareLaunchArgument(
        "max_calibration_yaw_rate_rad_s",
        default_value="0.1",
        description=(
            "Maximum |yaw rate| (rad/s) during calibration. "
            "Ensures straight-line travel for accurate GPS bearing. "
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
            "1.0 = no smoothing (each GPS bearing immediately replaces offset). "
            "0.3 = strong smoothing (rejects GPS multipath / noisy bearings). "
            "Lower values require more straight-line driving to converge."
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
        min_speed_arg,
        max_yaw_rate_arg,
        min_distance_arg,
        min_time_delta_arg,
        ema_alpha_arg,
        imu_compass_node,
    ])
