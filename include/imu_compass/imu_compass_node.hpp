#ifndef IMU_COMPASS__IMU_COMPASS_NODE_HPP
#define IMU_COMPASS__IMU_COMPASS_NODE_HPP

//==============================================================================
// imu_compass_node.hpp — ROS2 Jazzy Node
//==============================================================================
//
// PURPOSE:
// --------
// Publishes a calibrated compass heading by fusing IMU orientation with a
// GPS-derived heading offset. The IMU gives smooth, high-rate orientation;
// GPS gives drift-free absolute heading when the robot is moving.
//
// COORDINATE CONVENTION:
// ----------------------
// The MicroStrain EKF IMU publishes orientation in ENU world frame:
//   - Yaw = 0°  → facing East  (East-North-Up, CCW-positive)
//
// This node converts to compass / NED convention throughout:
//   - Heading = 0°  → North, 90° → East, CW-positive
//   - Conversion: compass_heading = normalize_360(90° - ENU_yaw_deg)
//
// Using compass convention consistently means the calibration offset is a
// simple additive constant (mounting misalignment only), which is correct
// at ALL headings. Without the conversion, the offset computed while moving
// East would give wrong results when facing North.
//
// CALIBRATION CONCEPT:
// --------------------
//   GPS bearing between two successive fixes gives ground-truth heading
//   (direction of travel). IMU compass heading is the sensor's reported
//   orientation in compass convention. The difference is the mounting offset:
//
//     offset = normalize(gps_heading - imu_compass_heading)   → [-180, 180]
//
//   Applied every IMU tick:
//     calibrated_heading = normalize(imu_compass_heading + offset)  → [0, 360)
//
//   Offset is continuously updated and smoothed via EMA to track slow IMU
//   drift and reject GPS noise.
//
// CALIBRATION CONDITIONS (all must be true simultaneously):
//   1. gps_speed >= min_calibration_speed_m_s  (robot is translating)
//   2. |imu yaw rate| <= max_calibration_yaw_rate_rad_s  (moving straight-ish)
//      WHY a separate stricter yaw gate here: even if gps_speed is non-zero
//      during a fast arc, the GPS bearing on an arc doesn't match the robot's
//      instantaneous heading — so we'd compute a wrong offset. Requiring low
//      yaw rate ensures we only calibrate during straight-line travel.
//   3. GPS fix is valid (status >= STATUS_FIX)
//   4. Sufficient time and distance since the last GPS sample used for heading
//
// SUBSCRIBES:
//   /{robot_name}/sensors/microstrain/ekf/imu/data   [sensor_msgs/msg/Imu]
//       Quaternion orientation → extract ENU yaw → convert to compass heading.
//       Also angular_velocity.z for the calibration yaw rate gate.
//
//   /{robot_name}/sensors/ublox/fix                  [sensor_msgs/msg/NavSatFix]
//       Successive fixes → GPS bearing. Only used when calibration conditions met.
//
//   /{robot_name}/gps_speed                          [std_msgs/msg/Float64]
//       Pre-computed ground speed. Gate: only calibrate when >= threshold.
//
// PUBLISHES:
//   /{robot_name}/compass         [std_msgs/msg/Float64]
//       Calibrated heading in degrees [0, 360), CW from North.
//       -1.0 when uncalibrated. Published at IMU rate.
//       Suitable for BT node InputPorts and any node doing math on heading.
//
//   /{robot_name}/compass/status  [std_msgs/msg/String]
//       Human-readable calibration state. Published at IMU rate.
//       Uncalibrated: "UNCALIBRATED - Begin moving to calibrate"
//       Calibrated:   "Calibrated - Last update: 2024-03-15 14:23:07 UTC"
//
// PARAMETERS:
//   robot_name                      (string, required)
//       Robot namespace, e.g. "warthog1".
//
//   min_calibration_speed_m_s       (double, 1.0)
//       Minimum gps_speed to allow calibration update.
//
//   max_calibration_yaw_rate_rad_s  (double, 0.1)
//       Maximum |yaw rate| during calibration. Ensures straight-line heading
//       reference. 0.1 rad/s ≈ 5.7°/s.
//
//   min_gps_heading_distance_m      (double, 0.5)
//       Minimum distance between GPS fixes used to compute a bearing.
//
//   gps_heading_min_time_delta_s    (double, 0.1)
//       Minimum time between GPS fixes used for bearing computation.
//
//   calibration_ema_alpha           (double, 0.5)
//       EMA weight for calibration offset smoothing. 1.0 = no smoothing
//       (each GPS bearing immediately replaces the offset). 0.3 = strong
//       smoothing. Reduces impact of GPS multipath / noisy bearing samples.
//
// LAUNCH EXAMPLE:
//   ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1
//
// TOPIC EXAMPLES:
//   $ ros2 topic echo /warthog1/compass
//   data: "090.0°"
//   ---
//   $ ros2 topic echo /warthog1/compass/heading
//   data: 90.0
//   ---
//
//==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <cmath>
#include <ctime>
#include <optional>
#include <string>

class ImuCompassNode : public rclcpp::Node
{
public:
    explicit ImuCompassNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    //==========================================================================
    // CALLBACKS
    //==========================================================================

    /// Primary output callback — runs at IMU rate, publishes calibrated heading.
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /// GPS callback — computes GPS bearing and updates calibration offset when
    /// conditions are met.
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /// gps_speed callback — caches latest speed for calibration gating.
    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg);

    //==========================================================================
    // MATH HELPERS
    //==========================================================================

    /// Extract yaw from an IMU quaternion and convert to compass heading [0, 360).
    ///
    /// Converts from ENU convention (yaw=0 → East, CCW-positive) to
    /// compass/NED convention (heading=0 → North, CW-positive):
    ///   compass_heading = normalize_360(90° - ENU_yaw_deg)
    static double yaw_from_quaternion(double x, double y, double z, double w);

    /// GPS bearing in degrees [0, 360) from point 1 → point 2.
    /// Returns compass heading (0=North, CW-positive), matching yaw_from_quaternion().
    static double gps_bearing_deg(double lat1_deg, double lon1_deg,
                                  double lat2_deg, double lon2_deg);

    /// Haversine distance in meters between two WGS-84 points.
    static double haversine_distance_m(double lat1_deg, double lon1_deg,
                                       double lat2_deg, double lon2_deg);

    /// Normalize an angle in degrees to [-180, 180].
    static double normalize_180(double deg);

    /// Normalize an angle in degrees to [0, 360).
    static double normalize_360(double deg);

    static constexpr double deg2rad(double d) { return d * M_PI / 180.0; }
    static constexpr double rad2deg(double r) { return r * 180.0 / M_PI; }

    //==========================================================================
    // INTERNAL STATE
    //==========================================================================

    // --- ROS interfaces ---
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr        speed_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           compass_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            status_pub_;

    // --- Parameters ---
    std::string robot_name_;
    double min_calibration_speed_m_s_;
    double max_calibration_yaw_rate_rad_s_;
    double min_gps_heading_distance_m_;
    double gps_heading_min_time_delta_s_;
    double calibration_ema_alpha_;

    // --- Calibration state ---
    // WHY optional: we want a clear "not yet calibrated" sentinel distinct
    // from an offset of 0.0 (which is a valid calibrated state).
    std::optional<double> calibration_offset_deg_;

    // Human-readable UTC timestamp of the most recent successful calibration
    // update. Published on /compass/status when calibrated.
    // Format: "Calibrated - Last update: 2024-03-15 14:23:07 UTC"
    // Empty string until first calibration — status topic publishes the
    // UNCALIBRATED message instead.
    std::string last_calibration_time_str_;

    // --- Latest sensor values (written by callbacks, read cross-callback) ---
    // WHY atomic: these are written by their own callbacks and read by
    // gps_callback. With a single-threaded executor they never truly race,
    // but atomic is cheap and makes the cross-callback reads safe and clear.
    std::atomic<double> latest_speed_m_s_{0.0};
    std::atomic<double> latest_yaw_rate_rad_s_{0.0};

    // Cached IMU compass heading in NED convention [0, 360), updated every
    // IMU tick. Used by gps_callback to compute the calibration offset.
    // Named "compass" (not "yaw") to emphasize it is already in compass
    // convention — do NOT add the 90° offset again in gps_callback.
    std::atomic<double> latest_imu_compass_deg_{0.0};

    // --- Previous GPS sample used for bearing computation ---
    struct GpsSample {
        double lat_deg;
        double lon_deg;
        rclcpp::Time stamp;
    };
    std::optional<GpsSample> prev_gps_sample_;
};

#endif  // IMU_COMPASS__IMU_COMPASS_NODE_HPP
