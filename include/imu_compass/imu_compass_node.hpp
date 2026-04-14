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
//   1. effective_speed >= min_calibration_speed_m_s  (robot is translating)
//      "effective speed" is either status_speed (hardware odometry, preferred)
//      or gps_speed (GPS-derived, fallback). See use_status_speed parameter.
//   2. |imu yaw rate| <= max_calibration_yaw_rate_rad_s  (moving straight-ish)
//      WHY a separate stricter yaw gate here: even if speed is non-zero
//      during a fast arc, the GPS bearing on an arc doesn't match the robot's
//      instantaneous heading — so we'd compute a wrong offset. Requiring low
//      yaw rate ensures we only calibrate during straight-line travel.
//   3. GPS fix is valid (status >= STATUS_FIX)
//   4. Sufficient time and distance since the last GPS sample used for heading
//
// SPEED GATE — WHY status_speed IS PREFERRED OVER gps_speed:
// -----------------------------------------------------------
// gps_speed is computed by differencing successive GPS fixes (Haversine). At
// standstill, GPS position noise (multipath, atmospheric scintillation) causes
// the antenna's reported position to wander by 0.1–1.0 m per fix. This
// translates to false speed readings of 1–10 m/s that can slip through the
// min_calibration_speed_m_s gate, triggering calibration updates against a
// random GPS bearing at standstill — corrupting the offset.
//
// status_speed (wheel odometry / hardware velocity estimate) is not affected
// by GPS noise and reliably reports 0.0 when the robot is stationary. Using it
// as the speed gate eliminates standstill false calibrations entirely, and lets
// min_calibration_speed_m_s be set to a physically meaningful threshold (e.g.
// 0.3 m/s — actual motion) rather than an inflated noise-rejection value.
//
// AUTOMATIC FALLBACK:
//   If use_status_speed=true but no status_speed message has arrived yet
//   (e.g. the platform doesn't publish this topic), the node falls back to
//   gps_speed automatically and logs a throttled warning. This prevents the
//   node from being permanently blocked at startup on unsupported platforms.
//   Once a status_speed message arrives, the node switches to it immediately.
//
// SUBSCRIBES:
//   /{robot_name}/{imu_topic_suffix}                 [sensor_msgs/msg/Imu]
//       Quaternion orientation → extract ENU yaw → convert to compass heading.
//       Also angular_velocity.z for the calibration yaw rate gate.
//       Topic suffix driven by 'imu_topic_suffix' parameter.
//       Default: "sensors/microstrain/ekf/imu/data".
//
//   /{robot_name}/{gps_topic_suffix}                 [sensor_msgs/msg/NavSatFix]
//       Successive fixes → GPS bearing for calibration.
//       Only used when calibration conditions are met.
//       Topic suffix driven by 'gps_topic_suffix' parameter.
//       Default: "sensors/geofog/gps/fix".
//
//   /{robot_name}/sensors/gps_speed                  [std_msgs/msg/Float64]
//       GPS-derived ground speed from gps_speed_node.
//       Used as speed gate when use_status_speed=false, or as fallback when
//       use_status_speed=true but status_speed hasn't published yet.
//
//   /{robot_name}/{status_speed_topic_suffix}         [std_msgs/msg/Float64]
//       Hardware odometry speed in m/s. Subscribed when use_status_speed=true.
//       Preferred speed gate — not affected by GPS noise. Typically the Warthog
//       platform's wheel-encoder velocity estimate.
//       Default suffix: "status_speed".
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
//       Calibrated:   "Calibrated [status_speed] - Last update: 2024-03-15 14:23:07 UTC"
//       The tag in brackets shows which speed source is active.
//
// PARAMETERS:
//   robot_name                      (string, required)
//       Robot namespace, e.g. "warthog1".
//
//   use_status_speed                (bool, true)
//       When true, subscribe to /{robot_name}/{status_speed_topic_suffix} and
//       use it as the primary speed gate for calibration. Falls back to
//       gps_speed automatically if no status_speed message has been received.
//       Set to false to use gps_speed exclusively (legacy behaviour).
//
//   status_speed_topic_suffix       (string, "status_speed")
//       Topic path after /{robot_name}/ for the hardware speed signal.
//       Only used when use_status_speed=true.
//
//   gps_topic_suffix                (string, "sensors/geofog/gps/fix")
//       GPS topic path after /{robot_name}/. Selects GPS hardware variant.
//
//   imu_topic_suffix                (string, "sensors/microstrain/ekf/imu/data")
//       IMU topic path after /{robot_name}/.
//
//   min_calibration_speed_m_s       (double, 0.3 when status_speed active,
//                                            1.0 when gps_speed fallback)
//       Minimum speed to allow calibration update.
//       With status_speed: can be set close to actual minimum translating speed
//       (e.g. 0.3 m/s) since odometry has no noise floor at standstill.
//       With gps_speed: must be set above the GPS noise floor at standstill
//       (from gps_noise_characterizer.py, typically 0.5–2.0 m/s depending on
//       antenna placement and sky view).
//
//   max_calibration_yaw_rate_rad_s  (double, 0.1)
//       Maximum |yaw rate| during calibration. 0.1 rad/s ≈ 5.7°/s.
//
//   min_gps_heading_distance_m      (double, 0.5)
//       Minimum GPS displacement (m) between fixes for a valid bearing.
//
//   gps_heading_min_time_delta_s    (double, 0.1)
//       Minimum time (s) between GPS fixes used for bearing computation.
//
//   calibration_ema_alpha           (double, 0.5)
//       EMA smoothing weight for calibration offset updates.
//       new_offset = prev + alpha * normalize_180(new_sample - prev)
//       1.0 = instant update (no smoothing). 0.3 = strong smoothing.
//
// LAUNCH EXAMPLES:
//   # Default — uses status_speed if available, falls back to gps_speed
//   ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1
//
//   # Disable status_speed gate (legacy gps_speed-only behaviour)
//   ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \
//       use_status_speed:=false
//
//   # Override status_speed topic suffix
//   ros2 launch imu_compass imu_compass.launch.py robot_name:=warthog1 \
//       status_speed_topic_suffix:=platform/speed
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

    /// gps_speed callback — caches GPS-derived speed for use as fallback gate.
    void gps_speed_callback(const std_msgs::msg::Float64::SharedPtr msg);

    /// status_speed callback — caches hardware odometry speed for use as
    /// preferred calibration gate. Only subscribed when use_status_speed_=true.
    void status_speed_callback(const std_msgs::msg::Float64::SharedPtr msg);

    //==========================================================================
    // SPEED GATE HELPER
    //==========================================================================

    /// Returns the speed value used to gate calibration.
    ///
    /// Selection logic:
    ///   - If use_status_speed_=false  → always return gps_speed (legacy)
    ///   - If use_status_speed_=true AND status_speed_received_=true
    ///                                 → return status_speed (preferred)
    ///   - If use_status_speed_=true AND status_speed_received_=false
    ///                                 → return gps_speed (startup fallback)
    ///
    /// The fallback case is intentional: it prevents the node from being
    /// permanently blocked if the platform doesn't publish status_speed.
    /// A throttled WARN is emitted so the operator knows fallback is active.
    double effective_speed_m_s() const;

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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr        gps_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr        status_speed_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           compass_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            status_pub_;

    // --- Parameters ---
    std::string robot_name_;
    bool        use_status_speed_;              ///< Prefer status_speed over gps_speed
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
    // update, including the speed source tag for diagnostics.
    // Format: "Calibrated [status_speed] - Last update: 2024-03-15 14:23:07 UTC"
    // Empty until first calibration.
    std::string last_calibration_time_str_;

    // --- Latest sensor values (written by callbacks, read cross-callback) ---
    // WHY atomic: these are written by their own callbacks and read by
    // gps_callback. With a single-threaded executor they never truly race,
    // but atomic is cheap and makes the cross-callback reads safe and clear.

    /// GPS-derived ground speed from gps_speed_node. Updated by gps_speed_callback.
    /// Used as speed gate when use_status_speed_=false, or when status_speed
    /// hasn't published yet.
    std::atomic<double> latest_gps_speed_m_s_{0.0};

    /// Hardware odometry speed from the platform's status_speed topic.
    /// Updated by status_speed_callback. Only meaningful when
    /// status_speed_received_=true.
    std::atomic<double> latest_status_speed_m_s_{0.0};

    /// Set to true on the first incoming status_speed message.
    /// Once true, effective_speed_m_s() returns status_speed instead of gps_speed.
    /// Never reset to false — once the topic arrives, it stays available.
    std::atomic<bool> status_speed_received_{false};

    /// IMU yaw rate (rad/s). Updated by imu_callback, read by gps_callback.
    std::atomic<double> latest_yaw_rate_rad_s_{0.0};

    /// Cached IMU compass heading in NED convention [0, 360), updated every
    /// IMU tick. Used by gps_callback to compute the calibration offset.
    /// Named "compass" (not "yaw") to emphasize it is already in compass
    /// convention — do NOT add the 90° offset again in gps_callback.
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
