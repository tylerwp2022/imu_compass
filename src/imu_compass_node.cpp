//==============================================================================
// imu_compass_node.cpp — ROS2 Jazzy Node
//==============================================================================
// See include/imu_compass/imu_compass_node.hpp for full documentation.
//==============================================================================

#include "imu_compass/imu_compass_node.hpp"

#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <ctime>
#include <iomanip>
#include <sstream>

static constexpr double EARTH_RADIUS_M = 6'371'000.0;

//==============================================================================
// CONSTRUCTOR
//==============================================================================

ImuCompassNode::ImuCompassNode(const rclcpp::NodeOptions & options)
: Node("imu_compass_node", options)
{
    //--------------------------------------------------------------------------
    // Declare + retrieve parameters
    //--------------------------------------------------------------------------
    this->declare_parameter<std::string>("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    if (robot_name_.empty())
    {
        RCLCPP_FATAL(this->get_logger(),
            "Parameter 'robot_name' is required. "
            "Set via: --ros-args -p robot_name:=warthog1");
        throw std::runtime_error("Missing required parameter: robot_name");
    }

    // use_status_speed: when true (default), subscribe to the hardware odometry
    // speed topic and use it as the primary calibration gate. Falls back to
    // gps_speed if status_speed hasn't published yet. See header for full rationale.
    this->declare_parameter<bool>("use_status_speed", true);
    use_status_speed_ = this->get_parameter("use_status_speed").as_bool();

    // status_speed topic suffix — the segment after /{robot_name}/
    this->declare_parameter<std::string>("status_speed_topic_suffix", "status_speed");
    const std::string status_speed_suffix =
        this->get_parameter("status_speed_topic_suffix").as_string();

    this->declare_parameter<double>("min_calibration_speed_m_s",       1.0);
    this->declare_parameter<double>("max_calibration_yaw_rate_rad_s",  0.1);
    this->declare_parameter<double>("min_gps_heading_distance_m",      0.5);
    this->declare_parameter<double>("gps_heading_min_time_delta_s",    0.1);
    this->declare_parameter<double>("calibration_ema_alpha",           0.5);

    // Topic suffix parameters — see header for rationale.
    this->declare_parameter<std::string>("gps_topic_suffix",
        "sensors/geofog/gps/fix");
    this->declare_parameter<std::string>("imu_topic_suffix",
        "sensors/microstrain/ekf/imu/data");

    min_calibration_speed_m_s_      = this->get_parameter("min_calibration_speed_m_s").as_double();
    max_calibration_yaw_rate_rad_s_ = this->get_parameter("max_calibration_yaw_rate_rad_s").as_double();
    min_gps_heading_distance_m_     = this->get_parameter("min_gps_heading_distance_m").as_double();
    gps_heading_min_time_delta_s_   = this->get_parameter("gps_heading_min_time_delta_s").as_double();
    calibration_ema_alpha_          = this->get_parameter("calibration_ema_alpha").as_double();
    const std::string gps_topic_suffix = this->get_parameter("gps_topic_suffix").as_string();
    const std::string imu_topic_suffix = this->get_parameter("imu_topic_suffix").as_string();

    RCLCPP_INFO(this->get_logger(),
        "Parameters: robot_name=%s  min_cal_speed=%.2fm/s  "
        "max_cal_yaw_rate=%.3frad/s (%.1fdeg/s)  "
        "min_gps_distance=%.2fm  min_gps_time_delta=%.3fs  "
        "ema_alpha=%.2f  use_status_speed=%s",
        robot_name_.c_str(),
        min_calibration_speed_m_s_,
        max_calibration_yaw_rate_rad_s_,
        rad2deg(max_calibration_yaw_rate_rad_s_),
        min_gps_heading_distance_m_,
        gps_heading_min_time_delta_s_,
        calibration_ema_alpha_,
        use_status_speed_ ? "true" : "false");

    //--------------------------------------------------------------------------
    // Build topic names
    // WHY ABSOLUTE PATHS (leading slash): this node is launched under the
    // robot namespace, so relative topics would double-prefix. Absolute paths
    // are explicit and immune to executor-level namespace surprises.
    //--------------------------------------------------------------------------
    const std::string imu_topic          = "/" + robot_name_ + "/" + imu_topic_suffix;
    const std::string gps_topic          = "/" + robot_name_ + "/" + gps_topic_suffix;
    const std::string gps_speed_topic    = "/" + robot_name_ + "/sensors/gps_speed";
    const std::string status_speed_topic = "/" + robot_name_ + "/" + status_speed_suffix;
    const std::string compass_topic      = "/" + robot_name_ + "/compass";
    const std::string status_topic       = "/" + robot_name_ + "/compass/status";

    //--------------------------------------------------------------------------
    // Subscribers
    //--------------------------------------------------------------------------

    // IMU: high rate (~100-500 Hz). This is the primary output driver.
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            this->imu_callback(msg);
        });

    // GPS: ~9 Hz. Used only for calibration bearing computation.
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            this->gps_callback(msg);
        });

    // gps_speed: ~9 Hz. Always subscribed — either used as the primary gate
    // (use_status_speed=false) or as the startup fallback (use_status_speed=true).
    gps_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        gps_speed_topic,
        rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            this->gps_speed_callback(msg);
        });

    // status_speed: only subscribed when use_status_speed=true.
    // WHY CONDITIONAL SUBSCRIPTION: if use_status_speed=false the user has
    // explicitly opted into the legacy gps_speed-only path. Creating the
    // subscription anyway would add noise to ros2 topic info output and could
    // cause confusion if the topic doesn't exist on the platform.
    if (use_status_speed_)
    {
        // Use SensorDataQoS (best-effort) so this works whether the platform
        // publishes with best-effort or reliable — ROS2 will match on the
        // publisher's QoS. If status_speed is published reliable, a
        // best-effort subscriber will still connect and receive all messages
        // on a lossless local network (loopback / shared memory).
        status_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            status_speed_topic,
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                this->status_speed_callback(msg);
            });
    }

    //--------------------------------------------------------------------------
    // Publishers
    //--------------------------------------------------------------------------
    compass_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        compass_topic, rclcpp::QoS(10).reliable());

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        status_topic, rclcpp::QoS(10).reliable());

    //--------------------------------------------------------------------------
    // Startup log — make the active configuration immediately visible
    //--------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "Subscribed to IMU:       %s", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to GPS:       %s", gps_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to gps_speed: %s", gps_speed_topic.c_str());

    if (use_status_speed_)
    {
        RCLCPP_INFO(this->get_logger(),
            "Subscribed to status_speed: %s  (preferred speed gate — "
            "will fall back to gps_speed until first message arrives)",
            status_speed_topic.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
            "use_status_speed=false — using gps_speed as calibration gate. "
            "GPS noise at standstill may cause false calibration updates. "
            "Set use_status_speed:=true if the platform publishes %s.",
            status_speed_topic.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Publishing compass:      %s", compass_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing status:       %s", status_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
        "Status: UNCALIBRATED — move at >= %.2f m/s to calibrate.",
        min_calibration_speed_m_s_);
}

//==============================================================================
// SPEED GATE HELPER
//==============================================================================

double ImuCompassNode::effective_speed_m_s() const
{
    if (!use_status_speed_)
    {
        // Legacy mode: always use gps_speed.
        return latest_gps_speed_m_s_.load();
    }

    if (status_speed_received_.load())
    {
        // Preferred path: odometry is available and noise-free at standstill.
        return latest_status_speed_m_s_.load();
    }

    // Fallback: use_status_speed=true but no status_speed message yet.
    // This happens at startup, or if the platform doesn't publish the topic.
    // The warning is throttled to avoid log spam; it fires at most once per 10s.
    // Once status_speed_received_ flips true, this branch is never taken again.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "use_status_speed=true but no status_speed message received yet — "
        "falling back to gps_speed for calibration gate. "
        "Check that the status_speed topic is publishing. "
        "This warning disappears once the first message arrives.");

    return latest_gps_speed_m_s_.load();
}

//==============================================================================
// GPS SPEED CALLBACK — cache GPS-derived speed
//==============================================================================

void ImuCompassNode::gps_speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    latest_gps_speed_m_s_.store(msg->data);
}

//==============================================================================
// STATUS SPEED CALLBACK — cache hardware odometry speed
//==============================================================================

void ImuCompassNode::status_speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    const bool was_received = status_speed_received_.load();

    latest_status_speed_m_s_.store(msg->data);

    // Flip the received flag on the first message. Log once so the operator
    // can confirm the switch from fallback mode happened as expected.
    if (!was_received)
    {
        status_speed_received_.store(true);
        RCLCPP_INFO(this->get_logger(),
            "status_speed topic is live (first message: %.4f m/s). "
            "Calibration gate is now using hardware odometry — "
            "GPS noise at standstill will no longer trigger false calibrations.",
            msg->data);
    }
}

//==============================================================================
// IMU CALLBACK — publish compass heading at IMU rate
//==============================================================================

void ImuCompassNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    //--------------------------------------------------------------------------
    // Extract yaw from quaternion and cache it for the GPS callback to use
    // when computing the calibration offset.
    //--------------------------------------------------------------------------
    const double imu_compass_deg = yaw_from_quaternion(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    latest_imu_compass_deg_.store(imu_compass_deg);
    latest_yaw_rate_rad_s_.store(msg->angular_velocity.z);

    //--------------------------------------------------------------------------
    // Publish compass heading and status
    //
    // /compass        — Float64: calibrated heading [0, 360), or -1.0 if not
    //                   yet calibrated. Always published at IMU rate.
    // /compass/status — String: human-readable calibration state.
    //--------------------------------------------------------------------------
    auto hdg_msg = std_msgs::msg::Float64();
    auto status_msg = std_msgs::msg::String();

    if (!calibration_offset_deg_.has_value())
    {
        hdg_msg.data = -1.0;
        status_msg.data = "UNCALIBRATED - Begin moving to calibrate";
    }
    else
    {
        hdg_msg.data = normalize_360(imu_compass_deg + calibration_offset_deg_.value());
        status_msg.data = last_calibration_time_str_;
    }

    compass_pub_->publish(hdg_msg);
    status_pub_->publish(status_msg);
}

//==============================================================================
// GPS CALLBACK — update calibration offset when conditions are met
//==============================================================================

void ImuCompassNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    //--------------------------------------------------------------------------
    // Gate 1: Valid GPS fix
    //--------------------------------------------------------------------------
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No GPS fix — skipping calibration update.");
        // Don't advance prev_gps_sample_ on a bad fix.
        return;
    }

    //--------------------------------------------------------------------------
    // Gate 2: Speed sufficient for reliable GPS heading.
    //
    // effective_speed_m_s() selects the best available speed signal:
    //   - status_speed (odometry) if use_status_speed=true and topic is live
    //   - gps_speed otherwise (with a logged fallback warning)
    //
    // WHY RESET ANCHOR ON LOW SPEED:
    //   If the robot slows below threshold, discard the GPS anchor accumulated
    //   during motion. We don't want the next calibration sample to span a
    //   stop-and-go event — the bearing between a pre-stop and post-stop fix
    //   is meaningless (it points from wherever the robot stopped to wherever
    //   it resumed, which has nothing to do with the robot's current heading).
    //--------------------------------------------------------------------------
    const double speed = effective_speed_m_s();

    const char * speed_source =
        (use_status_speed_ && status_speed_received_.load()) ? "status_speed" : "gps_speed";

    if (speed < min_calibration_speed_m_s_)
    {
        prev_gps_sample_.reset();

        RCLCPP_DEBUG(this->get_logger(),
            "[%s] Speed %.3f m/s < threshold %.3f m/s — resetting GPS anchor.",
            speed_source, speed, min_calibration_speed_m_s_);
        return;
    }

    //--------------------------------------------------------------------------
    // Gate 3: Low yaw rate (straight-ish movement)
    // WHY stricter than gps_speed's rotation gate: GPS bearing on a curved
    // arc doesn't match the robot's instantaneous heading, so we'd compute
    // a wrong calibration offset. Requiring low yaw rate ensures we only
    // calibrate during straight-line travel.
    //--------------------------------------------------------------------------
    const double yaw_rate = std::abs(latest_yaw_rate_rad_s_.load());

    if (yaw_rate > max_calibration_yaw_rate_rad_s_)
    {
        prev_gps_sample_.reset();

        RCLCPP_DEBUG(this->get_logger(),
            "Yaw rate %.3f rad/s > threshold %.3f rad/s — "
            "resetting GPS anchor (turning).",
            yaw_rate, max_calibration_yaw_rate_rad_s_);
        return;
    }

    //--------------------------------------------------------------------------
    // First valid sample: store anchor and wait for the next one
    //--------------------------------------------------------------------------
    const rclcpp::Time current_stamp = msg->header.stamp;

    if (!prev_gps_sample_.has_value())
    {
        prev_gps_sample_ = {msg->latitude, msg->longitude, current_stamp};
        RCLCPP_DEBUG(this->get_logger(),
            "GPS anchor set for heading computation [speed_source=%s, speed=%.3f m/s].",
            speed_source, speed);
        return;
    }

    //--------------------------------------------------------------------------
    // Time delta guard
    //--------------------------------------------------------------------------
    const double dt_s = (current_stamp - prev_gps_sample_->stamp).seconds();

    if (dt_s < gps_heading_min_time_delta_s_)
    {
        RCLCPP_DEBUG(this->get_logger(),
            "GPS heading time delta %.3fs < %.3fs — skipping.",
            dt_s, gps_heading_min_time_delta_s_);
        return;
    }

    //--------------------------------------------------------------------------
    // Distance check — bearing is unreliable for very small displacements
    //--------------------------------------------------------------------------
    const double distance_m = haversine_distance_m(
        prev_gps_sample_->lat_deg, prev_gps_sample_->lon_deg,
        msg->latitude, msg->longitude);

    if (distance_m < min_gps_heading_distance_m_)
    {
        RCLCPP_DEBUG(this->get_logger(),
            "GPS displacement %.4fm < %.4fm — not enough for reliable bearing.",
            distance_m, min_gps_heading_distance_m_);
        // Don't reset anchor — keep accumulating distance.
        return;
    }

    //--------------------------------------------------------------------------
    // Compute GPS bearing (ground-truth heading)
    //--------------------------------------------------------------------------
    const double gps_heading_deg = gps_bearing_deg(
        prev_gps_sample_->lat_deg, prev_gps_sample_->lon_deg,
        msg->latitude, msg->longitude);

    //--------------------------------------------------------------------------
    // Compute calibration offset
    //   offset = normalize(gps_heading - imu_heading) → [-180, 180]
    // WHY normalize to [-180, 180]: the offset is a signed correction, not
    // a heading. We want the shortest angular path between the two.
    //--------------------------------------------------------------------------
    const double imu_compass_deg = latest_imu_compass_deg_.load();
    const double new_offset  = normalize_180(gps_heading_deg - imu_compass_deg);

    //--------------------------------------------------------------------------
    // Update calibration offset via EMA smoothing.
    //
    // WHY EMA AND NOT DIRECT REPLACEMENT:
    //   A single GPS bearing sample can be corrupted by multipath, antenna
    //   vibration, or a momentary non-straight heading. EMA rejects outliers
    //   by weighting the new sample against the running estimate.
    //   alpha=1.0 → instant update (no smoothing).
    //   alpha=0.5 → equal weight to new sample and history (default).
    //   alpha=0.3 → strong smoothing; slow to converge, very noise-resistant.
    //
    // WHY NORMALIZE THE ANGULAR MEAN:
    //   The offset lives in [-180, 180]. A direct weighted sum of two angles
    //   near ±180° would wrap incorrectly. We compute the delta between old and
    //   new estimate and add a fraction of it — equivalent to EMA but correct
    //   across the ±180° boundary.
    //--------------------------------------------------------------------------
    const bool first_calibration = !calibration_offset_deg_.has_value();

    if (first_calibration)
    {
        // Bootstrap: no history to blend with, accept new_offset directly.
        calibration_offset_deg_ = new_offset;
    }
    else
    {
        const double prev_offset = calibration_offset_deg_.value();
        const double delta       = normalize_180(new_offset - prev_offset);
        calibration_offset_deg_  = normalize_180(
            prev_offset + calibration_ema_alpha_ * delta);
    }

    // Build a human-readable UTC timestamp for the status topic.
    // Include which speed source was active so the operator can verify.
    // Format: "Calibrated [status_speed] - Last update: 2024-03-15 14:23:07 UTC"
    {
        const std::time_t now_t = std::time(nullptr);
        char time_buf[80];
        std::snprintf(time_buf, sizeof(time_buf),
            "Calibrated [%s] - Last update: %%Y-%%m-%%d %%H:%%M:%%S UTC",
            speed_source);
        char final_buf[100];
        std::strftime(final_buf, sizeof(final_buf), time_buf, std::gmtime(&now_t));
        last_calibration_time_str_ = final_buf;
    }

    if (first_calibration)
    {
        RCLCPP_INFO(this->get_logger(),
            "CALIBRATED [%s]! Initial offset: %.2f° "
            "(GPS heading=%.2f°, IMU compass=%.2f°, distance=%.3fm, speed=%.3fm/s)",
            speed_source, new_offset,
            gps_heading_deg, imu_compass_deg, distance_m, speed);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(),
            "Calibration updated [%s]: offset=%.2f° "
            "(GPS=%.2f°, IMU=%.2f°, distance=%.3fm, speed=%.3fm/s)",
            speed_source, calibration_offset_deg_.value(),
            gps_heading_deg, imu_compass_deg, distance_m, speed);
    }

    // Advance GPS anchor — use current fix as the next starting point.
    prev_gps_sample_ = {msg->latitude, msg->longitude, current_stamp};
}

//==============================================================================
// MATH HELPERS
//==============================================================================

double ImuCompassNode::yaw_from_quaternion(double x, double y, double z, double w)
{
    // Standard ZYX Euler yaw extraction from a quaternion.
    // Result is in ENU convention (East-North-Up world frame):
    //   yaw_enu = 0°  → facing East, positive = CCW
    const double yaw_enu_rad = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));

    // Convert ENU yaw → compass/NED heading (0=North, 90=East, CW-positive):
    //   compass = 90° - yaw_enu_deg
    // Verification: East(0°ENU)→90°, North(90°ENU)→0°, West(180°ENU)→270° ✓
    return normalize_360(90.0 - rad2deg(yaw_enu_rad));
}

double ImuCompassNode::gps_bearing_deg(
    double lat1_deg, double lon1_deg,
    double lat2_deg, double lon2_deg)
{
    // Forward azimuth (bearing from point 1 to point 2) using spherical trig.
    const double lat1 = deg2rad(lat1_deg);
    const double lat2 = deg2rad(lat2_deg);
    const double dlon = deg2rad(lon2_deg - lon1_deg);

    const double y = std::sin(dlon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2)
                   - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    return normalize_360(rad2deg(std::atan2(y, x)));
}

double ImuCompassNode::haversine_distance_m(
    double lat1_deg, double lon1_deg,
    double lat2_deg, double lon2_deg)
{
    const double lat1 = deg2rad(lat1_deg);
    const double lat2 = deg2rad(lat2_deg);
    const double dlat = deg2rad(lat2_deg - lat1_deg);
    const double dlon = deg2rad(lon2_deg - lon1_deg);

    const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0)
                   + std::cos(lat1) * std::cos(lat2)
                   * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

    return EARTH_RADIUS_M * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

double ImuCompassNode::normalize_180(double deg)
{
    while (deg >  180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double ImuCompassNode::normalize_360(double deg)
{
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

//==============================================================================
// MAIN
//==============================================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<ImuCompassNode>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("imu_compass_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
