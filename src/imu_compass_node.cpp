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

    this->declare_parameter<double>("min_calibration_speed_m_s",       1.0);
    this->declare_parameter<double>("max_calibration_yaw_rate_rad_s",  0.1);
    this->declare_parameter<double>("min_gps_heading_distance_m",      0.5);
    this->declare_parameter<double>("gps_heading_min_time_delta_s",    0.1);

    min_calibration_speed_m_s_      = this->get_parameter("min_calibration_speed_m_s").as_double();
    max_calibration_yaw_rate_rad_s_ = this->get_parameter("max_calibration_yaw_rate_rad_s").as_double();
    min_gps_heading_distance_m_     = this->get_parameter("min_gps_heading_distance_m").as_double();
    gps_heading_min_time_delta_s_   = this->get_parameter("gps_heading_min_time_delta_s").as_double();

    RCLCPP_INFO(this->get_logger(),
        "Parameters: robot_name=%s  min_cal_speed=%.2fm/s  "
        "max_cal_yaw_rate=%.3frad/s (%.1fdeg/s)  "
        "min_gps_distance=%.2fm  min_gps_time_delta=%.3fs",
        robot_name_.c_str(),
        min_calibration_speed_m_s_,
        max_calibration_yaw_rate_rad_s_,
        rad2deg(max_calibration_yaw_rate_rad_s_),
        min_gps_heading_distance_m_,
        gps_heading_min_time_delta_s_);

    //--------------------------------------------------------------------------
    // Build topic names
    //--------------------------------------------------------------------------
    const std::string imu_topic     = "/" + robot_name_ + "/sensors/microstrain/ekf/imu/data";
    const std::string gps_topic     = "/" + robot_name_ + "/sensors/geofog/gps/fix";
    const std::string speed_topic   = "/" + robot_name_ + "/gps_speed";
    const std::string compass_topic = "/" + robot_name_ + "/compass";
    const std::string status_topic  = "/" + robot_name_ + "/compass/status";

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

    // gps_speed: ~9 Hz. Pre-computed by gps_speed_node.
    speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        speed_topic,
        rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            this->speed_callback(msg);
        });

    //--------------------------------------------------------------------------
    // Publishers
    //--------------------------------------------------------------------------
    compass_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        compass_topic, rclcpp::QoS(10).reliable());

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        status_topic, rclcpp::QoS(10).reliable());

    RCLCPP_INFO(this->get_logger(), "Subscribed to IMU:      %s", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to GPS:      %s", gps_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to speed:    %s", speed_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing compass:     %s", compass_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing status:      %s", status_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
        "Status: UNCALIBRATED — move at >= %.1f m/s to calibrate.",
        min_calibration_speed_m_s_);
}

//==============================================================================
// SPEED CALLBACK — just cache it
//==============================================================================

void ImuCompassNode::speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    latest_speed_m_s_.store(msg->data);
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
    //                   Uncalibrated: "UNCALIBRATED - Begin moving to calibrate"
    //                   Calibrated:   "Calibrated - Last update: 2024-03-15 14:23:07 UTC"
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
    // Gate 2: Speed sufficient for reliable GPS heading
    //--------------------------------------------------------------------------
    const double speed = latest_speed_m_s_.load();

    if (speed < min_calibration_speed_m_s_)
    {
        // Not moving fast enough — reset GPS anchor so we start fresh when
        // the robot picks up speed, rather than using a stale pre-stop position.
        prev_gps_sample_.reset();

        RCLCPP_DEBUG(this->get_logger(),
            "Speed %.2f m/s < threshold %.2f m/s — resetting GPS anchor.",
            speed, min_calibration_speed_m_s_);
        return;
    }

    //--------------------------------------------------------------------------
    // Gate 3: Low yaw rate (straight-ish movement)
    // WHY stricter than the gps_speed rotation gate: GPS bearing on a curved
    // arc doesn't match the robot's instantaneous heading, so we'd compute
    // a wrong calibration offset.
    //--------------------------------------------------------------------------
    const double yaw_rate = std::abs(latest_yaw_rate_rad_s_.load());

    if (yaw_rate > max_calibration_yaw_rate_rad_s_)
    {
        // Robot is turning — reset GPS anchor so bearing is computed between
        // two points both taken during straight-line travel.
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
        RCLCPP_DEBUG(this->get_logger(), "GPS anchor set for heading computation.");
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
        // Don't reset anchor — keep accumulating distance
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

    const bool first_calibration = !calibration_offset_deg_.has_value();
    calibration_offset_deg_ = new_offset;

    // Build a human-readable UTC timestamp for the status topic.
    // Format: "Calibrated - Last update: 2024-03-15 14:23:07 UTC"
    {
        const std::time_t now_t = std::time(nullptr);
        char time_buf[64];
        std::strftime(time_buf, sizeof(time_buf),
            "Calibrated - Last update: %Y-%m-%d %H:%M:%S UTC",
            std::gmtime(&now_t));
        last_calibration_time_str_ = time_buf;
    }

    if (first_calibration)
    {
        RCLCPP_INFO(this->get_logger(),
            "CALIBRATED! Initial offset: %.2f° "
            "(GPS heading=%.2f°, IMU compass=%.2f°, distance=%.3fm)",
            new_offset, gps_heading_deg, imu_compass_deg, distance_m);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(),
            "Calibration updated: offset=%.2f° "
            "(GPS=%.2f°, IMU=%.2f°, distance=%.3fm, speed=%.2fm/s)",
            new_offset, gps_heading_deg, imu_compass_deg, distance_m, speed);
    }

    //--------------------------------------------------------------------------
    // Advance GPS anchor — use current fix as the next starting point
    //--------------------------------------------------------------------------
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

    const double bearing_rad = std::atan2(y, x);

    return normalize_360(rad2deg(bearing_rad));
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
