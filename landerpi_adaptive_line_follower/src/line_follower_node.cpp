// Copyright 2026 ldavalle
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "landerpi_adaptive_line_follower/line_follower_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "lifecycle_msgs/msg/state.hpp"

namespace landerpi_adaptive_line_follower
{

namespace
{

rcl_interfaces::msg::ParameterDescriptor floatRangeDescriptor(double from, double to)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

}  // namespace

LineFollowerNode::LineFollowerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("line_follower_node", options)
{
}

LineTrackerConfig LineFollowerNode::readLineTrackerConfig() const
{
  LineTrackerConfig config;
  config.l_tolerance = get_parameter("lab_l_tolerance").as_double();
  config.ab_tolerance = get_parameter("lab_ab_tolerance").as_double();
  config.min_contour_area = get_parameter("min_contour_area").as_double();
  config.near_roi = {
    get_parameter("near_roi_y_start").as_double(), get_parameter("near_roi_y_end").as_double()};
  config.mid_roi = {
    get_parameter("mid_roi_y_start").as_double(), get_parameter("mid_roi_y_end").as_double()};
  config.far_roi = {
    get_parameter("far_roi_y_start").as_double(), get_parameter("far_roi_y_end").as_double()};
  return config;
}

PidConfig LineFollowerNode::readAngularPidConfig() const
{
  PidConfig config;
  config.kp = get_parameter("angular_kp").as_double();
  config.ki = get_parameter("angular_ki").as_double();
  config.kd = get_parameter("angular_kd").as_double();
  const double output_max = get_parameter("angular_output_max").as_double();
  config.output_min = -output_max;
  config.output_max = output_max;
  const double integral_limit = get_parameter("angular_integral_limit").as_double();
  config.integral_min = -integral_limit;
  config.integral_max = integral_limit;
  return config;
}

ZoneConfig LineFollowerNode::readObstacleZoneConfig() const
{
  ZoneConfig config;
  config.zone_min_angle_deg = get_parameter("obstacle_min_angle_deg").as_double();
  config.zone_max_angle_deg = get_parameter("obstacle_max_angle_deg").as_double();
  config.zone_radius_m = get_parameter("obstacle_stop_distance_m").as_double();
  config.noise_filter_window =
    static_cast<std::size_t>(get_parameter("obstacle_noise_filter_window").as_int());
  return config;
}

LineFollowerNode::CallbackReturn LineFollowerNode::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter("image_topic", std::string("/ascamera/camera_publisher/rgb0/image"));
  declare_parameter("cmd_vel_topic", std::string("/controller/cmd_vel"));
  declare_parameter("linear_speed_max", 0.15, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("linear_speed_min", 0.10, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("curvature_speed_gain", 0.0, floatRangeDescriptor(0.0, 10.0));
  declare_parameter("angular_kp", 0.005, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("angular_ki", 0.0, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("angular_kd", 0.001, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("angular_output_max", 1.0, floatRangeDescriptor(0.0, 2.0));
  declare_parameter("angular_integral_limit", 50.0, floatRangeDescriptor(0.0, 500.0));
  declare_parameter("color_sample_patch_px", 20);
  declare_parameter("lab_l_tolerance", 40.0, floatRangeDescriptor(1.0, 128.0));
  declare_parameter("lab_ab_tolerance", 20.0, floatRangeDescriptor(1.0, 128.0));
  declare_parameter("min_contour_area", 30.0, floatRangeDescriptor(1.0, 10000.0));
  declare_parameter("near_roi_y_start", 0.85, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("near_roi_y_end", 0.95, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("mid_roi_y_start", 0.55, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("mid_roi_y_end", 0.65, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("far_roi_y_start", 0.25, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("far_roi_y_end", 0.35, floatRangeDescriptor(0.0, 1.0));

  declare_parameter("search_grace_period_s", 0.5, floatRangeDescriptor(0.0, 5.0));
  declare_parameter("search_timeout_s", 25.0, floatRangeDescriptor(1.0, 120.0));
  declare_parameter("search_settle_time_s", 0.3, floatRangeDescriptor(0.05, 5.0));
  // Safety cap only — the real stop condition for the return turn is the
  // IMU-integrated yaw reaching search_found_bearing_deg_ (see
  // imuCallback/kReturnCenter), not this duration.
  declare_parameter("search_return_max_s", 4.0, floatRangeDescriptor(0.5, 15.0));
  declare_parameter("imu_topic", std::string("/ros_robot_controller/imu_raw"));
  // Joint1's real physical limit is +-120.2 deg (see the driver's
  // transform.py, same figure B1's arm_joint1_limit_deg uses) — 120.0
  // stays a hair inside that.
  declare_parameter("search_arm_max_bearing_deg", 120.0, floatRangeDescriptor(1.0, 120.0));
  declare_parameter("search_arm_step_deg", 10.0, floatRangeDescriptor(1.0, 30.0));
  // Which sign of joint1 pulse delta corresponds to a positive (right)
  // bearing depends on how the servo is mounted — flip this if the arm
  // pans the wrong way. Confirm empirically, same as any other unverified
  // sign convention on this robot (see CLAUDE.md/README wheel wiring
  // saga for why we don't just assume).
  declare_parameter("search_arm_joint1_pulse_sign", 1.0);
  declare_parameter("search_chassis_turn_speed", 0.3, floatRangeDescriptor(0.0, 1.0));
  declare_parameter("servo_controller_topic", std::string("servo_controller"));
  // Found once with tools/arm_teleop.py: joints 1-4 (base, shoulder,
  // elbow, wrist-pitch) pulses for a pose where the camera sees the line
  // well. This is the pose SEARCHING always starts from and returns to.
  declare_parameter(
    "line_follow_arm_pulses", std::vector<int64_t>{500, 500, 500, 500});

  declare_parameter("scan_topic", std::string("/scan_raw"));
  declare_parameter("obstacle_min_angle_deg", -20.0, floatRangeDescriptor(-180.0, 180.0));
  declare_parameter("obstacle_max_angle_deg", 20.0, floatRangeDescriptor(-180.0, 180.0));
  declare_parameter("obstacle_stop_distance_m", 0.20, floatRangeDescriptor(0.05, 2.0));
  declare_parameter("obstacle_noise_filter_window", 3);

  declare_parameter("odom_topic", std::string("/odom_raw"));
  declare_parameter("lap_loop_radius_m", 0.3, floatRangeDescriptor(0.05, 5.0));

  obstacle_monitor_ = std::make_unique<ZoneMonitor>(readObstacleZoneConfig());

  line_tracker_ = std::make_unique<LineTracker>(readLineTrackerConfig());
  angular_pid_ = std::make_unique<Pid>(readAngularPidConfig());

  linear_speed_max_ = get_parameter("linear_speed_max").as_double();
  linear_speed_min_ = get_parameter("linear_speed_min").as_double();
  curvature_speed_gain_ = get_parameter("curvature_speed_gain").as_double();
  color_sample_patch_px_ = static_cast<int>(get_parameter("color_sample_patch_px").as_int());
  base_l_tolerance_ = get_parameter("lab_l_tolerance").as_double();
  base_ab_tolerance_ = get_parameter("lab_ab_tolerance").as_double();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    get_parameter("cmd_vel_topic").as_string(), 1);
  servo_pub_ = create_publisher<servo_controller_msgs::msg::ServosPosition>(
    get_parameter("servo_controller_topic").as_string(), 10);

  init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/init_finish",
    [](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
    });
  set_running_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_running",
    std::bind(
      &LineFollowerNode::setRunningCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_target_color_srv_ = create_service<interfaces::srv::SetPoint>(
    "~/set_target_color",
    std::bind(
      &LineFollowerNode::setTargetColorCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  get_target_color_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/get_target_color",
    std::bind(
      &LineFollowerNode::getTargetColorCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  set_threshold_srv_ = create_service<interfaces::srv::SetFloat64>(
    "~/set_threshold",
    std::bind(
      &LineFollowerNode::setThresholdCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(
      &LineFollowerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  get_lap_stats_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/get_lap_stats",
    std::bind(
      &LineFollowerNode::getLapStatsCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "configured");
  return CallbackReturn::SUCCESS;
}

LineFollowerNode::CallbackReturn LineFollowerNode::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_pub_->on_activate();
  servo_pub_->on_activate();

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    get_parameter("image_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&LineFollowerNode::imageCallback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    get_parameter("imu_topic").as_string(), 10,
    std::bind(&LineFollowerNode::imuCallback, this, std::placeholders::_1));

  // Same QoS the stock lidar_app/line_following nodes use for /scan_raw.
  rclcpp::QoS scan_qos(rclcpp::KeepLast(5));
  scan_qos.best_effort();
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    get_parameter("scan_topic").as_string(), scan_qos,
    std::bind(&LineFollowerNode::scanCallback, this, std::placeholders::_1));
  obstacle_monitor_->reset();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(), 10,
    std::bind(&LineFollowerNode::odomCallback, this, std::placeholders::_1));

  // Ticks continuously once activated, but only acts while state_ ==
  // kSearching (see searchTimerCallback) — simplest way to drive a
  // multi-step arm/chassis sequence without chaining async service
  // callbacks by hand.
  search_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LineFollowerNode::searchTimerCallback, this));

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Activating the lifecycle only makes the node ready — it does not
    // start moving the robot. FOLLOWING requires an explicit
    // ~/set_running(true) once a target color is calibrated.
    state_ = BehaviorState::kIdle;
    has_last_callback_time_ = false;
    line_lost_since_valid_ = false;
  }

  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}

LineFollowerNode::CallbackReturn LineFollowerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  image_sub_.reset();
  imu_sub_.reset();
  scan_sub_.reset();
  odom_sub_.reset();
  if (search_timer_) {
    search_timer_->cancel();
  }

  // Stop the robot before the publisher goes inactive: the stock's low-level
  // driver has no watchdog of its own, so otherwise it keeps executing the
  // last velocity command it received.
  publishStop();
  cmd_vel_pub_->on_deactivate();
  servo_pub_->on_deactivate();

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = BehaviorState::kIdle;
  lap_tracking_active_ = false;

  RCLCPP_INFO(get_logger(), "deactivated");
  return CallbackReturn::SUCCESS;
}

LineFollowerNode::CallbackReturn LineFollowerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  image_sub_.reset();
  imu_sub_.reset();
  scan_sub_.reset();
  odom_sub_.reset();
  search_timer_.reset();
  cmd_vel_pub_.reset();
  servo_pub_.reset();
  init_finish_srv_.reset();
  set_running_srv_.reset();
  set_target_color_srv_.reset();
  get_target_color_srv_.reset();
  set_threshold_srv_.reset();
  reset_srv_.reset();
  get_lap_stats_srv_.reset();
  line_tracker_.reset();
  angular_pid_.reset();
  obstacle_monitor_.reset();

  std::lock_guard<std::mutex> lock(state_mutex_);
  target_color_calibrated_ = false;
  last_frame_ = cv::Mat();
  state_ = BehaviorState::kIdle;
  has_search_original_pulses_ = false;

  return CallbackReturn::SUCCESS;
}

LineFollowerNode::CallbackReturn LineFollowerNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    publishStop();
  }
  return CallbackReturn::SUCCESS;
}

void LineFollowerNode::publishStop()
{
  if (!cmd_vel_pub_) {
    return;
  }
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void LineFollowerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  last_frame_ = cv_ptr->image.clone();

  if (state_ != BehaviorState::kFollowing) {
    // kSearching/kStopped/kIdle: searchTimerCallback (or set_running/reset)
    // owns any cmd_vel/arm decisions for those states — this callback's job
    // is only to keep last_frame_ fresh for them.
    return;
  }

  const LineTrackerResult result = line_tracker_->Track(last_frame_);

  const rclcpp::Time now = get_clock()->now();
  double dt_s = 0.0;
  if (has_last_callback_time_) {
    dt_s = (now - last_callback_time_).seconds();
  }
  last_callback_time_ = now;
  has_last_callback_time_ = true;

  if (!result.line_found) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500, "line_found=0 near=%d mid=%d far=%d",
      result.near_band.found, result.mid_band.found, result.far_band.found);
    if (!line_lost_since_valid_) {
      line_lost_since_ = now;
      line_lost_since_valid_ = true;
    }
    const double grace_s = get_parameter("search_grace_period_s").as_double();
    if ((now - line_lost_since_).seconds() >= grace_s) {
      enterSearchingLocked();
    } else {
      // Still within the grace period: hold last commanded motion off
      // rather than guessing, same conservative fallback as before.
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
    return;
  }
  line_lost_since_valid_ = false;

  const double angular_output = angular_pid_->Update(result.offset_px, dt_s);

  // Curvature is only trustworthy when all three ROI bands saw the line
  // (Track() leaves it at 0.0 both for a real straight and for "not enough
  // data" — those two cases can't be told apart from curvature_rad alone).
  // In the "not enough data" case, use a conservative fraction of vmax
  // rather than assuming a straight.
  double linear_speed = linear_speed_max_ * 0.75;
  if (result.near_band.found && result.mid_band.found && result.far_band.found) {
    linear_speed = linear_speed_max_ - curvature_speed_gain_ * std::fabs(result.curvature_rad);
    linear_speed = std::clamp(linear_speed, linear_speed_min_, linear_speed_max_);
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_speed;
  // offset_px > 0 means the line is to the right of center, which should
  // turn the robot right (negative angular.z in ROS's convention).
  twist.angular.z = -angular_output;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 300,
    "offset_px=%.1f (near=%.1f/%d mid=%.1f/%d far=%.1f/%d) angular_z=%.3f linear_x=%.3f "
    "curvature_rad=%.3f",
    result.offset_px,
    result.near_band.x_px, result.near_band.found,
    result.mid_band.x_px, result.mid_band.found,
    result.far_band.x_px, result.far_band.found,
    twist.angular.z, twist.linear.x, result.curvature_rad);
  cmd_vel_pub_->publish(twist);
}

void LineFollowerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!search_return_integrating_) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp);
  if (search_return_has_last_imu_stamp_) {
    const double dt_s = (stamp - search_return_last_imu_stamp_).seconds();
    if (dt_s > 0.0) {
      search_return_integrated_yaw_rad_ += msg->angular_velocity.z * dt_s;
    }
  }
  search_return_last_imu_stamp_ = stamp;
  search_return_has_last_imu_stamp_ = true;
}

void LineFollowerNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const GuardResult result = obstacle_monitor_->update(
    msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);

  if (result.intrusion_detected) {
    if (state_ == BehaviorState::kFollowing || state_ == BehaviorState::kSearching) {
      pre_obstacle_state_ = state_;
      state_ = BehaviorState::kStoppedObstacle;
      RCLCPP_WARN(
        get_logger(), "obstacle at %.2fm (angle %.1f deg) -> STOPPED_OBSTACLE",
        result.distance_m, result.angle_deg);
    }
    if (state_ == BehaviorState::kStoppedObstacle) {
      // Overrides whatever imageCallback/searchTimerCallback would have
      // published — both already no-op while state_ != their own state,
      // so this is the only thing driving cmd_vel here.
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
    return;
  }

  if (state_ == BehaviorState::kStoppedObstacle) {
    // Transient by nature (unlike STOPPED from a failed search): resumes
    // on its own, no ~/reset needed.
    state_ = pre_obstacle_state_;
    if (state_ == BehaviorState::kFollowing) {
      angular_pid_->Reset();
      has_last_callback_time_ = false;
      line_lost_since_valid_ = false;
    }
    RCLCPP_INFO(get_logger(), "obstacle cleared -> resuming");
  }
}

void LineFollowerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!lap_tracking_active_) {
    return;
  }

  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  if (lap_awaiting_start_odom_) {
    lap_start_x_ = x;
    lap_start_y_ = y;
    lap_start_time_ = get_clock()->now();
    lap_awaiting_start_odom_ = false;
    lap_last_x_ = x;
    lap_last_y_ = y;
    lap_has_last_odom_ = true;
    return;
  }

  if (lap_has_last_odom_) {
    lap_distance_traveled_m_ += std::hypot(x - lap_last_x_, y - lap_last_y_);
  }
  lap_last_x_ = x;
  lap_last_y_ = y;
  lap_has_last_odom_ = true;

  const double radius = get_parameter("lap_loop_radius_m").as_double();
  const double dist_from_start = std::hypot(x - lap_start_x_, y - lap_start_y_);

  if (!lap_has_left_start_) {
    if (dist_from_start > radius) {
      lap_has_left_start_ = true;
    }
    return;
  }

  if (dist_from_start <= radius) {
    const double elapsed_s = (get_clock()->now() - lap_start_time_).seconds();
    last_lap_time_s_ = elapsed_s;
    last_lap_distance_m_ = lap_distance_traveled_m_;
    last_lap_avg_speed_mps_ = elapsed_s > 0.0 ? lap_distance_traveled_m_ / elapsed_s : 0.0;
    has_last_lap_ = true;
    RCLCPP_INFO(
      get_logger(), "lap complete: time=%.2fs distance=%.2fm avg_speed=%.3fm/s",
      elapsed_s, lap_distance_traveled_m_, last_lap_avg_speed_mps_);

    // Immediately start timing the next lap from here, in case the robot
    // keeps going around the circuit.
    lap_start_x_ = x;
    lap_start_y_ = y;
    lap_start_time_ = get_clock()->now();
    lap_distance_traveled_m_ = 0.0;
    lap_has_left_start_ = false;
  }
}

void LineFollowerNode::getLapStatsCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_last_lap_) {
    response->success = false;
    response->message = "no completed lap yet";
    return;
  }
  response->success = true;
  std::ostringstream oss;
  oss << "time_s=" << last_lap_time_s_
      << " distance_m=" << last_lap_distance_m_
      << " avg_speed_mps=" << last_lap_avg_speed_mps_;
  response->message = oss.str();
}

void LineFollowerNode::loadConfiguredArmPoseLocked()
{
  // The line-following pose is a fixed, hand-tuned configuration (find it
  // once with tools/arm_teleop.py, then set line_follow_arm_pulses) rather
  // than "whatever the arm happened to be at" — that used to be fragile:
  // if SEARCHING re-triggered before a previous move had settled, or the
  // arm was left in a bad pose from an earlier bug, it would get captured
  // and locked in as the reference. A fixed pose sidesteps that class of
  // bug entirely and means ~/set_running(true) always starts from the
  // same known-good position.
  const auto configured = get_parameter("line_follow_arm_pulses").as_integer_array();
  for (size_t i = 0; i < 4; ++i) {
    search_original_pulses_[i] =
      static_cast<int32_t>(i < configured.size() ? configured[i] : 500);
  }
  has_search_original_pulses_ = true;
  restoreOriginalArmPulsesLocked();
}

void LineFollowerNode::panArmToBearingDeltaLocked(double bearing_delta_deg)
{
  if (!has_search_original_pulses_) {
    return;
  }
  // HX-06L bus servos: 0-1000 pulse maps to 0-240 degrees (confirmed in
  // PORTFOLIO_PROJECTS.md against the arm's real spec sheet), so this
  // ratio holds for any joint on this arm, joint1 included.
  constexpr double kPulsesPerDegree = 1000.0 / 240.0;
  const double sign = get_parameter("search_arm_joint1_pulse_sign").as_double();
  const int32_t delta =
    static_cast<int32_t>(std::lround(sign * bearing_delta_deg * kPulsesPerDegree));

  std::vector<uint16_t> pulses(4);
  for (size_t i = 0; i < 4; ++i) {
    pulses[i] = static_cast<uint16_t>(std::clamp<int32_t>(search_original_pulses_[i], 0, 1000));
  }
  pulses[0] = static_cast<uint16_t>(
    std::clamp<int32_t>(search_original_pulses_[0] + delta, 0, 1000));
  publishArmPulses(pulses);
}

void LineFollowerNode::restoreOriginalArmPulsesLocked()
{
  if (!has_search_original_pulses_) {
    return;
  }
  std::vector<uint16_t> pulses(4);
  for (size_t i = 0; i < 4; ++i) {
    pulses[i] = static_cast<uint16_t>(std::clamp<int32_t>(search_original_pulses_[i], 0, 1000));
  }
  publishArmPulses(pulses);
}

void LineFollowerNode::publishArmPulses(const std::vector<uint16_t> & pulse)
{
  if (pulse.size() < 4) {
    RCLCPP_WARN(get_logger(), "asked to publish %zu arm pulses, expected at least 4", pulse.size());
    return;
  }
  if (!servo_pub_->is_activated()) {
    RCLCPP_WARN(get_logger(), "servo publisher not activated, dropping arm pulses");
    return;
  }

  servo_controller_msgs::msg::ServosPosition msg;
  msg.duration = 0.4;
  // See servos-position-unit-gotcha: controller_manager silently drops the
  // whole message if position_unit is left unset, no error anywhere.
  msg.position_unit = "pulse";
  const auto addServo = [&msg](uint16_t id, uint16_t position) {
      servo_controller_msgs::msg::ServoPosition servo;
      servo.id = id;
      servo.position = static_cast<float>(std::clamp<uint16_t>(position, 0, 1000));
      msg.position.push_back(servo);
    };
  addServo(1, pulse[0]);
  addServo(2, pulse[1]);
  addServo(3, pulse[2]);
  addServo(4, pulse[3]);
  addServo(5, 500);
  addServo(10, 500);
  servo_pub_->publish(msg);
}

void LineFollowerNode::enterSearchingLocked()
{
  state_ = BehaviorState::kSearching;
  search_phase_ = SearchPhase::kPanRight;
  search_phase_action_sent_ = false;
  search_current_bearing_deg_ = 0.0;
  search_found_side_ = 0;
  search_deadline_ = get_clock()->now() +
    rclcpp::Duration::from_seconds(get_parameter("search_timeout_s").as_double());
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  // Arm's reference pose is already loaded (loadConfiguredArmPoseLocked
  // ran when ~/set_running(true) was called) — nothing to capture here.
  RCLCPP_INFO(get_logger(), "line lost -> SEARCHING (peeking with the arm)");
}

void LineFollowerNode::enterStoppedLocked()
{
  state_ = BehaviorState::kStopped;
  search_return_integrating_ = false;
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  restoreOriginalArmPulsesLocked();  // put the arm back exactly how it was
  RCLCPP_WARN(get_logger(), "SEARCHING gave up -> STOPPED, call ~/reset to retry");
}

bool LineFollowerNode::checkLineInLastFrameLocked()
{
  if (last_frame_.empty()) {
    return false;
  }
  return line_tracker_->Track(last_frame_).line_found;
}

void LineFollowerNode::searchTimerCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != BehaviorState::kSearching) {
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  if (now >= search_deadline_ && search_phase_ != SearchPhase::kReturnCenter) {
    enterStoppedLocked();
    return;
  }

  const double max_bearing_deg = get_parameter("search_arm_max_bearing_deg").as_double();
  const double step_deg = get_parameter("search_arm_step_deg").as_double();
  const double settle_s = get_parameter("search_settle_time_s").as_double();

  switch (search_phase_) {
    case SearchPhase::kPanRight:
      if (!search_phase_action_sent_) {
        search_current_bearing_deg_ =
          std::min(search_current_bearing_deg_ + step_deg, max_bearing_deg);
        panArmToBearingDeltaLocked(search_current_bearing_deg_);
        search_phase_action_sent_ = true;
        search_phase_deadline_ = now + rclcpp::Duration::from_seconds(settle_s);
        return;
      }
      if (now < search_phase_deadline_) {
        return;
      }
      if (checkLineInLastFrameLocked()) {
        search_found_side_ = 1;
        search_found_bearing_deg_ = search_current_bearing_deg_;
        search_phase_ = SearchPhase::kReturnCenter;
        search_phase_action_sent_ = false;
      } else if (search_current_bearing_deg_ >= max_bearing_deg) {
        search_phase_ = SearchPhase::kPanLeft;
        search_current_bearing_deg_ = 0.0;
        search_phase_action_sent_ = false;
      } else {
        search_phase_action_sent_ = false;  // one more step further right
      }
      break;

    case SearchPhase::kPanLeft:
      if (!search_phase_action_sent_) {
        search_current_bearing_deg_ =
          std::max(search_current_bearing_deg_ - step_deg, -max_bearing_deg);
        panArmToBearingDeltaLocked(search_current_bearing_deg_);
        search_phase_action_sent_ = true;
        search_phase_deadline_ = now + rclcpp::Duration::from_seconds(settle_s);
        return;
      }
      if (now < search_phase_deadline_) {
        return;
      }
      if (checkLineInLastFrameLocked()) {
        search_found_side_ = -1;
        search_found_bearing_deg_ = search_current_bearing_deg_;
        search_phase_ = SearchPhase::kReturnCenter;
        search_phase_action_sent_ = false;
      } else if (search_current_bearing_deg_ <= -max_bearing_deg) {
        search_phase_ = SearchPhase::kBlindSweep;
        search_phase_action_sent_ = false;
      } else {
        search_phase_action_sent_ = false;  // one more step further left
      }
      break;

    case SearchPhase::kReturnCenter:
      {
        if (!search_phase_action_sent_) {
          restoreOriginalArmPulsesLocked();
          search_phase_action_sent_ = true;
          search_return_integrating_ = true;
          search_return_integrated_yaw_rad_ = 0.0;
          search_return_has_last_imu_stamp_ = false;
          const double max_s = get_parameter("search_return_max_s").as_double();
          search_phase_deadline_ = now + rclcpp::Duration::from_seconds(max_s);  // safety cap
          RCLCPP_INFO(
            get_logger(),
            "search: line found at %.1f deg, turning chassis that far (IMU-measured) "
            "while arm re-centers",
            search_found_bearing_deg_);
        }

        const double target_yaw_rad = std::abs(search_found_bearing_deg_) * M_PI / 180.0;
        const bool turned_enough = std::abs(search_return_integrated_yaw_rad_) >= target_yaw_rad;
        if (turned_enough || now >= search_phase_deadline_) {
          if (!turned_enough) {
            RCLCPP_WARN(
              get_logger(),
              "search: return turn hit its safety timeout before the IMU confirmed %.1f deg",
              search_found_bearing_deg_);
          }
          search_return_integrating_ = false;
          cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
          state_ = BehaviorState::kFollowing;
          angular_pid_->Reset();
          has_last_callback_time_ = false;
          line_lost_since_valid_ = false;
          return;
        }
        // Directed turn toward the side the line was actually seen on — not
        // a blind sweep, we already confirmed it's there. Kept running
        // (not stepped) until the IMU says we've turned far enough.
        geometry_msgs::msg::Twist twist;
        const double turn_speed = get_parameter("search_chassis_turn_speed").as_double();
        twist.angular.z = search_found_side_ > 0 ? -turn_speed : turn_speed;
        cmd_vel_pub_->publish(twist);
        break;
      }

    case SearchPhase::kBlindSweep:
      // Neither side found anything with the arm. Blindly sweeping the
      // whole chassis is deferred (see CLAUDE.md, "a definir el detalle
      // cuando se implemente") — for now this is the same safe fallback
      // as running out of the overall SEARCHING timeout.
      enterStoppedLocked();
      break;
  }
}

void LineFollowerNode::resetCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != BehaviorState::kStopped) {
    response->success = false;
    response->message = "not in STOPPED state";
    return;
  }
  state_ = BehaviorState::kFollowing;
  angular_pid_->Reset();
  has_last_callback_time_ = false;
  line_lost_since_valid_ = false;
  response->success = true;
  response->message = "retrying from FOLLOWING";
}

void LineFollowerNode::setRunningCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (request->data) {
    if (!target_color_calibrated_) {
      response->success = false;
      response->message = "cannot start following: no target color calibrated yet";
      return;
    }
    state_ = BehaviorState::kFollowing;
    angular_pid_->Reset();
    has_last_callback_time_ = false;
    // Actively move the arm to the configured line-following pose every
    // time a run starts, rather than trusting it's already there.
    loadConfiguredArmPoseLocked();
    // Lap tracker: start capturing from the next odom message (see
    // odomCallback), not from here — we don't have a synchronous odom
    // reading at this point.
    lap_tracking_active_ = true;
    lap_awaiting_start_odom_ = true;
    lap_has_left_start_ = false;
    lap_has_last_odom_ = false;
    lap_distance_traveled_m_ = 0.0;
    response->success = true;
    response->message = "following";
  } else {
    state_ = BehaviorState::kIdle;
    lap_tracking_active_ = false;
    publishStop();
    response->success = true;
    response->message = "idle";
  }
}

void LineFollowerNode::setTargetColorCallback(
  const std::shared_ptr<interfaces::srv::SetPoint::Request> request,
  std::shared_ptr<interfaces::srv::SetPoint::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (last_frame_.empty()) {
    response->success = false;
    response->message = "no camera frame received yet";
    return;
  }

  if (state_ == BehaviorState::kFollowing) {
    // Recalibrating only happens from IDLE: stop the robot and invalidate
    // the old calibration before sampling the new one.
    state_ = BehaviorState::kIdle;
    publishStop();
  }

  const int w = last_frame_.cols;
  const int h = last_frame_.rows;
  const int half = std::max(1, color_sample_patch_px_ / 2);
  const int cx = static_cast<int>(request->data.x * w);
  const int cy = static_cast<int>(request->data.y * h);
  const int x0 = std::clamp(cx - half, 0, w - 1);
  const int y0 = std::clamp(cy - half, 0, h - 1);
  const int x1 = std::clamp(cx + half, x0 + 1, w);
  const int y1 = std::clamp(cy + half, y0 + 1, h);

  const cv::Mat patch_bgr = last_frame_(cv::Range(y0, y1), cv::Range(x0, x1));
  const cv::Scalar mean_bgr = cv::mean(patch_bgr);
  target_bgr_mean_ = cv::Vec3d(mean_bgr[0], mean_bgr[1], mean_bgr[2]);

  cv::Mat patch_lab;
  cv::cvtColor(patch_bgr, patch_lab, cv::COLOR_BGR2Lab);
  const cv::Scalar mean_lab = cv::mean(patch_lab);
  line_tracker_->SetTargetColor(cv::Vec3d(mean_lab[0], mean_lab[1], mean_lab[2]));
  target_color_calibrated_ = true;

  response->success = true;
  response->message = "target color calibrated";
}

void LineFollowerNode::getTargetColorCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!target_color_calibrated_) {
    response->success = false;
    response->message = "no target color calibrated yet";
    return;
  }

  std::ostringstream oss;
  oss << std::lround(target_bgr_mean_[2]) << "," << std::lround(target_bgr_mean_[1]) << ","
      << std::lround(target_bgr_mean_[0]);  // R,G,B, same order the stock reports
  response->success = true;
  response->message = oss.str();
}

void LineFollowerNode::setThresholdCallback(
  const std::shared_ptr<interfaces::srv::SetFloat64::Request> request,
  std::shared_ptr<interfaces::srv::SetFloat64::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Unlike the stock's single 0-1 threshold, here `data` is a plain
  // multiplier on the two independently-configured base tolerances
  // (lab_l_tolerance/lab_ab_tolerance params); 1.0 reproduces the
  // configured defaults.
  const double factor = request->data;
  line_tracker_->SetTolerance(base_l_tolerance_ * factor, base_ab_tolerance_ * factor);
  response->success = true;
  response->message = "threshold updated";
}

}  // namespace landerpi_adaptive_line_follower
