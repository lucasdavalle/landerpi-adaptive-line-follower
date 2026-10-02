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

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/set_float64.hpp"
#include "interfaces/srv/set_point.hpp"
#include "landerpi_adaptive_line_follower/line_tracker.hpp"
#include "landerpi_adaptive_line_follower/pid.hpp"
#include "landerpi_adaptive_line_follower/zone_monitor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "servo_controller_msgs/msg/servos_position.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace landerpi_adaptive_line_follower
{

// Behavior state machine (independent of the rclcpp_lifecycle state — see
// the project's CLAUDE.md, section "Ciclo de vida"). kStoppedObstacle
// (lidar safety cutoff) is added in a later stage.
enum class BehaviorState
{
  kIdle,
  kFollowing,
  kSearching,
  kStopped,
  kStoppedObstacle
};

// Sub-states of kSearching — see CLAUDE.md, "Decisión 2026-09-18": peek
// left/right with the arm-mounted camera before ever moving the chassis
// blindly. Driven by searchTimerCallback().
enum class SearchPhase
{
  kPanRight,
  kPanLeft,
  kReturnCenter,
  kBlindSweep
};

class LineFollowerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LineFollowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishStop();
  void setRunningCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setTargetColorCallback(
    const std::shared_ptr<interfaces::srv::SetPoint::Request> request,
    std::shared_ptr<interfaces::srv::SetPoint::Response> response);
  void getTargetColorCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void setThresholdCallback(
    const std::shared_ptr<interfaces::srv::SetFloat64::Request> request,
    std::shared_ptr<interfaces::srv::SetFloat64::Response> response);
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  LineTrackerConfig readLineTrackerConfig() const;
  PidConfig readAngularPidConfig() const;
  ZoneConfig readObstacleZoneConfig() const;

  // Arm control for SEARCHING: rotate joint1 (the base) directly from a
  // fixed, hand-tuned reference pose (line_follow_arm_pulses param — find
  // it once with tools/arm_teleop.py), instead of going through inverse
  // kinematics. IK solves a *shape* for the requested XYZ and can move
  // every joint to get there — the wrong tool when the only thing that
  // should move is the base yaw. See CLAUDE.md, "Decisión 2026-09-18
  // (revisada)".
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void getLapStatsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void loadConfiguredArmPoseLocked();
  void panArmToBearingDeltaLocked(double bearing_delta_deg);
  void restoreOriginalArmPulsesLocked();
  void publishArmPulses(const std::vector<uint16_t> & pulse);
  void enterSearchingLocked();
  void enterStoppedLocked();
  bool checkLineInLastFrameLocked();
  void searchTimerCallback();

  using TwistPublisher = rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>;
  using ServosPositionPublisher =
    rclcpp_lifecycle::LifecyclePublisher<servo_controller_msgs::msg::ServosPosition>;

  std::unique_ptr<LineTracker> line_tracker_;
  std::unique_ptr<Pid> angular_pid_;
  std::shared_ptr<TwistPublisher> cmd_vel_pub_;
  std::shared_ptr<ServosPositionPublisher> servo_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<ZoneMonitor> obstacle_monitor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_running_srv_;
  rclcpp::Service<interfaces::srv::SetPoint>::SharedPtr set_target_color_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_target_color_srv_;
  rclcpp::Service<interfaces::srv::SetFloat64>::SharedPtr set_threshold_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_lap_stats_srv_;
  rclcpp::TimerBase::SharedPtr search_timer_;

  std::mutex state_mutex_;
  BehaviorState state_ = BehaviorState::kIdle;
  bool target_color_calibrated_ = false;
  cv::Vec3d target_bgr_mean_{0.0, 0.0, 0.0};
  cv::Mat last_frame_;
  rclcpp::Time last_callback_time_;
  bool has_last_callback_time_ = false;
  bool line_lost_since_valid_ = false;
  rclcpp::Time line_lost_since_;
  // Remembers what to resume once STOPPED_OBSTACLE clears — see
  // scanCallback.
  BehaviorState pre_obstacle_state_ = BehaviorState::kIdle;

  // SEARCHING state. kPanRight/kPanLeft step search_current_bearing_deg_
  // out from 0 toward +-search_arm_max_bearing_deg in
  // search_arm_step_deg increments, checking the frame after every step —
  // so a line found partway through the sweep stops it immediately
  // instead of only ever checking the two endpoints.
  SearchPhase search_phase_ = SearchPhase::kPanRight;
  bool search_phase_action_sent_ = false;
  rclcpp::Time search_phase_deadline_;
  rclcpp::Time search_deadline_;
  double search_current_bearing_deg_ = 0.0;
  int search_found_side_ = 0;  // +1 right, -1 left, 0 = not found yet
  double search_found_bearing_deg_ = 0.0;  // exact bearing the line was found at
  // Real IMU-integrated yaw during the kReturnCenter turn, so the chassis
  // turns by roughly search_found_bearing_deg_ instead of a fixed
  // duration — see imuCallback. Only accumulates while
  // search_return_integrating_ is true.
  bool search_return_integrating_ = false;
  double search_return_integrated_yaw_rad_ = 0.0;
  bool search_return_has_last_imu_stamp_ = false;
  rclcpp::Time search_return_last_imu_stamp_;
  // Arm pulses (id 1-4) captured the instant SEARCHING starts — the
  // "how it was" position panArmToBearingDeltaLocked/
  // restoreOriginalArmPulsesLocked pivot around and return to.
  std::array<int32_t, 4> search_original_pulses_{500, 500, 500, 500};
  bool has_search_original_pulses_ = false;

  double linear_speed_max_ = 0.15;
  double linear_speed_min_ = 0.10;
  double curvature_speed_gain_ = 0.0;

  // Lap tracker: based on /odom_raw (dead-reckoning only, see CLAUDE.md),
  // good enough for performance tuning even if it drifts. A lap starts
  // when ~/set_running(true) succeeds (see odomCallback, which captures
  // the first odom position it sees afterward as the start) and completes
  // when the robot, having moved more than lap_loop_radius_m away from
  // that start, comes back within that same radius.
  bool lap_tracking_active_ = false;
  bool lap_awaiting_start_odom_ = false;
  bool lap_has_left_start_ = false;
  double lap_start_x_ = 0.0;
  double lap_start_y_ = 0.0;
  double lap_last_x_ = 0.0;
  double lap_last_y_ = 0.0;
  bool lap_has_last_odom_ = false;
  double lap_distance_traveled_m_ = 0.0;
  rclcpp::Time lap_start_time_;
  bool has_last_lap_ = false;
  double last_lap_time_s_ = 0.0;
  double last_lap_distance_m_ = 0.0;
  double last_lap_avg_speed_mps_ = 0.0;
  int color_sample_patch_px_ = 20;
  double base_l_tolerance_ = 40.0;
  double base_ab_tolerance_ = 20.0;
};

}  // namespace landerpi_adaptive_line_follower
