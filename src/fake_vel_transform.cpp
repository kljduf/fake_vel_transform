// Copyright 2025 Lihan Chen
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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("yaw_feedback_angle_topic", "/target");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("reset_yaw_service_name", "reset_fake_yaw");
  this->declare_parameter<double>("cmd_vel_publish_frequency", 50.0);
  this->declare_parameter<float>("init_spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("yaw_feedback_angle_topic", yaw_feedback_angle_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("reset_yaw_service_name", reset_yaw_service_name_);
  this->get_parameter("cmd_vel_publish_frequency", cmd_vel_publish_frequency_);
  this->get_parameter("init_spin_speed", spin_speed_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  if (!yaw_feedback_angle_topic_.empty()) {
    yaw_feedback_angle_sub_ = this->create_subscription<wdr_msgs::msg::NavSend>(
      yaw_feedback_angle_topic_, 1,
      std::bind(&FakeVelTransform::yawFeedbackAngleCallback, this, std::placeholders::_1));
  }
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  reset_yaw_service_ = this->create_service<std_srvs::srv::Trigger>(
    reset_yaw_service_name_,
    std::bind(
      &FakeVelTransform::resetYawCallback, this, std::placeholders::_1, std::placeholders::_2));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 500Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(2), std::bind(&FakeVelTransform::publishTransform, this));

  // Timer to publish cmd_vel at configured frequency
  if (cmd_vel_publish_frequency_ > 0.0) {
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / cmd_vel_publish_frequency_));
    cmd_vel_timer_ =
      this->create_wall_timer(period, std::bind(&FakeVelTransform::publishCmdVel, this));
  } else {
    RCLCPP_WARN(get_logger(), "cmd_vel_publish_frequency is non-positive, cmd_vel timer disabled");
  }

  // 初始化 旋转角累积值 的时间
  last_integration_time_ = this->now();
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

void FakeVelTransform::yawFeedbackAngleCallback(const wdr_msgs::msg::NavSend::SharedPtr msg)
{
  latest_yaw_feedback_angle_ = normalizeAngle(msg->big_yaw);
  has_yaw_feedback_angle_ = true;
  tryAutoZeroCalibration();
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // NOTE: Haven't synced with local_plan
  if ((rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
    has_robot_base_angle_ = true;
    tryAutoZeroCalibration();
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  latest_cmd_vel_ = msg;
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // If received velocity cannot be synchronized, transform and cache it directly
    latest_aft_tf_vel_ = transformVelocity(msg, getCalibratedRobotBaseAngle());
    last_cmd_vel_update_time_ = this->now();
  } else {
    latest_cmd_vel_ = msg;
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = rclcpp::Clock().now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  has_robot_base_angle_ = true;
  tryAutoZeroCalibration();
  float yaw_diff = getCalibratedRobotBaseAngle();
  latest_aft_tf_vel_ = transformVelocity(current_cmd_vel, yaw_diff);
  last_cmd_vel_update_time_ = this->now();
}

void FakeVelTransform::publishCmdVel()
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  // 启动阶段（尚未收到 nav2/controller 的 cmd_vel）也要持续发布 0 速度，
  // 防止下游因为“没有话题输出”而保持上一拍非零速度或进入不期望状态。
  if (last_cmd_vel_update_time_.nanoseconds() == 0) {
    geometry_msgs::msg::Twist zero;
    // 保持原有接口语义：linear.z 用于透传 fake_cumulative_yaw_
    zero.linear.z = normalizeAngle(fake_cumulative_yaw_);
    cmd_vel_chassis_pub_->publish(zero);
    return;
  }

  // If stale for more than CONTROLLER_TIMEOUT, force linear x,y to zero
  if ((this->now() - last_cmd_vel_update_time_).seconds() > CONTROLLER_TIMEOUT) {
    latest_aft_tf_vel_.linear.x = 0.0;
    latest_aft_tf_vel_.linear.y = 0.0;
  }
  latest_aft_tf_vel_.linear.z = normalizeAngle(fake_cumulative_yaw_);
  cmd_vel_chassis_pub_->publish(latest_aft_tf_vel_);
}

void FakeVelTransform::publishTransform()
{
  //累积角度的时间戳
  auto current_time = this->now();
  double dt = (current_time - last_integration_time_).seconds();
  last_integration_time_ = current_time;

  // 1. 积分逻辑：根据当前的指令角速度，让 fake 坐标系旋转
  // 注意：需要加锁获取 latest_cmd_vel_
  double cmd_omega = 0.0;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (
      latest_cmd_vel_) {  //检查指针 latest_cmd_vel_ 是否有效（不为 nullptr），或者是 std::optional 是否包含值。防止直接访问空指针
      cmd_omega = latest_cmd_vel_->angular.z;
    }
  }

  // 只有当时间差合理时才积分（避免暂停恢复后的跳变）
  if (dt > 0 && dt < 0.5) {
    fake_cumulative_yaw_ += cmd_omega * dt;
    // 规范化角度到 -PI ~ PI (可选，但推荐)
    fake_cumulative_yaw_ = normalizeAngle(fake_cumulative_yaw_);
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;

  // 3. 【关键数学修改】
  // 原逻辑：Fake = 0  =>  TF = -Real
  // 新逻辑：Fake = Virtual =>  Real + TF = Virtual  =>  TF = Virtual - Real
  // 也就是说，Fake 坐标系相对于 Real 坐标系的旋转量
  double tf_yaw = fake_cumulative_yaw_ - getCalibratedRobotBaseAngle();

  tf2::Quaternion q;
  q.setRPY(0, 0, tf_yaw);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float current_real_yaw)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;

  // 线速度投影
  // 把 Fake 坐标系下的 (vx, vy) 转换到 Real 坐标系下。
  // 投影角度 = Real 坐标系相对于 Fake 坐标系的角度的【相反数】(用于逆变换)
  // 也就是：我们想知道 Real 相对于 Fake 转了多少度？
  // Delta = Real - Fake
  // 之前的逻辑：Fake=0，所以 Delta = Real。
  // 现在的逻辑：Delta = current_real_yaw - fake_cumulative_yaw_;

  float yaw_diff = current_real_yaw - fake_cumulative_yaw_;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

void FakeVelTransform::resetYawCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  fake_cumulative_yaw_ = getCalibratedRobotBaseAngle();
  last_integration_time_ = this->now();
  response->success = true;
  response->message = "fake_cumulative_yaw_ reset to calibrated robot base angle";
  RCLCPP_INFO(get_logger(), "Reset fake_cumulative_yaw_ to calibrated robot base angle");
}

double FakeVelTransform::getCalibratedRobotBaseAngle() const
{
  return normalizeAngle(current_robot_base_angle_ + robot_base_angle_compensation_);
}

void FakeVelTransform::tryAutoZeroCalibration()
{
  if (auto_zero_calibrated_ || !has_robot_base_angle_ || !has_yaw_feedback_angle_) {
    return;
  }

  robot_base_angle_compensation_ =
    normalizeAngle(latest_yaw_feedback_angle_ - current_robot_base_angle_);
  fake_cumulative_yaw_ = latest_yaw_feedback_angle_;
  last_integration_time_ = this->now();
  auto_zero_calibrated_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Auto zero calibrated: yaw_feedback=%.6f, robot_base=%.6f, compensation=%.6f",
    latest_yaw_feedback_angle_, current_robot_base_angle_, robot_base_angle_compensation_);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
