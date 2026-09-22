#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  // How far along the path to aim. Too short and the robot weaves; too long
  // and it cuts corners into the inflation the planner worked around.
  this->declare_parameter("lookahead", 1.5);
  this->declare_parameter("max_linear", 1.3);
  this->declare_parameter("max_angular", 1.5);
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("control_period", 0.1);
  this->declare_parameter("turn_slowdown", 2.0);

  control_.configure(
    this->get_parameter("lookahead").as_double(),
    this->get_parameter("max_linear").as_double(),
    this->get_parameter("max_angular").as_double(),
    this->get_parameter("goal_tolerance").as_double(),
    this->get_parameter("turn_slowdown").as_double());

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(this->get_parameter("control_period").as_double()),
    std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_ = *msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  const auto & q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  have_odom_ = true;
}

void ControlNode::controlLoop()
{
  // Zero-initialised, so losing odometry stops the robot rather than letting
  // it coast on the last command.
  geometry_msgs::msg::Twist cmd;

  if (have_odom_) {
    const auto velocity = control_.computeVelocity(path_, robot_x_, robot_y_, robot_yaw_);
    cmd.linear.x = velocity.first;
    cmd.angular.z = velocity.second;
  }

  cmd_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
