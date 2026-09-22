#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "map_memory_node.hpp"

namespace
{
// Smallest turn that counts as a new viewpoint. The costmap rotates with the
// robot, so spinning in place reveals an entirely new view while the distance
// travelled stays zero.
constexpr double kDefaultUpdateAngle = 0.5;

// Shortest signed angle from b to a, so that crossing +/-pi is not mistaken
// for a full turn.
double angleDifference(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}
}  // namespace

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  // A 30 m x 30 m world at 0.2 m per cell, anchored at the world origin. The
  // costmap is finer (0.1 m) so that roughly four of its cells land in each
  // map cell and rotation cannot punch holes through the result.
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("width", 150);
  this->declare_parameter("height", 150);
  this->declare_parameter("origin_x", -15.0);
  this->declare_parameter("origin_y", -15.0);
  this->declare_parameter("global_frame", "sim_world");
  this->declare_parameter("update_distance", 1.5);
  this->declare_parameter("update_angle", kDefaultUpdateAngle);

  global_frame_ = this->get_parameter("global_frame").as_string();
  update_distance_ = this->get_parameter("update_distance").as_double();
  update_angle_ = this->get_parameter("update_angle").as_double();

  map_memory_.initGrid(
    this->get_parameter("resolution").as_double(),
    static_cast<int>(this->get_parameter("width").as_int()),
    static_cast<int>(this->get_parameter("height").as_int()),
    this->get_parameter("origin_x").as_double(),
    this->get_parameter("origin_y").as_double());

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Yaw straight out of the quaternion. The robot drives on a plane, so roll
  // and pitch carry nothing the map needs.
  const auto & q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  have_odom_ = true;
}

void MapMemoryNode::updateMap()
{
  if (have_costmap_ && have_odom_) {
    const double dx = robot_x_ - last_x_;
    const double dy = robot_y_ - last_y_;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double turn = std::abs(angleDifference(robot_yaw_, last_yaw_));

    if (!have_fused_ || distance >= update_distance_ || turn >= update_angle_) {
      map_memory_.integrate(latest_costmap_, robot_x_, robot_y_, robot_yaw_);

      last_x_ = robot_x_;
      last_y_ = robot_y_;
      last_yaw_ = robot_yaw_;
      have_fused_ = true;
    }
  }

  // Published every tick, including before the first fusion: the planner
  // cannot plan against a topic that has never published.
  map_pub_->publish(buildMessage());
}

nav_msgs::msg::OccupancyGrid MapMemoryNode::buildMessage()
{
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = global_frame_;

  msg.info.resolution = static_cast<float>(map_memory_.resolution());
  msg.info.width = static_cast<uint32_t>(map_memory_.width());
  msg.info.height = static_cast<uint32_t>(map_memory_.height());
  msg.info.origin.position.x = map_memory_.originX();
  msg.info.origin.position.y = map_memory_.originY();
  msg.info.origin.orientation.w = 1.0;

  msg.data = map_memory_.data();

  return msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
