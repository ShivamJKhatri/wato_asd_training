#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void configure(double lookahead, double max_linear, double max_angular,
                   double goal_tolerance, double turn_slowdown);

    // Pure Pursuit. Returns (linear m/s, angular rad/s) for a robot at
    // (x, y, yaw). Zero when the path is empty or the goal has been reached --
    // both of which must actually stop the robot, so they come first.
    std::pair<double, double> computeVelocity(const nav_msgs::msg::Path& path,
                                              double x, double y, double yaw) const;

  private:
    rclcpp::Logger logger_;

    double lookahead_ = 1.5;
    double max_linear_ = 1.3;
    double max_angular_ = 1.5;
    double goal_tolerance_ = 0.5;

    // How sharply speed falls off with curvature. 0 is constant speed.
    double turn_slowdown_ = 2.0;
};

}

#endif
