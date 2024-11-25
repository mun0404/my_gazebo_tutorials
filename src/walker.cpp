/**
 * @file walker.cpp
 * @author Mohammed Munawwar
 * @brief C++ header file for the walker node
 * @version 0.1
 * @date 2024-11-24
 *
 * @copyright Copyright (c) 2024
 * @license MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "../include/walker.hpp"

/**
 * @brief Constructor for the TurtleBotNode class.
 * Initializes the ROS 2 node, sets up the publisher and subscriber, and creates
 * a TurtleBot instance.
 */
TurtleBotNode::TurtleBotNode() : Node("turtlebot_node") {
  velocity_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&TurtleBotNode::laser_callback, this, std::placeholders::_1));
  turtlebot_ = std::make_shared<TurtleBot>();
}

/**
 * @brief Callback function for processing LaserScan messages.
 * Updates the TurtleBot state and publishes the velocity commands.
 *
 * @param msg Shared pointer to the incoming LaserScan message.
 */
void TurtleBotNode::laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  turtlebot_->update(msg);
  twist_msg_.linear.x = turtlebot_->get_linear_vel();
  twist_msg_.angular.z = turtlebot_->get_angular_vel();
  velocity_publisher_->publish(twist_msg_);
}

// RotatingState class definition

/**
 * @brief Static member to toggle rotation direction.
 */
bool RotatingState::is_clockwise = true;

/**
 * @brief Constructor for the RotatingState class.
 * Sets the initial angular velocity based on the rotation direction.
 */
RotatingState::RotatingState() {
  if (is_clockwise) {
    angular_vel = -0.5;
  } else {
    angular_vel = 0.5;
  }
}

/**
 * @brief Updates the robot's velocity based on LaserScan data.
 * Stops linear motion if an obstacle is close.
 *
 * @param msg Shared pointer to the incoming LaserScan message.
 */
void RotatingState::update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!msg->ranges.empty() && msg->ranges[0] < 1) {
    linear_vel = 0.0;
  } else {
    linear_vel = 0.2;
    angular_vel = 0.2;
  }
}

/**
 * @brief Determines the next state for the robot.
 * Switches to ForwardState if no obstacle is detected.
 *
 * @param msg Shared pointer to the incoming LaserScan message.
 * @return Shared pointer to the next state, or nullptr if no state change is
 * required.
 */
std::shared_ptr<TurtleBotState> RotatingState::next_state(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!msg->ranges.empty() && msg->ranges[0] >= 1) {
    is_clockwise = !is_clockwise;
    return std::make_shared<ForwardState>();
  }
  return nullptr;
}

/**
 * @brief Gets the linear velocity for the robot.
 *
 * @return Linear velocity.
 */
double RotatingState::get_linear_vel() const { return linear_vel; }

/**
 * @brief Gets the angular velocity for the robot.
 *
 * @return Angular velocity.
 */
double RotatingState::get_angular_vel() const { return angular_vel; }

// ForwardState class definition

/**
 * @brief Updates the robot's velocity based on LaserScan data.
 * Reduces speed if an obstacle is close.
 *
 * @param msg Shared pointer to the incoming LaserScan message.
 */
void ForwardState::update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  linear_vel = 0.2;
  angular_vel = 0.0;

  if (!msg->ranges.empty() && msg->ranges[0] < 0.8) {
    linear_vel = 0.0;
  }
}

/**
 * @brief Determines the next state for the robot.
 * Switches to RotatingState if an obstacle is detected.
 *
 * @param msg Shared pointer to the incoming LaserScan message.
 * @return Shared pointer to the next state, or nullptr if no state change is
 * required.
 */
std::shared_ptr<TurtleBotState> ForwardState::next_state(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!msg->ranges.empty() && msg->ranges[0] < 0.8) {
    return std::make_shared<RotatingState>();
  }
  return nullptr;
}

/**
 * @brief Gets the linear velocity for the robot.
 *
 * @return Linear velocity.
 */
double ForwardState::get_linear_vel() const { return linear_vel; }

/**
 * @brief Gets the angular velocity for the robot.
 *
 * @return Angular velocity.
 */
double ForwardState::get_angular_vel() const { return angular_vel; }

// TurtleBot class definition

/**
 * @brief Constructor for the TurtleBot class.
 * Initializes the robot in the ForwardState.
 */
TurtleBot::TurtleBot() : current_state_(std::make_shared<ForwardState>()) {}

/**
 * @brief Updates the robot's state based on LaserScan data.
 *
 * @param msg Shared pointer to the incoming LaserScan message.
 */
void TurtleBot::update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  current_state_->update(msg);
  auto next_state = current_state_->next_state(msg);
  if (next_state) {
    current_state_ = next_state;
  }
}

/**
 * @brief Gets the robot's linear velocity.
 *
 * @return Linear velocity.
 */
double TurtleBot::get_linear_vel() const {
  return current_state_->get_linear_vel();
}

/**
 * @brief Gets the robot's angular velocity.
 *
 * @return Angular velocity.
 */
double TurtleBot::get_angular_vel() const {
  return current_state_->get_angular_vel();
}

// Main function

/**
 * @brief Entry point for the walker node.
 * Initializes the ROS 2 node and spins it to process callbacks.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleBotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
