
/**
 * @file walker.hpp
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
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief Abstract base class representing a state of the TurtleBot.
 */
class TurtleBotState {
 public:
  /**
   * @brief Virtual destructor for TurtleBotState.
   */
  virtual ~TurtleBotState() = default;

  /**
   * @brief Updates the state based on the latest laser scan data.
   * @param msg Shared pointer to the LaserScan message.
   */
  virtual void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;

  /**
   * @brief Gets the linear velocity for the current state.
   * @return Linear velocity.
   */
  virtual double get_linear_vel() const = 0;

  /**
   * @brief Gets the angular velocity for the current state.
   * @return Angular velocity.
   */
  virtual double get_angular_vel() const = 0;

  /**
   * @brief Determines the next state based on the laser scan data.
   * @param msg Shared pointer to the LaserScan message.
   * @return Shared pointer to the next state.
   */
    virtual std::shared_ptr<TurtleBotState> next_state(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;


  double linear_vel = 0.0; /**< Linear velocity for the current state. */
  double angular_vel = 0.0; /**< Angular velocity for the current state. */
};

/**
 * @brief Represents the forward movement state of the TurtleBot.
 */
class ForwardState : public TurtleBotState {
 public:
  /**
   * @brief Updates the state based on the laser scan data.
   * @param msg Shared pointer to the LaserScan message.
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) override;

  /**
   * @brief Determines the next state based on the laser scan data.
   * @param msg Shared pointer to the LaserScan message.
   * @return Shared pointer to the next state.
   */
    std::shared_ptr<TurtleBotState> next_state(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) override;


  /**
   * @brief Gets the linear velocity for the forward state.
   * @return Linear velocity.
   */
  double get_linear_vel() const override;

  /**
   * @brief Gets the angular velocity for the forward state.
   * @return Angular velocity.
   */
  double get_angular_vel() const override;
};

/**
 * @brief Represents the rotating state of the TurtleBot.
 */
class RotatingState : public TurtleBotState {
 public:
  /**
   * @brief Constructor for RotatingState.
   */
  RotatingState();

  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) override;

  /**
   * @brief Determines the next state based on the laser scan data.
   * @param msg Shared pointer to the LaserScan message.
   * @return Shared pointer to the next state.
   */
    std::shared_ptr<TurtleBotState> next_state(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) override;

  /**
   * @brief Gets the linear velocity for the rotating state.
   * @return Linear velocity.
   */
  double get_linear_vel() const override;

  /**
   * @brief Gets the angular velocity for the rotating state.
   * @return Angular velocity.
   */
  double get_angular_vel() const override;

 private:
  static bool is_clockwise; /**< Flag to determine the direction of rotation. */
};

/**
 * @brief Represents the TurtleBot and manages its current state.
 */
class TurtleBot {
 public:
  /**
   * @brief Constructor for TurtleBot.
   */
  TurtleBot();

  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Gets the linear velocity of the TurtleBot.
   * @return Linear velocity.
   */
  double get_linear_vel() const;

  /**
   * @brief Gets the angular velocity of the TurtleBot.
   * @return Angular velocity.
   */
  double get_angular_vel() const;

 private:
// Pointer to the current state of the TurtleBot.
std::shared_ptr<TurtleBotState> current_state_;
};

/**
 * @brief ROS 2 node that integrates the TurtleBot and handles laser scan data and velocity publishing.
 */
class TurtleBotNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for TurtleBotNode.
   */
  TurtleBotNode();

 private:
  /**
   * @brief Callback function for laser scan data.
   * @param msg Shared pointer to the LaserScan message.
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Shared pointer to the TurtleBot object.
  std::shared_ptr<TurtleBot> turtlebot_;
  // Publisher for velocity commands.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  // Subscriber for laser scan data.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
  laser_subscriber_;
  // Twist message for publishing velocity commands.
  geometry_msgs::msg::Twist twist_msg_;
};
