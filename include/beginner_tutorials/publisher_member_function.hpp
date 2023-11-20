/**
 * @file publisher_member_function.hpp
 * @author Kiran S Patil (kpatil27.umd.edu)
 * @brief Header file for publisher member function
 * @version 0.1
 * @date 2023-11-06
 *
 * MIT License
 * 
 * Copyright (c) 2023 Kiran S Patil
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <beginner_tutorials/srv/new_msg.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


/**
 * @brief A minimal ROS2 publisher node.
 *
 * This class demonstrates a simple ROS2 publisher node with a timer and a service.
 */
class MinimalPublisher : public rclcpp::Node {

  public:
    /**
     * @brief Constructor for the MinimalPublisher class.
     *
     * @param freq The desired frequency for the publisher.
     */
    MinimalPublisher(double freq) : Node("minimal_publisher"), count_(0) {
      double pub_freq = this->declare_parameter("publisher_frequency", freq);
      auto timer_interval = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / pub_freq));
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
          timer_interval, std::bind(&MinimalPublisher::timer_callback, this));
      service_ = this->create_service<beginner_tutorials::srv::NewMsg>(
        "modify_string", [this](
            const std::shared_ptr<beginner_tutorials::srv::NewMsg::Request> request,
            std::shared_ptr<beginner_tutorials::srv::NewMsg::Response> response) {
          modify_string_service(request, response);
        });
    }

  private:

    /**
     * @brief Callback function for the timer.
     */
    void timer_callback();

    /**
     * @brief Service callback for modifying the string.
     *
     * @param request The request containing the string to modify.
     * @param response The response containing the modified string.
     */
    void modify_string_service(
    const std::shared_ptr<beginner_tutorials::srv::NewMsg::Request> request,
    std::shared_ptr<beginner_tutorials::srv::NewMsg::Response> response);

    /**
     * @brief Get message to be published.
     *
     * @return The message to be published.
     */
    std::string getMsg();

    rclcpp::TimerBase::SharedPtr timer_;    ///< Timer for publishing messages.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;   ///< Publisher for string messages.
    size_t count_;    ///< Count of published messages.
    std::string base_string_ = "ROS2 Programming Assignment 2";   ///< Base string to be modified.
    rclcpp::Service<beginner_tutorials::srv::NewMsg>::SharedPtr service_;   ///< Service for modifying strings.
};

