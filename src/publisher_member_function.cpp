/**
 * @file publisher_member_function.cpp
 * @author Kiran S Patil (kpatil27.umd.edu)
 * @brief Function Implementation file for publisher member function
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/beginner_tutorials/publisher_member_function.hpp"

/**
 * @brief Function Implementation for timer_callback
 *
 */
void MinimalPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}
