/**
 * @file subscriber_member_function.cpp
 * @author Kiran S Patil (kpatil27.umd.edu)
 * @brief Function Implementation file for subscriber_member_function
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/beginner_tutorials/subscriber_member_function.hpp"

/**
 * @brief Function Implementation for topic_callback
 *
 */
void MinimalSubscriber::topic_callback(const std_msgs::msg::String & msg) const {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
