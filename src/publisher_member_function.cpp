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

  message.data = getData();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Publishing: " << message.data.c_str());
  publisher_->publish(message);
}

/**
 * @brief 
 * 
 */
void MinimalPublisher::modify_string_service(
      const std::shared_ptr<beginner_tutorials::srv::NewMsg::Request> request,
      std::shared_ptr<beginner_tutorials::srv::NewMsg::Response> response) {

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
    //                    "Modifying String - "<<request->new_msg);
    // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("minimal_publisher"),
    //                    "Modifying String - "<<request->new_msg);                     
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("minimal_publisher"),
    //                    "Modifying String - "<<request->new_msg);  
    // RCLCPP_ERROR_STREAM(rclcpp::get_logger("minimal_publisher"),
    //                    "Modifying String - "<<request->new_msg);                     
    // RCLCPP_FATAL_STREAM(rclcpp::get_logger("minimal_publisher"),
    //                    "Modifying String - "<<request->new_msg);                                    
    base_string_ = request->new_msg;
    response->set__response(true);
}

/**
 * @brief 
 * 
 */
std::string MinimalPublisher::getData() {
    return base_string_ + " - " + std::to_string(count_++);
}