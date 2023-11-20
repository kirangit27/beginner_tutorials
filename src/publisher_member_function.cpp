/**
 * @file publisher_member_function.cpp
 * @author Kiran S Patil (kpatil27.umd.edu)
 * @brief Function Implementation file for publisher member function
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

    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Modifying String - "<<request->new_msg);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Modifying String - "<<request->new_msg);                     
    RCLCPP_WARN_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Modifying String - "<<request->new_msg);  
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Modifying String - "<<request->new_msg);                     
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Modifying String - "<<request->new_msg);                                    
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