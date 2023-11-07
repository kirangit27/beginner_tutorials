/**
 * @file pub_main.cpp
 * @author Kiran S Patil (kpatil27.umd.edu)
 * @brief main file for publisher member function
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/beginner_tutorials/publisher_member_function.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

