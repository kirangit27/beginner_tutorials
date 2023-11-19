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
  // Default frequency value
  double frequency = 2.0;
   // Check if a custom frequency is provided as a command-line argument
  if (argc > 1) {
    frequency = std::atof(argv[1]);
  }
  rclcpp::spin(std::make_shared<MinimalPublisher>(frequency));
  rclcpp::shutdown();
  return 0;
}

