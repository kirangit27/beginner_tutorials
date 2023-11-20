/**
 * @file pub_main.cpp
 * @author Kiran S Patil (kpatil27.umd.edu)
 * @brief main file for publisher member function
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

