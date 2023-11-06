

#include "../include/beginner_tutorials/subscriber_member_function.hpp"


void MinimalSubscriber::topic_callback(const std_msgs::msg::String & msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
