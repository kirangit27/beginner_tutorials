cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  #UPDATED

set(srv_files           #UPDATED
  "srv/NewMsg.srv"
)

include_directories(include/beginner_tutorials) 

rosidl_generate_interfaces(${PROJECT_NAME}  #UPDATED
  ${srv_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target  #UPDATED
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_dependencies(rosidl_default_runtime)   #UPDATED

add_executable(talker src/publisher_member_function.cpp src/pub_main.cpp) 
add_executable(listener src/subscriber_member_function.cpp src/sub_main.cpp) 

ament_target_dependencies(talker rclcpp std_msgs) 
ament_target_dependencies(listener rclcpp std_msgs) 

target_link_libraries(talker "${cpp_typesupport_target}")   #UPDATED
target_link_libraries(listener "${cpp_typesupport_target}")   #UPDATED

install(TARGETS       #UPDATED
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY       #UPDATED
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_export_dependencies(rosidl_default_runtime)   #UPDATED

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
