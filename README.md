# beginner_tutorials

## ROS 2 Programming Assignment 1 - Publisher/Subscriber


### Setup

 - First create a ROS2 [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

 - Inside the src folder present in the workspace clone the github repo.

    ```$ git clone  https://github.com/kirangit27/beginner_tutorials.git```

 - [Build](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) the package.
 Navigate back to workspace directory and run the following build command.

    ```$ colcon build```

### Execution
 - Running the publisher node.

    ```$ ros2 run beginner_tutorials talker```

 - Running the subscriber node.

    ```$ ros2 run beginner_tutorials listener```

Note: It is assummed that all the dependencies are already present.
dependencies - ROS2 Humble, ament_cmake, rclcpp, std_msgs
