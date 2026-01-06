---
title: Basic Guide to build and run ROS 2 Interfaces (Python & C++)
tags: ros2
article_header:
  type: cover
pseudocode: true
---

If you don't know about ROS 2 Topics and Services, go to [this](https://jaehyun-jeong.github.io/2025/12/09/ros2-topics.html) and [this](https://jaehyun-jeong.github.io/2025/12/21/ros2-services.html) page to learn.

**ROS 2 Interfaces define contents type when sending data through topics and services.**

Interfaces for topics are called msg, and interfaces for services are called srv.

Msg is a unidirectional data, so if publisher send msg, subscriber only receives msg. However, when it comes to services, if a server sends data, a client can respond.

In these sort of interaction in ROS 2, Interfaces define msg and srv.

You can check any primitive types for an Interface is in [this](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html) page. On top of that, you can check example_interfaces and common interfaces in [this](https://github.com/ros2/example_interfaces) and [this](https://github.com/ros2/common_interfaces) repository.

## Creating a simple package for interfaces

```bash
ros2 pkg create <package name for interfaces>

cd <package name for interfaces>
rm -r include/ src/
```

In the package.xml add below three lines
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="jj@todo.todo">jj</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <!-- ========================================== -->
  <!-- These three lines are what you need to add -->
  <!-- ========================================== -->

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <!-- ========================================== -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

And, in the CMakeLists.txt, remove lines like the code below.
```txt
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ====================
# = REMOVE FROM HERE =
# ====================

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ====================

# =================
# = ADD THIS PART =
# =================

find_package(rosidl_default_generators REQUIREDD)

rosidl_generate_interfaces(${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

# =================

ament_package()
```

## Message

First, in the interface package directory, create a directory for messages.

```bash
mkdir msg
```

Let's create an msg file with the command below.

```bash
cd msg
touch <msg name>.msg
```

Then you can write down like the code below.

```
float64 temperature
bool are_motors_ready
string debug_message
```
*<msg name>.msg*

After this be sure by checking
```bash
ros2 interface show <interface package name>/msg/<msg name>
```

## Import a custom interface

### In Python

```python
import <interface package name>.msg import <msg name>
```

**NOTE: Be sure to build with "colcon build" before importing an interface**

### example
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisherNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("hardware_status_publisher")  # MODIFY NAME
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Hw status publisher has been started.")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 43.7
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special"
        self.hw_status_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### In C++

```cpp
#include "<interface package name>/msg/<interface name>.hpp"
```

**NOTE: If you decided the interface name as "HardwareStatus", then in the .cpp file, you need to include with "hardware_status.hpp"**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using namespace std::chrono_literals;


class HardwareStatusPublisherNode : public rclcpp::Node{  // MODIFY NAME
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher_node")
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>(
            "hardware_status",
            10
        );

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this)
        );
        RCLCPP_INFO(this->get_logger(), "Hardware status publisher has been started");
    }
private:
    void publishHardwareStatus(){
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 57.2;
        msg.are_motors_ready = false;
        msg.debug_message = "Motors are too hot!";
        pub_ -> publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();  // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
```

## Service

First, in the interface package directory, create a directory for services.

```bash
mkdir srv
cd srv
touch <service name>.srv  # name service starting with verb.
```

Since services had **request** and **respond**, service requires "---" such as the code below.
```
float64 length
float64 width
---
float64 area
```
*<service name>.srv*

Then, you should add this service into rosidl_generate_interfaces

```txt
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "./msg/HardwareStatus.msg"  # No comma
  "./srv/ComputeRectangleArea.srv"
)

ament_package()
```

Just like the case of messages, services also can be imported or included in the same manner.
