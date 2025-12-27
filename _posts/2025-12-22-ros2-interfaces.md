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

And create a folder for msg

```bash
mkdir msg
```
