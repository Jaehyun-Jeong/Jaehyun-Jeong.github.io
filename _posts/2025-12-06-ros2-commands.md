---
title: Basic Commands for ROS 2
tags: ros2
article_header:
  type: cover
pseudocode: true
---

## Run nodes

```bash
ros2 run <package name> <node name>
```

**NOTE: "-h" option shows arguments and options like below**
```bash
ros2 -h
ros2 run -h
ros2 node -h
```

## Checking running nodes

```bash
ros2 node list
```
*Check running nodes*

```bash
ros2 node info <node name>
```

**WARNING: It is not encouraged to run two nodes with identical names. These could run at the same time, but they will show the message like below**
```bash
WARNING: Be aware that there are nodes in the graph that share an exact name, which can have unintended side effects.
```

## Running nodes with the same node name

```bash
ros2 run <package name> <node name> --ros-args -r __node:=<new node name>
```
*"-r" can be replaced with "--remap"*

## Building commands

The basic build command is
```bash
colcon build
```
*Build all packages*

**NOTE: The build command should only be executed in the project folder that contains the src folder**

The command below builds only the selected package.
```bash
colcon build --packages-select <package name>
```

## Building commands only for Python

```bash
colcon build --packages-select <package name> --symlink-install
```
"--symlink-install" option makes the package run with source file. Therefore, rebuilding is unnecessary when the Python file changed.

## ROS 2 with GUI

The commands below open the GUI tools for ROS 2.
```bash
rqt
rqt_graph  # Shows a graph of packages
```

## Topics

The command below shows currently running topics.
```bash
ros2 topic list
```

To see what topic is recieving, run the command below.
```bash
ros2 topic echo <topic name>
```

```bash
ros2 topic info <topic name>
```

The commands below print frequency and bandwidth of the topic.
```bash
ros2 topic hz <topic name>
ros2 topic bw <topic name>
```

The command below instantly publish a topic.
```bash
ros2 topic pub -r <seconds> <topic name> <interface name> <data>
# Like this one
ros2 topic pub -r 5 /robot_news example_interfaces/msg/String "{data: 'Hello from the terminal'}"
```

The command below can change the topic name.
```bash
ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station -r robot_news:=abc
```
*robot_news to abc*

**NOTE: In the same way, node, topic publisher, and topic reciever can be remaped with -r option.**

## Interfaces

The command below returns the interface information.
```bash
ros2 interface <interface name>
```

```bash
ros2 interface show geometry_msgs/msg/Twist
```

## Bags

```bash
# Help
ros2 bag -h

# Record topics
ros2 bag record <topic name 1> <topic name 2> ...
# Record topics with custom record name
ros2 bag record -o <record name> <topic name 1> <topic name 2> ...
# Record all topics
ros2 bag record -a

# Play a record
ros2 bag play <record name>

# Print record Information
ros2 bag info <record name>
```

## Services

```bash
ros2 service call <server node name> <interface name> <request>
# Like this one
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"
```
