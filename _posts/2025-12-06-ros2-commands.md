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
