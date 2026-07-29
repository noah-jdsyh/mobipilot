# 09_how_to_set_roi

## 9.1 Function Description

For Falcon series Lidar, support real-time ROI adjustment.

When you need to adjust the radar ROI on the driver side, please publish data to the following two topics. The data format is Float64.

```
/<lidar_name>_hori_roi
/<lidar_name>_vert_roi
```

## 9.2 Command line test for ROI

```
//for ROS
rostopic pub /<lidar_name>_hori_roi std_msgs/Float64 "data: <hori_roi_data>"
rostopic pub /<lidar_name>_vert_roi std_msgs/Float64 "data: <vert_roi_data>"

// for ROS2
ros2 topic pub /<lidar_name>_hori_roi std_msgs/msg/Float64 data:\ <hori_roi_data>
ros2 topic pub /<lidar_name>_vert_roi std_msgs/msg/Float64 data:\ <vert_roi_data>
```
