# 09_how_to_set_roi_cn

## 9.1 功能说明

对于Falcon系列雷达，支持ROI实时调整

当需要在驱动侧调整雷达ROI时，请在以下两个topic发布数据，数据格式为Float64

```
/<lidar_name>_hori_roi
/<lidar_name>_vert_roi
```

## 9.2 命令行设置ROI测试

```
//for ROS
rostopic pub /<lidar_name>_hori_roi std_msgs/Float64 "data: <hori_roi_data>"
rostopic pub /<lidar_name>_vert_roi std_msgs/Float64 "data: <vert_roi_data>"

// for ROS2
ros2 topic pub /<lidar_name>_hori_roi std_msgs/msg/Float64 data:\ <hori_roi_data>
ros2 topic pub /<lidar_name>_vert_roi std_msgs/msg/Float64 data:\ <vert_roi_data>
```
