# Seyond Lidar ROS Usage Document

## About the project
  The project is based on the seyond client sdk and supports all types of lidars for seyond. After the project is started, it will monitor the UDP data of Lidar and use topic:/iv_points to publish point cloud data. This project serves as a demo for customers to refer to how to use seyond client sdk on ros.

  **directory structure**

```
├── cfg                                 // ros1 dynamic parameters
├── config                              // lidar config file
├── doc                                 // usage document
├── launch                              // ros1 & ros2 launch file
├── msg                                 // msg file
├── node                                // main file
│   ├── seyond_node.cc
│   └── seyond_test.cc
├── package.xml
├── rviz                                // rviz config file
├── src
│   ├── driver
│   │   ├── driver_lidar.cc
│   │   ├── driver_lidar.h
│   │   ├── point_types.h
│   │   ├── ros1_driver_adapter.hpp
│   │   ├── ros2_driver_adapter.hpp
│   │   └── yaml_tools.hpp
│   ├── multi_fusion                    // for multiple lidars fusion
│   │   ├── ros1_multi_fusion.hpp
│   │   └── ros2_multi_fusion.hpp
│   ├── test                            // test node code
│   │   ├── ros1_test.hpp
│   │   └── ros2_test.hpp
│   └── inno_sdk                        // seyond sdk
├── CMakeLists.txt
├── LICENSE
└── README.md

```

## Environment and Dependencies

The following official versions are verified to support, of course **Rolling distribution** is also supported

| Distro         | Link                               | Release data     | EOL data       |
|:--------:      | :---------:                        | :---------:      | :---------:    |
| melodic        |  https://wiki.ros.org/melodic      |  May 23rd, 2018  | June     2023  |
| noetic         |  https://wiki.ros.org/noetic       |  May 23rd, 2020  | May      2025  |
| foxy           |  https://docs.ros.org/en/foxy      |  June 5th, 2020  | June     2023  |
| galactic       |  https://docs.ros.org/en/galactic  |  May 23rd, 2021  | December 2022  |
| humble         |  https://docs.ros.org/en/humble    |  May 23rd, 2022  | May      2027  |
| jazzy          |  https://docs.ros.org/en/jazzy     |  May 23rd, 2024  | May      2029  |

## libyaml-cpp

Installation:

```sh
sudo apt-get update
sudo apt-get install -y  libyaml-cpp-dev
```

## Config params
  please refer to ./config/config.yaml, support multiple lidars input

## ROS params
| Parameter          | Default Value | description   |
|:--------:          | :---------:   | :---------:   |
| config_path        |  ""           |  config_path, if use this param, other params will become invalid   |
| log_level          |  info         |  limit log from lidar, can choose from (info warn error)   |
| replay_rosbag      |  false        |  replay rosbag packet flag   |
| packet_mode        |  false        |  packet mode enable   |
| aggregate_num      |  20           |  aggregate packets num   |
| frame_id           |  seyond       |      -        |
| frame_topic        |  iv_points    |      -        |
| packet_topic       |  iv_packets   |      -        |
| lidar_name         |  seyond       |  lidar name   |
| lidar_ip           |  172.168.1.10 |      -        |
| port               |  8010         |  tcp port     |
| udp_port           |  8010         |  if < 0, use tcp for transmission, if = 0, use lidar configured udp_port, if > 0, set the value to udp_port   |
| reflectance_mode   |  true         |  0:intensiy mode 1:reflectance mode   |
| multiple_return    |  1            |  lidar detection echo mode   |
| enable_falcon_ring |  false        |  enable falcon ring_id calculation   |
| enable_imu_msg     |  false        |  enable imu message publish   |
| continue_live      |  false        |  fatal error encountered, restart driver   |
| inno_pc_file       |  ""           |  path of inno_pc file   |
| pcap_file          |  ""           |  path of path playback pcapfile   |
| hv_table_file      |  ""           |  path of hv table file, only for generic lidar   |
| packet_rate        |  10000        |  file playback rate   |
| file_rewind        |  0            |  number of file replays 0:no rewind -1: unlimited times   |
| max_range          |  2000         |  display point maximum distance   |
| min_range          |  0.4          |  display point minimum distance   |
| name_value_pairs   |  ""           |  some settings of lidar are consistent with the usage of inno_pc_client   |
| coordinate_mode    |  3            |  convert the xyz direction of a point cloud, x/y/z, 0:up/right/forward 3:forward/left/up  |
| transform_enable   |  false        |  transform enable   |
| x                  |  0.0          |      -        |
| y                  |  0.0          |      -        |
| z                  |  0.0          |      -        |
| pitch              |  0.0          |      -        |
| yaw                |  0.0          |      -        |
| roll               |  0.0          |      -        |
| transform_matrix   |  ""           |  transform matrix string, if not empty, priority is higher than x/y/z/pitch/yaw/roll   |


## RUN

 **Compile**
  copy /seyond_lidar_ros/ directory to your ROS/ROS2 workspace
  
  ```bash
  // build inno_sdk
  cd seyond_lidar_ros
  cd src/inno_sdk/build
  ./build_unix.sh
  cd -
  ```

  ```bash
  source /opt/ros/<ROS_DISTRO>/setup.sh
  // Ensure that the value of $ROS_VERSION is correct.
  ```

  ```bash
  // for ROS
  catkin_make install
  ```
  ```bash
  // for ROS2
  colcon build
  ```

 **Run driver**

  ```bash
  source install/setup.bash
  roslaunch seyond start_with_rviz.launch
  // or
  ros2 launch seyond start_with_rviz.py
  ```
