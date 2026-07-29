# inno_pc_client
## A. What is inno_pc_client
```shell
inno_pc_client is used to record pointcloud in various forms of files(inno_pc, inno_pc_xyz, etc) and view pointcloud in browser.
```

## B. Introduction of parameter
```shell
# you can use ./inno_pc_client -h or ./inno_pc_client to get all information
--falcon-eye 10,10                                                     # x: ROI horizontal center (in [-60, 60] degrees)
                                                                       # y: ROI vertical center (in [-25, 25], degrees)
--reflectance 1                                                        # 1: intensity
                                                                       # 2: reflectance
--multireturn 1                                                        # 1: single
                                                                       # 2: twostrongest
                                                                       # 3: strongest+furthest
--lidar-id 172.168.1.10                                                # lidar IP
--udp-ip 172.168.1.10                                                  # UDP IP
--udp-port 8010                                                        # data udp destination port
--udp-port-status 8010                                                 # status udp destination port
--udp-port-message 8010                                                # message udp destination port
--udp-port-raw 8010                                                    # raw udp destination port
--udp-port-status-local @@@@                                           # <LOCAL_STATUS_UDP_DEST_PORT>
--tcp-port                                                             # tcp listen port
--status-interval-ms 50                                                # status udp packet's interval in MS
--record-inno-pc-filename out                                          # the name of recorded inno_pc file
--record-inno-pc-size-in-m 20                                          # record inno_pc file size
--inno-pc-record-npy                                                   # @@@@@@@
--record-png-filename out                                              # the name of recorded PNG file
--record-rosbag-filename out                                           # the name of recorded ROSBAG file
--record-rosbag-size-in-m 20                                           # record ROSBAG file size
--record-raw-filename out                                              # the name of recorded RAW file
--record-raw-size-in-m 20                                              # record RAW file size
--config /app/pointcloud/configs/inno_pc_server.config                 # Path to config file
--config2 /mnt/config_firmware/inno_internal_file_PCS_CFG              # Path to config file
--dtc /mnt/dtc_file.json                                               # Path to dtc file
--out-format                                                           # out format
--processed                                                            # @@@@
--debug                                                                # debug level
--quiet                                                                # @@@@@@
--log-filename /tmp/inno_pc_server.txt                                 # Path to log file
--log-file-rotate-number 2                                             # rotate number for log file
--log-file-max-size-k 20                                               # max log file size(k)
--error-log-filename /mnt/pointcloud_log/inno_pc_server.txt.err        # path to error log file
--error-log-file-rotate-number 2                                       # rotate number for error log file
--error-log-file-max-size-k 20                                         # max error log file size(k)
--show-viewer                                                          # open browser ro see point_cloud
```

## C. How to use inno_pc_client
```shell
# Connect to a live lidar and view the pointcloud in browser:
./inno_pc_client --lidar-ip 172.168.1.10 --lidar-port 8010 --lidar-udp-port 8010
http://viewer.innovusion.info/v60/?url=127.0.0.1&port=8011/stream&autoplay=-2.0

# Connect to a live lidar and record the pointcloud in out1.inno_pc_xyz file:
./inno_pc_client --lidar-ip 172.168.1.10 --lidar-port 8010 --lidar-udp-port 8010 --record-inno-pc-filename out1 --use-xyz 1

# Connect to a live lidar and record the pointcloud in out1.inno_pc file:
./inno_pc_client --lidar-ip 172.168.1.10 --lidar-port 8010 --lidar-udp-port 8010 --record-inno-pc-filename out1

# Read from an inno_pc file and repeat 10 times:
./inno_pc_client --file input.inno_pc --speed 15 --rewind 10
```

## D. Location
If you want to get detial of inno_pc_client, please press [here](../apps/pcs/pcs_main.cpp)
