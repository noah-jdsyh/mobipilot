# latency_diagram.py
## A. What is latency_diagram.py
```shell
This script is used to calculate the time it takes for the lidar to emit a laser and for the client to receive Innodatapackert.
```

## B. Environment
```shell
pip3 install matplotlib
pip3 install numpy
```

## C. Introduction of parameter
```shell
--unit 100                                        #100: latency time min unit
--step 1                                          #1: latency histogram show step length
--frame-number 100                                #100: get_pcd record frame number
--input latency.data                              #latency.data: input Innodatapacket latency data file
--lidar-ip 172.168.1.10                           #172.168.1.10: lidar ip
--lidar-port 8010                                 #8010: lidar port
--latency-file latency.data                       #latency.data: out latency data filename
```

## D. How to use latency_diagram.py
```shell
# Use default value
./latency_diagram.py

# calculate latency by using input file
./latency_diagram.py --input latency.data --frame-number 100 --latency-file test.data --step 1 --unit 100

# calculate latency by using lidar directly
./latency_diagram.py --lidar-ip 172.168.1.10 --lidar-port 8010 --frame-number 100 --latency-file test.data --step 1 --unit 100
```

## E. Location
If you want to get detial of latency_diagram.py, please press [here](../apps/tools/latency_diagram/latency_diagram.py)
