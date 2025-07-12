# MM-LINS
## m3dgr
```
source devel/setup.bash

roslaunch fast_lio mapping_avia.launch

source devel/setup.bash

roslaunch aloam_velodyne fastlio_ouster64.launch

rosbag play sharp_turn_1.bag /livox/avia/lidar:=/livox/lidar /livox/avia/imu:=/livox/imu
```
