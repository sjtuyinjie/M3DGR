# LIO-mapping
## m3dgr
```
source devel/setup.bash

roslaunch livox_repub mid360_repub.launch

roslaunch lio test_indoor.launch

roslaunch lio 16_scans_test.launch

rosbag play sharp_turn_1.bag /livox/mid360/imu:=/imu/data
```
