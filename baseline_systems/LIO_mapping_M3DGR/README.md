# LIO-mapping
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LIO_mapping_M3DGR

cd LIO_mapping_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

roslaunch lio test_indoor.launch (roslaunch lio 16_scans_test.launch )

rosbag play sharp_turn_1.bag /livox/mid360/imu:=/imu/data
```
