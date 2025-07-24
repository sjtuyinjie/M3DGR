# Faster-LIO
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/Faster_LIO_M3DGR

cd baseline_systems/Faster_LIO_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch faster_lio mapping_m3dgr_avia.launch

# for mid360
roslaunch faster_lio mapping_m3dgr_mid360.launch

rosbag play Dynamic01.bag
```
