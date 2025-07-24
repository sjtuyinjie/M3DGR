# VoxelMap
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/VoxelMap_M3DGR

cd baseline_systems/VoxelMap_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch livox_repub mid360_repub.launch
(roslaunch livox_repub avia_repub.launch # for avia)

source devel/setup.bash
roslaunch voxel_map mapping_m3dgr.launch

rosbag play Dynamic01.bag
```
