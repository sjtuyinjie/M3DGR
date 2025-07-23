# A-LOAM
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/A_LOAM_M3DGR

cd A_LOAM_M3DGR

catkin_make
```
## M3DGR
```
 source devel/setup.bash
 roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch    # aloam_velodyne_HDL_32.launch
 
 source devel/setup.bash
 roslaunch livox_repub mid360_repub.launch
 
 rosbag play Dynamic01.bag

```
