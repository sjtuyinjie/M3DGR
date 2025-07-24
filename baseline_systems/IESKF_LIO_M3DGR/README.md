# IESKF-LIO
Quickly run on M3DGR :

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/IESKF_LIO_M3DGR

cd baseline_systems/IESKF_LIO_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch livox_repub avia_repub.launch

source devel/setup.bash
roslaunch eskf_lio m3dgr_avia.launch

# for mid360
source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

source devel/setup.bash
roslaunch eskf_lio m3dgr_mid360.launch

rosbag play Dynamic.bag

```
