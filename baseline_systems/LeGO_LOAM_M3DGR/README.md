# LeGO-LOAM
Quickly run on M3DGR:

## 1. Compie
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LeGO_LOAM_M3DGR

cd baseline_systems/LeGO_LOAM_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch livox_repub avia_repub.launch

source devel/setup.bash
roslaunch lego_loam m3dgr_avia_run.launch

rosbag play Dynamic01.bag  --clock
```
