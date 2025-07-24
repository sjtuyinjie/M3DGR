# HM-LIO
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/HM_LIO_M3DGR

cd baseline_systems/HM_LIO_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

source devel/setup.bash
roslaunch hlio run_eskf.launch

rosbag paly Dynamic01.bag
```
