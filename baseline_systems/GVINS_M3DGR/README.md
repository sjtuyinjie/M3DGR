# GVINS
Quickly run on M3DGR:

## Compile 
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/GVINS_M3DGR

cd GVINS_M3DGR

catkin_make
```
## Run M3DGR example
```
source devel/setup.bash
roslaunch gvins m3dgr.launch

source devel/setup.bash
roslaunch gvins m3dgr_ungps.launch # can choose no gps

source devel/setup.bash
rviz -d ~/catkin_ws/src/GVINS/config/gvins_rviz_config.rviz
```
