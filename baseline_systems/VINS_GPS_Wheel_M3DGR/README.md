# VINS-GPS-Wheel
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/VINS_GPS_Wheel_M3DGR

cd VINS_GPS_Wheel_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch vins_estimator m3dgr.launch

source devel/setup.bash
rosrun multisensor_fusion multisensor_fusion_node (optional, for GPS)

source devel/setup.bash
roslaunch vins_estimator vins_rviz.launch
```
## 3. Run M2DGR-Plus example
```
source devel/setup.bash
roslaunch vins_estimator m2dgr_plus.launch

source devel/setup.bash 
rosrun multisensor_fusion multisensor_fusion_node (optional, for GPS)

source devel/setup.bash
roslaunch vins_estimator vins_rviz.launch

```
