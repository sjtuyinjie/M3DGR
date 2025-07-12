# VINS-GPS-Wheel
### m2dgr-plus
```
source devel/setup.bash

roslaunch vins_estimator m2dgr_plus.launch
 
rosrun multisensor_fusion multisensor_fusion_node (optional, for GPS)

roslaunch vins_estimator vins_rviz.launch

```
### m3dgr
```
source devel/setup.bash

roslaunch vins_estimator m3dgr.launch
 
rosrun multisensor_fusion multisensor_fusion_node (optional, for GPS)

roslaunch vins_estimator vins_rviz.launch
```
