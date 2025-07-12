# VIW-Fusion


## 3. Example

```

//m2dgr_plus
source devel/setup.bash

roslaunch vins vins_rviz.launch

rosrun vins viwo_node src/config/realsense_d435i/m2dgr_plus_realsense_stereo_imu_config_ridgeback.yaml 
(optional) rosrun loop_fusion loop_fusion_node src/config/realsense_d435i/m2dgr_plus_realsense_stereo_imu_config_ridgeback.yaml

//m3dgr
source devel/setup.bash

roslaunch vins vins_rviz.launch

rosrun vins viwo_node src/config/realsense_d435i/m3dgr_realsense_stereo_imu_config_ridgeback.yaml 
(optional) rosrun loop_fusion loop_fusion_node src/config/realsense_d435i/m3dgr_realsense_stereo_imu_config_ridgeback.yaml

```

