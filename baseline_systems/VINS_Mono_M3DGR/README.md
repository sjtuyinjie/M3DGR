# vins-mono
## m2dgr-plus
```
    source devel/setup.bash
    roslaunch vins_estimator m2dgr_plus.launch 
    
    source devel/setup.bash
    roslaunch vins_estimator vins_rviz.launch
    
    rosbag play tree3.bag --topic /camera/color/image_raw /camera/imu 
```
## m3dgr
```
    source devel/setup.bash
    roslaunch vins_estimator m3dgr.launch 
    
    source devel/setup.bash
    roslaunch vins_estimator vins_rviz.launch
    
```
