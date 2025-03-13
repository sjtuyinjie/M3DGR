# M3DGR
M3DGR: A Multi-sensor, Multi-scenario and Massive-baseline SLAM Dataset for Ground Robots


## 🎯 Notice
## This paper is under review now. Full datasets and codes will be released upon paper acceptance.

## SENSOR SETUP
### Acquisition Platform
Physical drawings and schematics of the ground robot. (a) Side view of the robot. (b) Sensor arrangement on the top layer. (c) Sensor arrangement on the middle and bottom layers. All dimensions are provided in centimeters.

<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/car.pdf" width="300px">
</div>
<p align="left">Figure 1. The directions of the sensors are marked in different colors,red for X,green for Y and blue for Z.</p>

### Sensor parameters

All the sensors and track devices and their most important parameters are listed as below:

* **LiDAR1** Livox Avia, Non-repetitive, 70.4° Horizontal Field of View (FOV), 77.2° vertical FOV, 10HZ, Max Range 450m, Range Precision 2cm, Angular Precision 0.05º, IMU 6-axis 200HZ.
* **LiDAR2** Livox MID-360, Non-repetitive, 360° Horizontal Field of View (FOV), -7° to +52° vertical FOV,  10Hz, Max Range 40 m, Range Resolution 3 cm, Angular Resolution 0.15°, IMU 6-axis, 200HZ.
* **V-I Sensor** Realsense d435i, RGB/Depth 640*480, 69°H-FOV, 42.5°V-FOV,15Hz; IMU 6-axis, 200Hz.
* **Omnidirectional Camera** Insta360 X4,  RGB 2880*1440, 360°H-FOV, 360°V-FOV, 15HZ.
* **Wheel Odometer** WHEELTEC, 2D, 20HZ.
* **GNSS Receiver** CUAV C-RTK9Ps, BDS/GPS/GLONASS/Galileo, 10HZ.
* **RTK Receiver**  CUAV C-RTK2HP,  localization accuracy 0.8cm(H)/1.5cm(V), 15HZ.
* **Motion-capture System** OptiTrack, localization accuracy 1mm, 360HZ.

The rostopics of our rosbag sequences are listed as follows:

* LiDAR1: `/livox/avia/lidar`

* LiDAR2: `/livox/mid360/lidar` 

* Wheel Odometer：`/odom`

* RGB Camera: `/camera/color/image_raw/compressed`

* Omnidirectional  Camera: `/cv_camera/image_raw/compressed`

* Depth Camera: `/camera/aligned_depth_to_color/image_raw/compressedDepth`

* GNSS:

 `/ublox_driver/ephem`
 
​	`/ublox_driver/glo_ephem`
 
​	`/ublox_driver/iono_params`
 
​	`/ublox_driver/range_meas`

​	`/ublox_driver/receiver_lla`

​	`/ublox_driver/receiver_pvt`

​	`/ublox_driver/time_pulse_info`


* IMU:
 
 `/camera/imu`
  
 `/livox/avia/imu`

 `/livox/mid360/imu`
 

## DATASET SEQUENCES
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/Traj.pdf" width="800px">
</div>
<p align="left">Figure 2. traj</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()

### Visual Challenge
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/light01.pdf" width="800px">
</div>
<p align="left">Figure 3. light01</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()

### Wheel Slippage
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/dark01.pdf" width="800px">
</div>
<p align="left">Figure 4. dark01</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()

### GNSS Denied
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/GNSS_Denied.pdf" width="800px">
</div>
<p align="left">Figure 5. GNSS_Denied</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()

### LiDAR Degeneration
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/corridor01.pdf" width="800px">
</div>
<p align="left">Figure 6. corridor01</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()

## 🔥 SLAM Benchmark
- A list of tested cutting-edge methods on **M3DGR**🦄 dataset:
- 
  - 💡 **Wheel system**:
  - 
    - ① Wheel Odom from [M3DGR](https://github.com/sjtuyinjie/M3DGR)'s wheel speedometer solution
      
  - 💡 **GNSS system**:
  - 
    - ② GNSS SPP from [Ground-Fusion++](https://github.com/sjtuyinjie/Ground-Fusion2) solution
  
  - 💡 **VO system**:
  
    - ③ [Tartanvo: A generalizable learning-based vo](https://arxiv.org/abs/2011.00359) and the [*open repository*](https://github.com/castacks/tartanvo) from PMLR2021
      
    - ④ [Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras](https://arxiv.org/abs/1610.06475) and the [*open repository*](https://github.com/raulmur/ORB_SLAM2) from T-RO2017
      
  - 💡 **VIO system**:
    
    - ⑤ [Orb-slam3: An accurate open-source library for visual, visual–inertial, and multimap slam](https://arxiv.org/abs/2007.11898) and the [*open repository*](https://github.com/UZ-SLAMLab/ORB_SLAM3) from T-RO2021
      
    - ⑥ [DM-VIO: Delayed marginalization visual-inertial odometry](https://arxiv.org/abs/2201.04114) and the [*open repository*](https://github.com/lukasvst/dm-vio) from RA-L2022
      
    - ⑦ [Vins-mono: A robust and versatile monocular visual-inertial state estimator](https://arxiv.org/abs/1708.03852) and the [*open repository*](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) from T-RO2018
      
    - ⑧ [VINS-RGBD: RGBD-inertial trajectory estimation and mapping for ground robots](https://www.mdpi.com/1424-8220/19/10/2251) and the [*open repository*](https://github.com/STAR-Center/VINS-RGBD) from Sensors2019
      
    - ⑨ [GVINS: Tightly coupled GNSS–visual–inertial fusion for smooth and consistent state estimation](https://arxiv.org/abs/2103.07899) and the [*open repository*](https://github.com/HKUST-Aerial-Robotics/GVINS) from T-RO2022
      
    - ⑩ VIW-Fusion: visual-inertial-wheel fusion odometry from [*open repository*](https://github.com/TouchDeeper/VIW-Fusion)2021
      
    - ⑪ VINS-GPS-Wheel: Visual-Inertial Odometry Coupled with Wheel Encoder and GNSS from [*open repository*](https://github.com/Wallong/VINS-GPS-Wheel)2021
      
    - ⑫ [Ground-fusion: A low-cost ground slam system robust to corner cases](https://arxiv.org/abs/2402.14308) and the [*open repository*](https://github.com/SJTU-ViSYS/Ground-Fusion) from ICRA2024
      
  - 💡 **LO system**:
    
    - ⑬ A-LOAM: Advanced implementation of [LOAM: Lidar odometry and mapping in real-time](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf), which from [*open repository*](https://github.com/HKUST-Aerial-Robotics/A-LOAM)2019
      
    - ⑭ [Loam livox: A fast, robust, high-precision LiDAR odometry and mapping package for LiDARs of small FoV](https://ieeexplore.ieee.org/abstract/document/9197440) and the [*open repository*](https://github.com/hku-mars/loam_livox) from ICRA2020
      
    - ⑮ CTLO: Continuous-Time LiDAR Odometry from [*open repository*](https://github.com/chengwei0427/CTLO)2023
      
    - ⑯ [Lego-loam: Lightweight and ground-optimized lidar odometry and mapping on variable terrain](https://ieeexplore.ieee.org/abstract/document/8594299) and the [*open repository*](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) from IROS2018
      
  - 💡 **LIO system**:
    
    - ⑰ [LIO-mapping: Tightly coupled 3d lidar inertial odometry and mapping](https://ieeexplore.ieee.org/abstract/document/8793511) and the [*open repository*](https://github.com/hyye/lio-mapping) from ICRA 2019
      
    - ⑱ [Lio-sam: Tightly-coupled lidar inertial odometry via smoothing and mapping](https://arxiv.org/abs/2007.00258) and the [*open repository*](https://github.com/TixiaoShan/LIO-SAM) from IROS2020
      
    - ⑲ [Lins: A lidar-inertial state estimator for robust and efficient navigation](https://arxiv.org/abs/1907.02233) and the [*open repository*](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) from ICRA2020
      
    - ⑳ [LiLi-OM: Towards high-performance solid-state-lidar-inertial odometry and mapping](https://arxiv.org/abs/2010.13150) and the [*open repository*](https://github.com/KIT-ISAS/lili-om) from RA-L2021
      
    - ㉑ LIO-Livox: A Robust LiDAR-Inertial Odometry for Livox LiDAR from [*open repository*](https://github.com/Livox-SDK/LIO-Livox)2021
      
    - ㉒ [Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels](https://ieeexplore.ieee.org/document/9718203) and the [*open repository*](https://github.com/gaoxiang12/faster-lio) from RA-L2022
      
    - ㉓ IESKF-LIO: reference to fast_lio1.0 from [*open repository*](https://github.com/Livox-SDK/LIO-Livox)2022
      
    - ㉔ [VoxelMap: Efficient and probabilistic adaptive voxel mapping method for LiDAR odometry](https://arxiv.org/abs/2109.07082) and the [*open repository*](https://github.com/hku-mars/VoxelMap) from RA-L2022
      
    - ㉕ [Fast-lio2: Fast direct lidar-inertial odometry](https://ieeexplore.ieee.org/abstract/document/9697912) and the [*open repository*](https://github.com/hku-mars/FAST_LIO) from T-RO2022
      
    - ㉖ [Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry](https://advanced.onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459) and the [*open repository*](https://github.com/hku-mars/Point-LIO) from AIS2023
      
    - ㉗ [LOG-LIO: A LiDAR-Inertial Odometry with Efficient Local Geometric Information Estimation](https://arxiv.org/abs/2307.09531) and the [*open repository*](https://github.com/tiev-tongji/LOG-LIO) from RA-L2023
      
    - ㉘ CT-LIO: Continuous-Time LiDAR-Inertial Odometry from [*open repository*](https://github.com/chengwei0427/ct-lio)2023
      
    - ㉙ [DLIO: Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction](https://arxiv.org/abs/2203.03749) and the [*open repository*](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) from ICRA2023
      
    - ㉚ HM-LIO: A Hash Map based LiDAR-Inertial Odometry from [*open repository*](https://github.com/chengwei0427/hm-lio)2023
      
    - ㉛ [MM-LINS: a Multi-Map LiDAR-Inertial System for Over-Degenerate Environments](https://ieeexplore.ieee.org/document/10557776) and the [*open repository*](https://github.com/lian-yue0515/MM-LINS) from T-IV2024
      
    - ㉜ [LIGO: Tightly Coupled LiDAR-Inertial-GNSS Odometry based on a Hierarchy Fusion Framework for Global Localization with Real-time Mapping](https://dl.acm.org/doi/10.1109/TRO.2025.3530298) and the [*open repository*](https://github.com/Joanna-HE/LIGO.) from T-RO2025
      
  - 💡 **LVIO system**:
    
    - ㉝ [LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping](https://arxiv.org/abs/2104.10831) and the [*open repository*](https://github.com/TixiaoShan/LVI-SAM) from ICRA2021
      
    - ㉞ [R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual tightly-coupled state Estimator and mapping](https://arxiv.org/abs/2102.12400) and the [*open repository*](https://github.com/hku-mars/r2live) from RA-L2021
      
    - ㉟ [R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package](https://arxiv.org/abs/2109.07982) and the [*open repository*](https://github.com/hku-mars/r3live) from ICRA2022
      
    - ㊱ [FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/abs/2203.00893) and the [*open repository*](https://github.com/hku-mars/FAST-LIVO) from IROS2022
    
    - ㊲ [Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline](https://arxiv.org/abs/2309.09808) and the [*open repository*](https://github.com/APRIL-ZJU/Coco-LIC) from RA-L2023
      
    - ㊳ [SR-LIVO: LiDAR-Inertial-Visual Odometry and Mapping with Sweep Reconstruction](https://arxiv.org/abs/2312.16800) and the [*open repository*](https://github.com/ZikangYuan/sr_livo) from RA-L2024
      
    - ㊴ [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/abs/2408.14035) and the [*open repository*](https://github.com/hku-mars/FAST-LIVO2) from T-RO2024
      
    - ㊵ Ground-Fusion++: Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework and the [*open repository*](https://github.com/sjtuyinjie/Ground-Fusion2)



