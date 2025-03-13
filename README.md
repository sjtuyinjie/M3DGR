# M3DGR
M3DGR: A Multi-sensor, Multi-scenario and Massive-baseline SLAM Dataset for Ground Robots


## 🎯 Notice
## This paper is under review now. Full datasets and codes will be released upon paper acceptance.

## SENSOR SETUP
### Acquisition Platform
Physical drawings and schematics of the ground robot. (a) Side view of the robot. (b) Sensor arrangement on the top layer. (c) Sensor arrangement on the middle and bottom layers. All dimensions are provided in centimeters.

<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/car.jpg" width="800px">
</div>
<p align="center">Figure 1. The directions of the sensors are marked in different colors,red for X,green for Y and blue for Z.</p>

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
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/Traj_page-0001.jpg" width="600px">
<p align="center">Figure 2. All trajectories are mapped in different colors.</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()
</div>

### Visual Challenge
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/light01_processed.jpg" width="600px">
<p align="center">Figure 3. light01 Sequences</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()
</div>

### Wheel Slippage
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/grass.jpg" width="600px">
<p align="center">Figure 4. Dark01 Sequences</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()
</div>

### GNSS Denied
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/GPS_denied.jpg" width="600px">
<p align="center">Figure 5. GNSS_Denied01 Sequences</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()
</div>

### LiDAR Degeneration
<div align=center>
<img src="https://github.com/sjtuyinjie/M3DGR/blob/main/fig/corridor.jpg" width="600px">
<p align="center">Figure 6. corridor01 Sequences</p>

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag|GT
--|:--|:--:|--:|--:|--:|--:
01|2021-07-31|16.4g|172s|dark,around gate|[Rosbag]()|[GT]()
02|2021-07-31|27.3g|327s|dark,loop back|[Rosbag]()|[GT]()
03|2021-08-04|21.9g|283s|day|[Rosbag]()|[GT]()
</div>

## 🔥 SLAM Benchmark
- A list of tested cutting-edge methods on **M3DGR**🦄 dataset:
- 
  - 💡 **Measurement**:

    - ① Wheel Odom from [M3DGR](https://github.com/sjtuyinjie/M3DGR)'s wheel speedometer solution
      
    - ② GNSS SPP from [Ground-Fusion++](https://github.com/sjtuyinjie/Ground-Fusion2) solution
  
  - 💡 **VO system**:
  
    - ③ [PMLR2021] Tartanvo: A generalizable learning-based vo [[paper](https://arxiv.org/abs/2011.00359)][[code](https://github.com/castacks/tartanvo)] 
      
    - ④ [T-RO2017] Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras [[paper](https://arxiv.org/abs/1610.06475)][[code](https://github.com/raulmur/ORB_SLAM2)] 
      
  - 💡 **VIO system**:
    
    - ⑤ [T-RO2021] Orb-slam3: An accurate open-source library for visual, visual–inertial, and multimap slam [[paper](https://arxiv.org/abs/2007.11898)][[code](https://github.com/UZ-SLAMLab/ORB_SLAM3)] 
      
    - ⑥ [RA-L2022] DM-VIO: Delayed marginalization visual-inertial odometry [[paper](https://arxiv.org/abs/2201.04114)][[code](https://github.com/lukasvst/dm-vio)] 
      
    - ⑦ [T-RO2018] Vins-mono: A robust and versatile monocular visual-inertial state estimator [[paper](https://arxiv.org/abs/1708.03852)][[code](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)]
      
    - ⑧ [Sensors2019] VINS-RGBD: RGBD-inertial trajectory estimation and mapping for ground robots [[paper](https://www.mdpi.com/1424-8220/19/10/2251)][[code](https://github.com/STAR-Center/VINS-RGBD)]
      
    - ⑨ [T-RO2022] GVINS: Tightly coupled GNSS–visual–inertial fusion for smooth and consistent state estimation [[paper](https://arxiv.org/abs/2103.07899)][[code](https://github.com/HKUST-Aerial-Robotics/GVINS)]
      
    - ⑩ [2021] VIW-Fusion: visual-inertial-wheel fusion odometry [[code](https://github.com/TouchDeeper/VIW-Fusion)]
      
    - ⑪ [2021] VINS-GPS-Wheel: Visual-Inertial Odometry Coupled with Wheel Encoder and GNSS [[code](https://github.com/Wallong/VINS-GPS-Wheel)]
      
    - ⑫ [ICRA2024] Ground-fusion: A low-cost ground slam system robust to corner cases [[paper](https://arxiv.org/abs/2402.14308)][[code](https://github.com/SJTU-ViSYS/Ground-Fusion)]
      
  - 💡 **LO system**:
    
    - ⑬ [RSS2014] LOAM: Lidar odometry and mapping in real-time [[paper](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)][[code](https://github.com/HKUST-Aerial-Robotics/A-LOAM)]
      
    - ⑭ [ICRA2020] Loam livox: A fast, robust, high-precision LiDAR odometry and mapping package for LiDARs of small FoV [[paper](https://ieeexplore.ieee.org/abstract/document/9197440)][[code](https://github.com/hku-mars/loam_livox)] 
      
    - ⑮ [2023] CTLO: Continuous-Time LiDAR Odometry [[code](https://github.com/chengwei0427/CTLO)]
      
    - ⑯ [IROS2018] Lego-loam: Lightweight and ground-optimized lidar odometry and mapping on variable terrain [[paper](https://ieeexplore.ieee.org/abstract/document/8594299)][[code](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)]
      
  - 💡 **LIO system**:
    
    - ⑰ [ICRA 2019] LIO-mapping: Tightly coupled 3d lidar inertial odometry and mapping [[paper](https://ieeexplore.ieee.org/abstract/document/8793511)][[code](https://github.com/hyye/lio-mapping)]
      
    - ⑱ [IROS2020] Lio-sam: Tightly-coupled lidar inertial odometry via smoothing and mapping [[paper](https://arxiv.org/abs/2007.00258)][[code](https://github.com/TixiaoShan/LIO-SAM)]
      
    - ⑲ [ICRA2020] Lins: A lidar-inertial state estimator for robust and efficient navigation [[paper](https://arxiv.org/abs/1907.02233)][[code](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)] 
      
    - ⑳ [RA-L2021] LiLi-OM: Towards high-performance solid-state-lidar-inertial odometry and mapping [[paper](https://arxiv.org/abs/2010.13150)][[code](https://github.com/KIT-ISAS/lili-om)]
      
    - ㉑ [2021] LIO-Livox: A Robust LiDAR-Inertial Odometry for Livox LiDAR [[code](https://github.com/Livox-SDK/LIO-Livox)]
      
    - ㉒ [RA-L2022] Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels [[paper](https://ieeexplore.ieee.org/document/9718203)][[code](https://github.com/gaoxiang12/faster-lio)]
      
    - ㉓ [2022] IESKF-LIO: reference to fast_lio1.0 [[code](https://github.com/Livox-SDK/LIO-Livox)]
      
    - ㉔ [RA-L2022] VoxelMap: Efficient and probabilistic adaptive voxel mapping method for LiDAR odometry [[paper](https://arxiv.org/abs/2109.07082)][[code](https://github.com/hku-mars/VoxelMap)]
      
    - ㉕ [T-RO2022] Fast-lio2: Fast direct lidar-inertial odometry [[paper](https://ieeexplore.ieee.org/abstract/document/9697912)][[code](https://github.com/hku-mars/FAST_LIO)]
      
    - ㉖ [AIS2023] Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry [[paper](https://advanced.onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459)][[code](https://github.com/hku-mars/Point-LIO)]
      
    - ㉗ [RA-L2023] LOG-LIO: A LiDAR-Inertial Odometry with Efficient Local Geometric Information Estimation [[paper](https://arxiv.org/abs/2307.09531)][[code](https://github.com/tiev-tongji/LOG-LIO)]
      
    - ㉘ [2023] CT-LIO: Continuous-Time LiDAR-Inertial Odometry [[code](https://github.com/chengwei0427/ct-lio)]
      
    - ㉙ [ICRA2023] DLIO: Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction [[paper](https://arxiv.org/abs/2203.03749)][[code](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)]
      
    - ㉚ [2023] HM-LIO: A Hash Map based LiDAR-Inertial Odometry [[code](https://github.com/chengwei0427/hm-lio)]
      
    - ㉛ [T-IV2024] MM-LINS: a Multi-Map LiDAR-Inertial System for Over-Degenerate Environments [[paper](https://ieeexplore.ieee.org/document/10557776)][[code](https://github.com/lian-yue0515/MM-LINS)]
      
    - ㉜ [T-RO2025] LIGO: Tightly Coupled LiDAR-Inertial-GNSS Odometry based on a Hierarchy Fusion Framework for Global Localization with Real-time Mapping [[paper](https://dl.acm.org/doi/10.1109/TRO.2025.3530298)][[code](https://github.com/Joanna-HE/LIGO.)]
      
  - 💡 **LVIO system**:
    
    - ㉝ [ICRA2021] LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping [[paper](https://arxiv.org/abs/2104.10831)][[code](https://github.com/TixiaoShan/LVI-SAM)]
      
    - ㉞ [RA-L2021] R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual tightly-coupled state Estimator and mapping [[paper](https://arxiv.org/abs/2102.12400)][[code](https://github.com/hku-mars/r2live)]
      
    - ㉟ [ICRA2022] R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package [[paper](https://arxiv.org/abs/2109.07982)][[code](https://github.com/hku-mars/r3live)]
      
    - ㊱ [IROS2022] FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry [[paper](https://arxiv.org/abs/2203.00893)][[code](https://github.com/hku-mars/FAST-LIVO)]
    
    - ㊲ [RA-L2023] Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline [[paper](https://arxiv.org/abs/2309.09808)][[code](https://github.com/APRIL-ZJU/Coco-LIC)]
      
    - ㊳ [RA-L2024] SR-LIVO: LiDAR-Inertial-Visual Odometry and Mapping with Sweep Reconstruction [[paper](https://arxiv.org/abs/2312.16800)][[code](https://github.com/ZikangYuan/sr_livo)]
      
    - ㊴ [T-RO2024] FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry [[paper](https://arxiv.org/abs/2408.14035)][[code](https://github.com/hku-mars/FAST-LIVO2)]
      
    - ㊵ [2025] Ground-Fusion++: Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework [[paper]()][[code](https://github.com/sjtuyinjie/Ground-Fusion2)]



