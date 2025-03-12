# M3DGR
M3DGR: A Multi-sensor, Multi-scenario and Massive-baseline SLAM Dataset for Ground Robots


## 🎯 Notice
This paper is under review now. Full datasets and codes will be released upon paper acceptance.

## SENSOR SETUP
### Acquisition Platform


### Sensor parameters

All the sensors and track devices and their most important parameters are listed as below:

* **LIDAR** Velodyne VLP-32C, 360 Horizontal Field of View (FOV),-30 to +10 vertical FOV,10Hz,Max Range 200 m,Range Resolution 3 cm, Horizontal Angular Resolution 0.2°.  

* **RGB Camera** FLIR Pointgrey CM3-U3-13Y3C-CS,fish-eye lens,1280*1024,190 HFOV,190 V-FOV, 15 Hz  
* **GNSS** Ublox M8T, GPS/BeiDou, 1Hz  
* **Infrared Camera**,PLUG 617,640*512,90.2 H-FOV,70.6 V-FOV,25Hz;  
* **V-I Sensor**,Realsense d435i,RGB/Depth 640*480,69H-FOV,42.5V-FOV,15Hz;IMU 6-axix, 200Hz  
* **Event Camera** Inivation DVXplorer, 640*480,15Hz;  
* **IMU**,Handsfree A9,9-axis,150Hz;  
* **GNSS-IMU** Xsens Mti 680G. GNSS-RTK,localization precision 2cm,100Hz;IMU 9-axis,100 Hz;  
* **Laser Scanner** Leica MS60, localization 1mm+1.5ppm  
* **Motion-capture System** Vicon Vero 2.2, localization accuracy 1mm, 50 Hz;

The rostopics of our rosbag sequences are listed as follows:

* LIDAR: `/velodyne_points` 

* RGB Camera: 
`/camera/left/image_raw/compressed `,  
`/camera/right/image_raw/compressed `,  
`/camera/third/image_raw/compressed `,  
`/camera/fourth/image_raw/compressed `,  
`/camera/fifth/image_raw/compressed `,  
`/camera/sixth/image_raw/compressed `,  
`/camera/head/image_raw/compressed `  
* GNSS Ublox M8T:  
`/ublox/aidalm `,  
`/ublox/aideph `,  
`/ublox/fix `,  
`/ublox/fix_velocity `,  
`/ublox/monhw `,  
`/ublox/navclock `,  
`/ublox/navpvt `,  
`/ublox/navsat `,  
`/ublox/navstatus `,  
`/ublox/rxmraw `  


* Infrared Camera:`/thermal_image_raw ` 
* V-I Sensor:  
`/camera/color/image_raw/compressed `,  
`/camera/imu`
* Event Camera:  
`/dvs/events`,  
`/dvs_rendering/compressed`
* IMU: `/handsfree/imu `
 

## DATASET SEQUENCES

### Visual Challenge


### Wheel Slippage


### GNSS Denied



### LiDAR Degeneration


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



