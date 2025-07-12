<a href="#"><img src="https://img.shields.io/badge/c++-%2300599C.svg?style=flat&logo=c%2B%2B&logoColor=white"></img></a>
  <a href="#"><img src="https://img.shields.io/github/stars/chengwei0427/ESKF_LIO"></img></a>
  <a href="#"><img src="https://img.shields.io/github/forks/chengwei0427/ESKF_LIO"></img></a>
  <a href="#"><img src="https://img.shields.io/github/repo-size/chengwei0427/ESKF_LIO"></img></a>
  <a href="https://github.com/chengwei0427/ESKF_LIO/issues"><img src="https://img.shields.io/github/issues/chengwei0427/ESKF_LIO"></img></a>
  <a href="https://github.com/chengwei0427/ESKF_LIO/graphs/contributors"><img src="https://img.shields.io/github/contributors/chengwei0427/ESKF_LIO?color=blue"></img></a>

## iESKF-lio

This repository is a modified LiDAR-inertial odometry system. The system is developed based on the open-source odometry framework [**FAST-LIO**](https://github.com/XW-HKU/fast_lio) to get the odometry information. And the feature extract moudle is implemented based on [**LIO-SAM**](https://github.com/TixiaoShan/LIO-SAM) .

## Modification

  - Feature extract moudle is implemented based on lio-sam, this moudle support **multiple lidar types**(such as velodyne,ouster,robosense, livox etc.);
  - laser mapping moudle is implemented base on **fast-lio 1.0**, Use Eigen matrix instead of **IKFom**;
  - use **ikdtree** manage the map;
  - the new laser mapping moudle support **multiple lidar types**: both traditional spinning lidar (velodyne, ouster, robsense etc.) and solid-state lidar(livox);
  - add online extrinsic calib as fast-lio2
  - add new lidar process moudle, this moudle support **process multi-lidar (as one Lidar)**;
  - add **greedy based feature select**(reference to M-LOAM);(code upload in a new branck(gfs) 2022-11-23)

##  DEMO
**[update 2022-11-08]**

<p align='center'>
    <img src="./doc/gfs.png" alt="drawing" width="1000"/>
</p>

[**VIDEO**: IESKF-LIO+Greedy based Feature Select](https://www.bilibili.com/video/BV1o14y1V7wS/?spm_id_from=333.999.0.0&vd_source=438f630fe29bd5049b24c7f05b1bcaa3)

**[update 2022-08-05]**

<p align="center">
  <img width="1000pix" src="./doc/UrbanNav-HK-TST-20210517.gif">
</p>
<p align='center'>
    <img src="./doc/time.png" alt="drawing" width="400"/>
    <img src="./doc/TST_map.png" alt="drawing" width="400"/>
</p>

[UrbanNav-HK-TST-20210517 test video](https://www.bilibili.com/video/BV1pS4y147Ta?spm_id_from=333.999.0.0&vd_source=438f630fe29bd5049b24c7f05b1bcaa3)

<p align='center'>
    <img src="./doc/calib_test.png" alt="drawing" width="1000"/>
</p>

[Ouster32 test video](https://www.bilibili.com/video/BV1NG411h7wE?spm_id_from=333.999.0.0&vd_source=438f630fe29bd5049b24c7f05b1bcaa3)

<p align='center'>
    <img src="./doc/test.png" alt="drawing" width="1000"/>
</p>

[hkust_20201105full test video](https://www.bilibili.com/video/BV1FG4y1v7co?spm_id_from=333.999.0.0&vd_source=438f630fe29bd5049b24c7f05b1bcaa3)

## Dependency
Follow the [fast_lio](https://github.com/hku-mars/FAST_LIO)

## Build

Use the following commands to download and compile the package.

```
cd ~/${yourdir}/src
git clone https://github.com/chengwei0427/ESKF_LIO.git
cd ..
source devel/setup.bash
catkin_make 
```

## How to run
1. change the params in **config/feat.yaml**;
2. test direct/feature based eskf-lio use **run.launch**;
3. test **multi-lidar** slam use **run_multi.launch**;

## other notes
1. you cloud test the **multi-lidar** with the **UrbanNavDataset**;
2. the **auxiliary** lidar **only support velodyne** current, the **primary** lidar support multi-type lidars(such as velodyne,ouster,robosense, livox etc.);

## TODO

  - [ ] add ivox 
  - [x] add extrinsic parameter calibration
  - [ ] compare with FAST-LIO2
  - [x] add test video
  - [x] support multi-lidar 
  - [ ] add greedy base feature select

<!-- CONTRIBUTING -->
## **Contributing**

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<!-- CONTACT -->
## **Contact**

**Author: chengwei zhao<br />
Maintainer: chengwei zhao(960732491@qq.com)<br />
Project Link: [ESKF_LIO](https://github.com/chengwei0427/ESKF_LIO)<br />**

## Acknowledgments
Thanks for LOAM, [FAST_LIO](https://github.com/hku-mars/FAST_LIO) ,[LIO_SAM](https://github.com/TixiaoShan/LIO-SAM), [M-LOAM](https://github.com/gogojjh/M-LOAM) and [UrbanNavDataset](https://github.com/IPNL-POLYU/UrbanNavDataset).

![Star History Chart](https://api.star-history.com/svg?repos=chengwei0427/ESKF_LIO&type=Date)
