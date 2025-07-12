## fisheye:
Examples_old/Monocular-Inertial/mono_inertial_tum_vi_old Vocabulary/ORBvoc.txt Examples_old/Monocular-Inertial/TUM-VI.yaml Examples_old/Monocular-Inertial/TUM_Image/dataset-corridor4_512_16/mav0/cam0/data Examples_old/Monocular-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt Examples_old/Monocular-Inertial/TUM_IMU/dataset-corridor4_512.txt tum_corridor4_512

Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM-VI.yaml Examples/Monocular-Inertial/TUM_Image/dataset-corridor4_512_16/mav0/cam0/data Examples_old/Monocular-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt Examples_old/Monocular-Inertial/TUM_IMU/dataset-corridor4_512.txt tum_corridor4_512

## ailab:
Examples_old/Monocular-Inertial/mono_inertial_tum_vi_old Vocabulary/ORBvoc.txt Examples_old/Monocular-Inertial/fisheye.yaml Examples_old/Monocular-Inertial/ailab_image Examples_old/Monocular-Inertial/ailab_timestamp/timestamps.txt Examples_old/Monocular-Inertial/ailab_IMU/Imu_Data.txt ailab

Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples_old/Monocular-Inertial/fisheye.yaml Examples_old/Monocular-Inertial/ailab_image Examples_old/Monocular-Inertial/ailab_timestamp/timestamps.txt Examples_old/Monocular-Inertial/ailab_IMU/Imu_Data.txt ailab

# IMU head:
x, 温度(摄氏度), a_x(g), a_y(g), a_z(g), w_x(度/秒), w_y((度/秒)), w_z(度/秒)

# 上海位于北纬31°12′，重力加速度g的精确值为9.7940 m/s²。
imuGravity: 9.7940

鱼眼那个包帧率是40帧，IMU数据速率500Hz
FOV对角线180度
就严格是1秒对应500行数据和40帧
