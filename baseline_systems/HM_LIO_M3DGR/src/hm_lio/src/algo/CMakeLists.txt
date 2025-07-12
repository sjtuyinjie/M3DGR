

add_library(${PROJECT_NAME}.algo
        static_imu_init.cc
        IMU_Processing.cpp
          )

target_link_libraries(${PROJECT_NAME}.algo
        glog gflags ${PROJECT_NAME}.common
        )

