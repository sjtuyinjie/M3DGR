cmake_minimum_required(VERSION 3.0.2)
project(imu_transformer)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  topic_tools
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_transformer_nodelet
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros geometry_msgs topic_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(imu_transformer_nodelet src/imu_transformer_nodelet.cpp)
target_link_libraries(imu_transformer_nodelet ${catkin_LIBRARIES})

add_executable(imu_transformer_node src/imu_transformer_node.cpp)
target_link_libraries(imu_transformer_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)

  find_package(tf2_geometry_msgs REQUIRED)
  include_directories(${tf2_geometry_msgs_INCLUDE_DIRS})
  catkin_add_gtest(test_imu_transforms test/test_imu_transforms.cpp)
  target_link_libraries(test_imu_transforms ${catkin_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})
endif()

install(TARGETS imu_transformer_node imu_transformer_nodelet
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
