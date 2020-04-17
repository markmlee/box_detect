cmake_minimum_required(VERSION 2.8.3)
project(box_detect)

set(OpenCV_DIR "/usr/share/OpenCV/")
find_package( OpenCV 3 REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  OpenCV
  image_transport
  cv_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
  object_detector
  tf
  
  
)

#define message

#catkin package option
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME} src/liftBox_node.cpp)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${OpenCV_LIBS}
#)


add_executable(${PROJECT_NAME}_tuning src/boxDetect_tuning.cpp)
add_dependencies(${PROJECT_NAME}_tuning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_tuning
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_pointcloud src/boxpointcloud_node.cpp)
add_dependencies(${PROJECT_NAME}_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pointcloud
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCV_LIBS}
)


add_executable(${PROJECT_NAME}_pose src/pose_publisher.cpp)
add_dependencies(${PROJECT_NAME}_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pose
    ${catkin_LIBRARIES}

)

add_executable(${PROJECT_NAME}_tf src/tf_publisher.cpp)
add_dependencies(${PROJECT_NAME}_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_tf
    ${catkin_LIBRARIES}

)

#####################################
# detecting  node for vitamin water  #
#####################################

add_executable(${PROJECT_NAME}_vitaminwater src/vitaminwater_pose.cpp)
add_dependencies(${PROJECT_NAME}_vitaminwater ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_vitaminwater
    ${catkin_LIBRARIES}

)

#####################################
# detecting  node for vitamin water  #
#####################################
add_executable(green_plane_pose src/green_plane_pose.cpp)
add_dependencies(green_plane_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(green_plane_pose
    ${catkin_LIBRARIES}

)

add_executable(local_plane_pose src/local_plane_pose.cpp)
add_dependencies(local_plane_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(local_plane_pose
    ${catkin_LIBRARIES}

)


add_executable(publish_upon_key_input src/publish_upon_key_input.cpp)
add_dependencies(publish_upon_key_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(publish_upon_key_input
    ${catkin_LIBRARIES}

)

#####################################
# tracking node  #
#####################################

add_executable(2d_tracking src/2d_tracking.cpp)
add_dependencies(2d_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(2d_tracking
    ${catkin_LIBRARIES}

)




#####################################
# export data node  #
#####################################


add_executable(${PROJECT_NAME}_exportCSV src/exportCSV.cpp)
add_dependencies(${PROJECT_NAME}_exportCSV ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_exportCSV
    ${catkin_LIBRARIES}

)




