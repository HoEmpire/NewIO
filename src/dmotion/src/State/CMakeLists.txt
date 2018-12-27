add_library(dmotion_state
   imu_filter.cpp
   #RobotStatus.cpp
   stateless_orientation.cpp
)

target_link_libraries(dmotion_state ${catkin_LIBRARIES})
