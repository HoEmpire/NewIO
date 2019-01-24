add_library(dmotion_state
   #imu_filter.cpp
   imu_filter_new.cpp
   #RobotStatus.cpp
   StateManager.cpp
   stateless_orientation.cpp
)

target_link_libraries(dmotion_state ${catkin_LIBRARIES})
