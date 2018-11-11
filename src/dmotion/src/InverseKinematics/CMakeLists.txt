add_library(dmotion_kinematics
   InverseKinematics.cpp
)

target_link_libraries(dmotion_kinematics ${catkin_LIBRARIES})
