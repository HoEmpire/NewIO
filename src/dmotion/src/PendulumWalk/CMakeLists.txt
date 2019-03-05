add_library(dmotion_PendulumWalk
   PendulumWalk.cpp
)
include_directories(../../include/dmotion/OneFootLanding)
target_link_libraries(dmotion_PendulumWalk ${catkin_LIBRARIES})
