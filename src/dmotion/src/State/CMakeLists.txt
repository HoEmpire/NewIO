add_library(dmotion_state
   IMUFilter.cpp
   StateManager.cpp
)

target_link_libraries(dmotion_state ${catkin_LIBRARIES})
