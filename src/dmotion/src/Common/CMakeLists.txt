add_library(dmotion_common
    # Config/Config.cpp
    # Config/ConfigCollections.cpp
    # Config/ActionConfig.cpp

    Parameters.cpp
)
target_link_libraries(dmotion_common ${catkin_LIBRARIES})