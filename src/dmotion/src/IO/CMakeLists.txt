add_library(dmotion_io
        dynamixel_sdk/packet_handler.cpp
        dynamixel_sdk/group_bulk_read.cpp
        dynamixel_sdk/group_bulk_write.cpp
        dynamixel_sdk/protocol1_packet_handler.cpp
        dynamixel_sdk/port_handler.cpp
        dynamixel_sdk/group_sync_read.cpp
        dynamixel_sdk/protocol2_packet_handler.cpp
        dynamixel_sdk/group_sync_write.cpp

        IOManager2.cpp
        IOManager3.cpp

        IMUReader.cpp
        ServoIO.cpp
        FeetSensorIO.cpp
)

target_link_libraries(dmotion_io ${catkin_LIBRARIES})
