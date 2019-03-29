cmake_minimum_required(VERSION 2.8.3)
project(dmotion)
add_definitions(-std=c++11 -Wall -O2)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    dmsgs
   )

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dmotion
#  CATKIN_DEPENDS dprocess
#  DEPENDS system_lib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

)

include_directories(
        ./include/dmotion/InverseKinematics
        ./include/dmotion/OneFootLanding
        ./include/dmotion/PendulumWalk
        ./include/dmotion/Utility
        ./include/dmotion/ThreeInterpolation
        ./include/dmotion/Polynomial
        ./include/dmotion/ForwardKinematics
        ./include/dmotion/IOCommunication
        ./include/dmotion/State
        ./include/Common
)

add_library(neo_dmotion_lib ./src/OneFootLanding/OneFootLanding.cpp
                            ./src/PendulumWalk/PendulumWalk.cpp
                            ./src/InverseKinematics/InverseKinematics.cpp
                            ./src/ThreeInterpolation/ThreeInterpolation.cpp
                            ./src/ForwardKinematics/ForwardKinematics.cpp
                            ./src/State/IMUFilter.cpp
                            ./src/State/StateManager.cpp
                            ./src/State/Odometer.cpp
                            ./src/Common/Parameters.cpp
                            ./src/IOCommunication/IOCommunication.cpp
)


add_subdirectory(src/IO)
target_link_libraries(neo_dmotion_lib dmotion_io)
#add_subdirectory(src/Common)

#add_executable(climb_read example/test/climb_read.cpp)
#target_link_libraries(climb_read ${catkin_LIBRARIES} dmotion_io dmotion_common dmotion_chazhi dmotion_kinematics)

#add_executable(climb_write example/test/climb_write.cpp)
#target_link_libraries(climb_write ${catkin_LIBRARIES} dmotion_io dmotion_common dmotion_chazhi dmotion_kinematics)

#add_executable(test_servo example/test/test_servo.cpp)
#target_link_libraries(test_servo ${catkin_LIBRARIES} dmotion_io dmotion_common)

#add_executable(servo_read example/servo_test/servo_read.cpp)
#target_link_libraries(servo_read ${catkin_LIBRARIES} dmotion_io dmotion_common)

#add_executable(servo_write example/servo_test/servo_write.cpp)
#target_link_libraries(servo_write ${catkin_LIBRARIES} dmotion_io dmotion_common)

#add_executable(IOManager_read_write example/IOManager_test/IOManager_read_write.cpp)
#target_link_libraries(IOManager_read_write ${catkin_LIBRARIES} dmotion_io dmotion_common)

#add_executable(power_test example/IOManager_test/power_test.cpp)
#target_link_libraries(power_test ${catkin_LIBRARIES} dmotion_io dmotion_common)

#add_executable(imu_test example/imu_test/imu_test.cpp)
#target_link_libraries(imu_test
#                      ${catkin_LIBRARIES}
#                      dmotion_io
#                      dmotion_common
#)

#add_executable(make_a_circle example/test/circle.cpp)
#target_link_libraries(make_a_circle
#                      ${catkin_LIBRARIES}
#                      dmotion_io
#                      dmotion_common
#                      neo_dmotion_lib)

#add_executable(state_test example/test/state_test.cpp)
#target_link_libraries(state_test
#                      ${catkin_LIBRARIES}
#                      dmotion_common
#                      dmotion_io
#                      neo_dmotion_lib
#)

#add_executable(pr_demo example/test/pr_demo.cpp)
#target_link_libraries(pr_demo
#                      ${catkin_LIBRARIES}
#                      dmotion_io
#                      dmotion_common
#                      dmotion_state
#                      neo_dmotion_lib
#                      )

#add_executable(imu_follow_test example/my_place/imu_follow_test.cpp)
#target_link_libraries(imu_follow_test
#                      ${catkin_LIBRARIES}
#                      dmotion_io
#                      dmotion_common
#                      dmotion_state
#                     neo_dmotion_lib
#)

add_executable(filter_test example/test/filter_test.cpp)
target_link_libraries(filter_test ${catkin_LIBRARIES} dmotion_io neo_dmotion_lib)

add_executable(fucking_test example/test/fucking_test.cpp)
target_link_libraries(fucking_test
                      ${catkin_LIBRARIES}
                      dmotion_io
                      neo_dmotion_lib
)

#add_executable(math_test example/test/math_test.cpp)
#target_link_libraries(math_test
#                      ${catkin_LIBRARIES}
#                      dmotion_common
#                      dmotion_io
#                      neo_dmotion_lib
#                     )

#add_executable(feetsensor_test example/feetsensor_test/feetsensor_test.cpp)
#target_link_libraries(feetsensor_test ${catkin_LIBRARIES} dmotion_io dmotion_common)

#add_executable(main src/Node/MotionIO.cpp)
#target_link_libraries(main
#                      ${catkin_LIBRARIES}
#                      dmotion_io
#                      neo_dmotion_lib
#)

#add_executable(dmotion_node src/main.cpp)
#target_link_libraries(dmotion_node
#                      com_dmotion_lib
#                      ${catkin_LIBRARIES}
#                      )
