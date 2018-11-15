cmake_minimum_required(VERSION 2.8.3)
project(dmotion)
add_definitions(-std=c++11 -Wall -O2)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
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

# add_subdirectory(3rdparty)
add_subdirectory(src/IO)
add_subdirectory(src/Common)
add_subdirectory(src/InverseKinematics)
add_subdirectory(src/ThreeInterpolation)

add_subdirectory(example/test)
add_subdirectory(example/servo_read)
add_subdirectory(example/servo_write)


add_executable(test_servo example/test/test_servo.cpp)
target_link_libraries(test_servo ${catkin_LIBRARIES} dmotion_io dmotion_common dmotion_chazhi dmotion_kinematics)

add_executable(servo_read example/servo_read/servo_read.cpp)
target_link_libraries(servo_read ${catkin_LIBRARIES} dmotion_io dmotion_common)

add_executable(servo_write example/servo_write/servo_write.cpp)
target_link_libraries(servo_write ${catkin_LIBRARIES} dmotion_io dmotion_common)

add_executable(IOManager_read_write example/IOManager_read_write/IOManager_read_write.cpp)
target_link_libraries(IOManager_read_write ${catkin_LIBRARIES} dmotion_io dmotion_common)
