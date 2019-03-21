#include "dmotion/IOCommunication/IOCommunication.h"
int main(int argc, char ** argv){
    ros::init(argc, argv, "MotionIO");
    ros::NodeHandle n("~");
  //  Motion::IOManager3 io;
    parameters.init(&n);
    Motion::IOCommunication io_com(&n);
//    Motion::IOManager3 io;
    while(ros::ok){
//	io.spinOnce();
    }
}
