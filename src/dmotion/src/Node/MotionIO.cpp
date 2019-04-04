#include "dmotion/IOCommunication/IOCommunication.h"
int main(int argc, char ** argv){
    ros::init(argc, argv, "MotionIO");
    ros::NodeHandle n("~");
  //  Motion::IOManager3 io;
    parameters.init(&n);
    Motion::IOCommunication io_com(&n);
    //timer a;
    while(ros::ok())
    {
     //io_com.IOLoop();
     // a.tic();
     // io_com.Publisher();
     // a.SmartDelayMs(10.0);
    }
}
