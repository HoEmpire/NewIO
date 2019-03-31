#include "dmotion/IOCommunication/IOCommunication.h"
int main(int argc, char ** argv){
    ros::init(argc, argv, "MotionIO");
    ros::NodeHandle n("~");

    parameters.init(&n);
    Motion::IOCommunication io_com(&n);
    while(ros::ok()){
      if(!ros::ok())
          std::abort();
    }
}
