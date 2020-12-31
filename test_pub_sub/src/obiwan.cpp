#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc,argv,"ObiWan");
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);

  while(ros::ok())
  {
    ROS_INFO("Help me Obi-Wan Kenobi, you'are my only hope");
  }

  return 0;
}