#include "ros/ros.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  return 0;
}