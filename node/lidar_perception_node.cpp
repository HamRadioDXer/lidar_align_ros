
#include <ros/ros.h>
#include "lidar_perception/lidar_perception.h"


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "lidar_perception_node");

    LidarPerception lp;
    lp.init();
    lp.doListening();

	return 0;
}