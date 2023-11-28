#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void msgCallback(const sensor_msgs::LaserScanPtr& ls){
	float range = (ls->angle_max - ls->angle_min)/ls->angle_increment;
	for(int i = 0; i < range;i++){
		ROS_INFO("range %d is %f",i, ls->ranges[i]);
		ROS_INFO("Intensity %d is %f",i, ls->intensities[i]);
	}
	
}

int main(int argc, char **argv){
	ros::init(argc,argv, "laser_scanner");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan",1000,msgCallback);
	ros::spin();
	return 0;
}
