#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

void newClustering(std::vector<std::vector<float>>& points,std::vector<int>& labels){
	int currentIteration = 0;
	const int radius = 1;
	labels[0] = 1;//Cause the first point is always gonna be on its own cluster
	int currentLabel = 1;
	float distance;
	std::vector<std::vector<float>> centers;
	centers.push_back(points[0]);
	bool found = false;
	while(true){
		for(int i=1;i<points.size();i++){
			distance += (points[i][0]-centers[currentLabel-1][0])*(points[i][0]-centers[currentLabel-1][0]);
			distance += (points[i][1]-centers[currentLabel-1][1])*(points[i][1]-centers[currentLabel-1][1]);
			distance = sqrt(distance);
			if(distance <= radius){
				labels[i] = currentLabel;
			}
			distance=0;
		}
		for(int i=0;i<labels.size();i++){
			if(labels[i]==0){
				centers.push_back(points[i]);
				currentLabel++;
				found = true;
				break;
			}
		}
		if(!found)
			break;
		found=false;
	}
}

std::vector<std::vector<float>> getCenters(std::vector<std::vector<float>> points, std::vector<int> labels){
	std::vector<std::vector<float>> centers;
	int numElements = 0;
	int currentLabel = 1;
	int maxLabel = 1;
	for(int i =0;i<labels.size();i++){
		if(labels[i]>=maxLabel)
			maxLabel=labels[i];
	}
	std::vector<float> temp_vector(2);
	float temp_distance_x=0;
	float temp_distance_y=0;
	while(currentLabel-1<maxLabel){
		for(int i=0;i<labels.size();i++){
			if(labels[i]==currentLabel){
				temp_distance_x += points[i][0];
				temp_distance_y += points[i][1];
				ROS_INFO("temp_distance_y: %f",temp_distance_y);
				numElements++;
			}
		}
		temp_distance_x /= numElements;
		temp_distance_y = temp_distance_y/numElements;
		temp_vector[0]=temp_distance_x;
		temp_vector[1]=temp_distance_y;
		centers.push_back(temp_vector);
		temp_distance_y = 0;
		temp_distance_x = 0;
		numElements = 0;
		currentLabel++;
	}
	return centers;
}

void msgCallback(const sensor_msgs::LaserScanPtr& ls){
	float range = (ls->angle_max - ls->angle_min)/ls->angle_increment;
	std::vector<std::vector<float>> points;
	std::vector<int> labels;
	std::vector<std::vector<float>> centers;
	ROS_INFO("min: %f; max: %f; increment: %f", ls->angle_min,ls->angle_max,ls->angle_increment);
	ROS_INFO("range max: %f; range min: %f",ls->range_max,ls->range_min);
	for(int i = 0; i < range;i++){
		if(ls->ranges[i] != 0 && ls->ranges[i] <=ls->range_max ){
			std::vector<float> temp_points(2);
			temp_points[0]=ls->ranges[i]*cos(i*ls->angle_increment);
			temp_points[1]=ls->ranges[i]*sin(i*ls->angle_increment);
			points.push_back(temp_points);
			labels.push_back(i);
		}
	}
	for(int i =0; i<centers.size();i++)
		centers[i] = points[0];
	std::vector<int> newLabels(points.size());
	newClustering(points,newLabels);
	std::vector<std::vector<float>> finalPoints = getCenters(points,newLabels);
	//for(int i = 0; i<newLabels.size();i++)
		//ROS_INFO("The point %d [%f,%f] has label %d",i,points[i][0],points[i][1],newLabels[i]);
	for(int i=0;i<finalPoints.size();i++)
		ROS_INFO("The person %d is in position [%f,%f]",i+1,finalPoints[i][0],finalPoints[i][1]);
}

int main(int argc, char **argv){
	ros::init(argc,argv, "laser_scanner");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan",1000,msgCallback);
	ros::spin();
	return 0;
}
