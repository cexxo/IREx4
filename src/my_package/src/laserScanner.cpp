#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

void clustering(std::vector<std::vector<float>> points, std::vector<std::vector<float>>& centers,int k,int maxIter){
	float tollerance = 0.05;//Tollerance of the neighborhood we want to 'cluster'
	int counter = 1;
	bool allInRange = true;
	int loopCounter = 0;
	while(counter<k){
		/*for(int i = 0; i< centers.size();i++){
			for(int j =1;j<points.size();j++){
				if(points[j][0] <= centers[i][0]-tollerance && counter < k){
					centers[counter] = points[j];
					ROS_INFO("iteration %d counter %d",i,counter);
					counter++;
				}
			}
		}*/
		for(int i = 1; i<points.size();i++){
			//ROS_INFO("FIRST LOOP");
			for(int j = 0; j< centers.size();j++){
				//ROS_INFO("SECOND LOOP");
				if(((points[i][0] >= centers[j][0]-tollerance && points[i][0] <= centers[j][0]+tollerance) || (points[i][1] >= centers[j][1]-tollerance && points[i][1] <= centers[j][1]+tollerance))&& counter <k){
					//ROS_INFO("CONDITION NOT SATISFIED");
					allInRange = false;
				}
			}
			if(allInRange && counter < k){
				centers[counter]=points[i];
				//ROS_INFO("iteration %d counter %d",i,counter);
				counter++;
			}
			else if(counter == k)
				break;
			else
				allInRange = true;
		}
		if(counter >= 3){
			loopCounter++;
			tollerance -=0.01;
		}
		if(loopCounter > maxIter)
			break;
	}
}
void msgCallback(const sensor_msgs::LaserScanPtr& ls){
	float range = (ls->angle_max - ls->angle_min)/ls->angle_increment;
	std::vector<std::vector<float>> points;
	std::vector<int> labels;
	std::vector<std::vector<float>> centers(6);
	ROS_INFO("min: %f; max: %f; increment: %f", ls->angle_min,ls->angle_max,ls->angle_increment);
	for(int i = 0; i < range;i++){
		//ROS_INFO("range %d is %f",i, ls->ranges[i]);
		//ROS_INFO("Intensity %d is %f",i, ls->intensities[i]);
		if(ls->ranges[i] != 0 && ls->ranges[i] <=20 ){
			std::vector<float> temp_points(2);
			temp_points[0]=ls->ranges[i]*cos(i*ls->angle_increment);
			temp_points[1]=ls->ranges[i]*sin(i*ls->angle_increment);
			points.push_back(temp_points);
			labels.push_back(i);
		}
	}
	for(int i =0; i<centers.size();i++)
		centers[i] = points[0];
	//ROS_INFO("BEFORE CLUSTERING");
	clustering(points,centers,6,5);
	//ROS_INFO("AFTER CLUSTERING");
	//for(int i=0; i<points.size();i++)
		//ROS_INFO("point %d is %f,%f",labels[i],points[i][0],points[i][1]);
	for(int i = 0; i<centers.size();i++)
		ROS_INFO("Center %d-th: [%f,%f]",i,centers[i][0],centers[i][1]);
	//TODO:
	//Clustering from the centers we have found so far
	//Merge the two closest clusters in order to detect the person
	//Find the centroids of the new clusters in order to determine as precise as possible the new person's position
	//Check if it works for the whole bag
	ros::shutdown();//Just cause i don't want to have a spam of points for now...
}

int main(int argc, char **argv){
	ros::init(argc,argv, "laser_scanner");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan",1000,msgCallback);
	ros::spin();
	return 0;
}
