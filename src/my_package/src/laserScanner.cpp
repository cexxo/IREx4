#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

/*void clustering(std::vector<std::vector<float>> points, std::vector<std::vector<float>>& centers,int k,int maxIter,std::vector<std::vector<float>>& reversePoints){
	float tollerance = 0.05;//Tollerance of the neighborhood we want to 'cluster'
	int counter = 1;
	bool allInRange = true;
	int loopCounter = 0;
	while(counter<k){
		for(int i = 0; i< centers.size();i++){
			for(int j =1;j<points.size();j++){
				if(points[j][0] <= centers[i][0]-tollerance && counter < k){
					centers[counter] = points[j];
					ROS_INFO("iteration %d counter %d",i,counter);
					counter++;
				}
			}
		}
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
}*/

void newClustering(std::vector<std::vector<float>>& points,std::vector<int>& labels,int maxCluster){
	int currentIteration = 0;
	const int radius = 1;
	labels[0] = 1;
	int currentLabel = 1;
	float distance;
	std::vector<std::vector<float>> centers;
	centers.push_back(points[0]);
	bool found = false;
	while(currentLabel-1 < maxCluster){
		for(int i=1;i<points.size();i++){
			distance += (points[i][0]-centers[currentLabel-1][0])*(points[i][0]-centers[currentLabel-1][0]);
			distance += (points[i][1]-centers[currentLabel-1][1])*(points[i][1]-centers[currentLabel-1][1]);
			distance = sqrt(distance);
			//ROS_INFO("Distance from center %d: [%f]",currentLabel,distance);
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
	float temp_distance_x;
	float temp_distance_y;
	while(currentLabel-1<maxLabel){
		for(int i=0;i<labels.size();i++){
			if(labels[i]==currentLabel){
				temp_distance_x += points[i][0];
				temp_distance_y += points[i][1];
				numElements++;
			}
		}
		temp_distance_x /= numElements;
		temp_distance_y /= numElements;
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
	std::vector<std::vector<float>> reversedPoints(points.size());
	for(int i =0; i<centers.size();i++)
		centers[i] = points[0];
	//ROS_INFO("BEFORE CLUSTERING");
	//clustering(points,centers,6,5,reversedPoints);
	std::vector<int> newLabels(points.size());
	newClustering(points,newLabels,100);
	std::vector<std::vector<float>> finalPoints = getCenters(points,newLabels);
	//ROS_INFO("AFTER CLUSTERING");
	//for(int i=0; i<points.size();i++)
		//ROS_INFO("point %d is %f,%f",labels[i],points[i][0],points[i][1]);
	//for(int i = 0; i<newLabels.size();i++)
		//ROS_INFO("The point %d [%f,%f] has label %d",i,points[i][0],points[i][1],newLabels[i]);
	for(int i=0;i<finalPoints.size();i++)
		ROS_INFO("The person %d is in position [%f,%f]",i+1,finalPoints[i][0],finalPoints[i][1]);
		//ROS_INFO("Center %d-th: [%f,%f]",i,centers[i][0],centers[i][1]);
	//TODO:
	//Clustering from the centers we have found so far
	//Merge the two closest clusters in order to detect the person
	//Find the centroids of the new clusters in order to determine as precise as possible the new person's position
	//Check if it works for the whole bag
	//ros::shutdown();//Just cause i don't want to have a spam of points for now...
}

int main(int argc, char **argv){
	ros::init(argc,argv, "laser_scanner");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan",1000,msgCallback);
	ros::spin();
	return 0;
}
