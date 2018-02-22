#ifndef OBJECT_TRACKING_VISUALIZATION_H
#define OBJECT_TRACKING_VISUALIZATION_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/io/io.h>

#include <vector>


using namespace std;
using namespace pcl;
using namespace Eigen;

class Visualization{
private:
	enum trackState_ { enumStatic = 99, enumDynamic = 5};
	VectorXd getCpFromBbox(const PointCloud<PointXYZ> bBox);

public:
	void visualizeJSKBBox(const std_msgs::Header input_header, const ros::Publisher tracked_visbb_pub,
	 const vector<bool> isStaticVec, 
	const vector<PointCloud<PointXYZ>> visBBs, const vector<int> visNumVec, const bool isStableBBox);

	void visualizeArrow(const vector<int> trackManage, const ros::Publisher vis_arrow, 
		const PointCloud<PointXYZ> targetPoints, const PointCloud<PointXYZ> localPoints,
		const vector<bool> isStaticVec, const vector<bool> isVisVec,
		const vector<vector<double>> targetVandYaw, const std_msgs::Header input_header, const double egoYaw );
};

#endif //OBJECT_TRACKING_VISUALIZATION_H