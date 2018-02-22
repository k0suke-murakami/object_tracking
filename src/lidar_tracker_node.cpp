// ROS related files include
#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>


// standard cpp library
#include <vector>
#include <iostream>
#include <math.h>


#include "lidar_tracker_node.h"
#include "ground_removal.h"
#include "component_clustering.h"
#include "box_fitting.h"
#include "imm_ukf_jpda.h"
#include "visualization.h"

using namespace std;
using namespace Eigen;
using namespace pcl;




LidarTracker::LidarTracker(){
	tf::TransformListener *lr (new  tf::TransformListener);
	tran_=lr;	
	
	sub_pointcloud_           = node_handle_.subscribe ("/input", 1, &LidarTracker::CloudCB, this);
	pub_elevated_pointcloud_  = node_handle_.advertise<sensor_msgs::PointCloud2> ("/elevated_pointcloud", 1);
	pub_clustered_pointcloud_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("/clustered_pointcloud", 1);
	pub_jsk_bb_               = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/vis_jsk_bb", 1);
	pub_arrow_                = node_handle_.advertise<visualization_msgs::Marker>( "/visualization_arrow", 1);
    count_ = 0;
}

void LidarTracker::CloudCB(const sensor_msgs::PointCloud2ConstPtr &input){
	count_ ++;
	cout << "Frame: "<<count_ << "----------------------------------------"<< endl;
	//Detection start------------------------------------------
	PointCloud<pcl::PointXYZ> cloud;
	PointCloud<pcl::PointXYZ>::Ptr elevatedCloud (new pcl::PointCloud<pcl::PointXYZ>());
	PointCloud<pcl::PointXYZ>::Ptr groundCloud   (new pcl::PointCloud<pcl::PointXYZ>());

	// Convert from ros msg to PCL::PointCloud data type
	fromROSMsg (*input, cloud);

	//process local tf point cloud, get elevatedCloud
	GroundRemoval gr;
	gr.groundRemove(cloud, elevatedCloud, groundCloud);

	// todo: using union find tree
	// get cluster with clusterID inside cartesianData
	ComponentClustering cc;
	vector<PointCloud<PointXYZRGB>>  clusteredPoints = cc.getComponentClustering(elevatedCloud);


	// Convert from PCL::PointCloud to ROS data type
	elevatedCloud->header.frame_id = cloud.header.frame_id;  // sync with pointcloud local tf
	sensor_msgs::PointCloud2 elevatedOutput;
	toROSMsg(*elevatedCloud, elevatedOutput);

	// Publish the elevated data.
	pub_elevated_pointcloud_.publish(elevatedOutput);

	//Publish the clustered data
	//error: do not work
	// cout<< clusteredPoints.size() << endl; 
	// for(int i = 0; i < clusteredPoints.size(); i++){
	// 	sensor_msgs::PointCloud2 clusteredOutput;
	// 	clusteredPoints[i].header.frame_id = cloud.header.frame_id;
	// 	cout << i<<"th "<<clusteredPoints[i].header.frame_id<<endl;
	// 	toROSMsg(clusteredPoints[i], clusteredOutput);
	// 	pub_clustered_pointcloud_.publish(clusteredOutput);
	// }


	// could be inappropriate to treat ros::Time with double
	// error: request for member ‘toSec’ in ‘cloud.pcl::PointCloud<pcl::PointXYZ>::header.pcl::PCLHeader::stamp’, which is of non-class type ‘uint64_t {aka long unsigned int}’
	// double timestamp  = cloud.header.stamp.toSec();
	double timestamp  = cloud.header.stamp;

	// fitting clusters by minAreaRectangle or L-shape fitting
	BoxFitting bf;
	vector<PointCloud<PointXYZ>> bBoxes = bf.getBBoxes(clusteredPoints, timestamp);

	// convert pointcloud from local to global for tracking-------------------------


	PointCloud<PointXYZ> newBox;
	for(int i = 0; i < bBoxes.size(); i++ ){
		bBoxes[i].header.frame_id = "velodyne";
		pcl_ros::transformPointCloud("/world", bBoxes[i], newBox, *tran_);
		bBoxes[i] = newBox;
	}

	//end converting----------------------------------------
	//Detection end------------------------------------------


	// tracking process start -------------------------
	Tracking tracker;
	PointCloud<PointXYZ> targetPoints;   // for visualizing tracking points
	vector<vector<double>> targetVandYaw;// for visualizing tracking arrows
	vector<int> trackManage;             // for checking tracking stage number
	vector<bool> isStaticVec;            // suggest object is static or not
	vector<bool> isVisVec;               // suggest each tracking point is stable enogh for visualizing or not
	vector<PointCloud<PointXYZ>> visBBs; // bounding box points for stable tracking points
	vector<int> visNumVec;               // for checking tracking stage number for each visBBs 
	tracker.immUkfPdaf(bBoxes, timestamp, targetPoints, targetVandYaw, trackManage, isStaticVec, isVisVec, visBBs, visNumVec);

	assert(targetPoints.size() == trackManage.size());
	assert(targetPoints.size() == targetVandYaw.size());
	// tracking process end ----------------------------

	// convert from global to local for visualization-------------------------
	for(int i = 0; i < visBBs.size(); i++ ){
		visBBs[i].header.frame_id = "world";
		pcl_ros::transformPointCloud("/velodyne", visBBs[i], newBox, *tran_);
		visBBs[i] = newBox;
	}
	// for debug
	for(int i = 0; i < bBoxes.size();i++){
		bBoxes[i].header.frame_id = "world";
		pcl_ros::transformPointCloud("/velodyne", bBoxes[i], newBox, *tran_);
		bBoxes[i] = newBox;
	}

	// end converting

	// converting from global to local tf for visualization
	PointCloud<PointXYZ> localPoints;
	targetPoints.header.frame_id = "world";
	pcl_ros::transformPointCloud("/velodyne", targetPoints, localPoints, *tran_);

	tf::StampedTransform transform;
	tran_->lookupTransform("/world", "/velodyne", ros::Time(0), transform);

	// get yaw angle from "world" to "velodyne" for direction(arrow) visualization
	tf::Matrix3x3 m(transform.getRotation());
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	//end converting to ego tf-------------------------


	Visualization vis;
	std_msgs::Header input_header = input->header;
	bool isStableBBox = true;
	vis.visualizeJSKBBox(input_header, pub_jsk_bb_, isStaticVec, 
	  visBBs, visNumVec, isStableBBox);
	// bool isStableBBox = false;
	// vis.visualizeJSKBBox(input_header, tracked_visbb_pub, isStaticVec, 
	//   bBoxes, visNumVec, isStableBBox);

	vis.visualizeArrow(trackManage, pub_arrow_, targetPoints, localPoints, 
	  isStaticVec, isVisVec,targetVandYaw, input_header, yaw );
}

int main(int argc, char **argv){
	ros::init(argc, argv, "object_tracking");
	LidarTracker node;
	ros::spin();

	return 0;
}
