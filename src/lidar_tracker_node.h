
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "ground_removal.h"
#include "component_clustering.h"
#include "box_fitting.h"
#include "imm_ukf_jpda.h"
#include "visualization.h"


class LidarTracker{
public:
	LidarTracker();
	void CloudCB(const sensor_msgs::PointCloud2ConstPtr &input);
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber sub_pointcloud_;
	ros::Publisher pub_elevated_pointcloud_;
	ros::Publisher pub_clustered_pointcloud_;
	ros::Publisher pub_jsk_bb_;
	ros::Publisher pub_arrow_;
	
	tf::TransformListener* tran_;
	int count_;

	GroundRemoval gr_;
	ComponentClustering cc_;
	BoxFitting bf_;
	Tracking tr_;
	Visualization vis_;
};