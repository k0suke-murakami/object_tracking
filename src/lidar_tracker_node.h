
#include <ros/ros.h>

#include <tf/transform_listener.h>



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

};