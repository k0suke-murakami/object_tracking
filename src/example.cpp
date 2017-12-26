#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/segmentation/sac_segmentation.h>

#include <vector>
#include <iostream>
#include <math.h>
#include <pcl/io/pcd_io.h>

#include <visualization_msgs/Marker.h>

#include "ground_removal.h"
#include "component_clustering.h"
#include "box_fitting.h"

// #include "ukf.h"
#include "imm_ukf_jpda.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

int counta = 0;

ros::Publisher pub;

ros::Publisher vis_pub;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr input){
  // Do data processing here...

  PointCloud<pcl::PointXYZ> cloud;
  PointCloud<pcl::PointXYZ>::Ptr elevatedCloud (new pcl::PointCloud<pcl::PointXYZ>());
  PointCloud<pcl::PointXYZ>::Ptr groundCloud   (new pcl::PointCloud<pcl::PointXYZ>());

  // Convert from ros msg to PCL::PointCloud data type
  fromROSMsg (*input, cloud);
  // cout << "time: " << cloud.header<< endl;

  //start processing pcl::pointcloud
  groundRemove(cloud, elevatedCloud, groundCloud);

  //todo: using union find tree
  // make array with initialized 0
  int numCluster = 0; // global variable?
  array<array<int, numGrid>, numGrid> cartesianData{};
  componentClustering(elevatedCloud, cartesianData, numCluster);

  // for visualization
  PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  makeClusteredCloud(elevatedCloud, cartesianData, clusteredCloud);

  // Convert from PCL::PointCloud to ROS data type
  clusteredCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  elevatedCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  sensor_msgs::PointCloud2 output;
  // toROSMsg(*clusteredCloud, output);
  toROSMsg(*elevatedCloud, output);
  
  // Publish the data.
  pub.publish(output);
  counta ++;
  cout << "Frame: "<<counta << "----------------------------------------"<< endl;


  // bounding box fitting bBoxes[numCluster][6](cx, cy, cz, width, height, depth)
  vector<PointCloud<PointXYZ>> bBoxes = boxFitting(elevatedCloud, cartesianData, numCluster);
  // targets[numTragets][2](x, y)
  double timestamp = cloud.header.stamp;
  vector<PointXY> targetPoints;
  vector<int> trackManage;
  vector<bool> isStaticVec;
  immUkfJpdaf(bBoxes, timestamp, targetPoints, trackManage, isStaticVec);
  // cout << "trackManage size: "<<trackManage.size()<<endl;
  // cout << "number of tracking object: "<<countTrack<<endl;
  // cout << "targetpoints size: "<<targetPoints.size()<<endl;
  // cout << "content: "<<trackManage[0]<<endl;
  // trackManage[0] = 5;

  assert(targetPoints.size() == trackManage.size());
  
  // tracking poiints visualizing start---------------------------------------------
  visualization_msgs::Marker pointsY, pointsG, pointsR, pointsB;
  pointsY.header.frame_id = pointsG.header.frame_id = pointsR.header.frame_id = pointsB.header.frame_id = "velo_link";
  pointsY.header.stamp= pointsG.header.stamp= pointsR.header.stamp =pointsB.header.stamp = ros::Time::now();
  pointsY.ns= pointsG.ns = pointsR.ns =pointsB.ns=  "points";
  pointsY.action = pointsG.action = pointsR.action = pointsB.action = visualization_msgs::Marker::ADD;
  pointsY.pose.orientation.w = pointsG.pose.orientation.w  = pointsR.pose.orientation.w =pointsB.pose.orientation.w= 1.0;

  pointsY.id = 1;
  pointsG.id = 2;
  pointsR.id = 3;
  pointsB.id = 4;
  pointsY.type = pointsG.type = pointsR.type = pointsB.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  pointsY.scale.x =pointsG.scale.x =pointsR.scale.x = pointsB.scale.x=0.5;
  pointsY.scale.y =pointsG.scale.y =pointsR.scale.y = pointsB.scale.y = 0.5;

  // yellow
  pointsY.color.r = 1.0f;
  pointsY.color.g = 1.0f;
  pointsY.color.b = 0.0f;
  pointsY.color.a = 1.0;

  // green
  pointsG.color.g = 1.0f;
  pointsG.color.a = 1.0;

  // red
  pointsR.color.r = 1.0;
  pointsR.color.a = 1.0;

  // blue 
  pointsB.color.b = 1.0;
  pointsB.color.a = 1.0;

  for(int i = 0; i < targetPoints.size(); i++){
    if(trackManage[i] == 0) continue;
    geometry_msgs::Point p;
    p.x = targetPoints[i].x;
    p.y = targetPoints[i].y;
    p.z = -1.73/2;
    // cout << trackManage[i] << endl;
    if(isStaticVec[i] == true){
      pointsB.points.push_back(p); 
    }
    else if(trackManage[i] < 5 ){
      pointsY.points.push_back(p);
    }
    else if(trackManage[i] == 5){
      pointsG.points.push_back(p);
    }
    else if(trackManage[i] > 5){
      pointsR.points.push_back(p); 
    }
  }
  vis_pub.publish(pointsY);
  // cout << "pointsG" << pointsG.points[0].x << " "<< pointsG.points[0].y << endl;
  vis_pub.publish(pointsG);
  vis_pub.publish(pointsR);
  vis_pub.publish(pointsB);
  // tracking poiints visualizing end---------------------------------------------

  // bounding box visualizing start---------------------------------------------
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "velo_link";
  line_list.header.stamp = ros::Time::now();
  line_list.ns =  "boxes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  //LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.1;
  // Points are green
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;

  int id = 0;string ids;
  for(int objectI = 0; objectI < bBoxes.size(); objectI ++){
    for(int pointI = 0; pointI < 4; pointI++){
      id ++; ids = to_string(id);
      geometry_msgs::Point p;
      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4].x;
      p.y = bBoxes[objectI][(pointI+1)%4].y;
      p.z = bBoxes[objectI][(pointI+1)%4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4+4].x;
      p.y = bBoxes[objectI][(pointI+1)%4+4].y;
      p.z = bBoxes[objectI][(pointI+1)%4+4].z;
      line_list.points.push_back(p);
    }
  }
  //line list end
  vis_pub.publish(line_list);
  // bounding box visualizing end---------------------------------------------
}

// void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//   // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
//   ROS_INFO("Imu angular valo x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
// }


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // ros::Subscriber subImu = nh.subscribe ("imu_data", 10, imu_cb);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 40, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  // Spin
  ros::spin ();
}