#include "visualization.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/io/io.h>

#include <vector>


using namespace std;
using namespace pcl;
using namespace Eigen;

Visualization::Visualization(){
	
}


VectorXd Visualization::getCpFromBbox(const PointCloud<PointXYZ> bBox){
    PointXYZ p1 = bBox[0];
    PointXYZ p2 = bBox[1];
    PointXYZ p3 = bBox[2];
    PointXYZ p4 = bBox[3];

    double S1 = ((p4.x -p2.x)*(p1.y - p2.y) - (p4.y - p2.y)*(p1.x - p2.x))/2;
    double S2 = ((p4.x -p2.x)*(p2.y - p3.y) - (p4.y - p2.y)*(p2.x - p3.x))/2;
    double cx = p1.x + (p3.x-p1.x)*S1/(S1+S2);
    double cy = p1.y + (p3.y-p1.y)*S1/(S1+S2);

    VectorXd cp(2);
    cp << cx, cy;
    return cp;
}


void Visualization::visualizeJSKBBox(const std_msgs::Header input_header,
	const ros::Publisher tracked_visbb_pub, const vector<bool> isStaticVec, 
	const vector<PointCloud<PointXYZ>> visBBs, const vector<int> visNumVec, const bool isStableBBox){
	jsk_recognition_msgs::BoundingBoxArray pub_bb_msg;
	pub_bb_msg.header = input_header;

	
	if(isStableBBox){
		assert(visNumVec.size() == visBBs.size());
	}
	
	
	for(int i = 0; i < visBBs.size(); i++){
		jsk_recognition_msgs::BoundingBox bb;
		bb.header          = input_header;
		VectorXd cp        = getCpFromBbox(visBBs[i]);
		bb.pose.position.x = cp[0];
		bb.pose.position.y = cp[1];
		bb.pose.position.z = -1;

		// convert yaw to quartenion
		tf::Matrix3x3 obs_mat;
		// double visYaw = visYawVec[i];

		PointXYZ p1 = visBBs[i][0];
		PointXYZ p2 = visBBs[i][1];
		PointXYZ p3 = visBBs[i][2];

		double dist1 = sqrt((p1.x- p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
		double dist2 = sqrt((p3.x- p2.x)*(p3.x - p2.x) + (p3.y - p2.y)*(p3.y - p2.y));
		double bbYaw;
		// dist1 is length
		if(dist1>dist2){
		    bbYaw = atan2(p1.y - p2.y, p1.x - p2.x);
		    bb.dimensions.x = dist1;
		    bb.dimensions.y = dist2;
		    bb.dimensions.z = 2;
		}
		else{
		    bbYaw = atan2(p3.y - p2.y, p3.x - p2.x);
		    bb.dimensions.x = dist2;
		    bb.dimensions.y = dist1;
		    bb.dimensions.z = 2;

		}

		obs_mat.setEulerYPR(bbYaw , 0, 0);

		tf::Quaternion q_tf;
		obs_mat.getRotation(q_tf);
		bb.pose.orientation.x = q_tf.getX();
		bb.pose.orientation.y = q_tf.getY();
		bb.pose.orientation.z = q_tf.getZ();
		bb.pose.orientation.w = q_tf.getW();


		if(isStableBBox){
			if(visNumVec[i] == enumStatic){
				bb.label = 0; 
			}
			else if(visNumVec[i] == enumDynamic){
				bb.label = 2;
			}
		}
		pub_bb_msg.boxes.push_back(bb);
	}
	tracked_visbb_pub.publish(pub_bb_msg);
}

void Visualization::visualizeArrow(const vector<int> trackManage, const ros::Publisher vis_arrow,
	const PointCloud<PointXYZ> targetPoints, 
	const PointCloud<PointXYZ> localPoints, const vector<bool> isStaticVec, const vector<bool> isVisVec,
	const vector<vector<double>> targetVandYaw, const std_msgs::Header input_header, const double egoYaw){
	for(int i = 0; i < targetPoints.size(); i++){
		visualization_msgs::Marker arrowsG;
		arrowsG.lifetime = ros::Duration(0.1);
		if(trackManage[i] == 0 ) {
			continue;
		}
		if(isVisVec[i] == false ) {
			continue;
		}
		if(isStaticVec[i] == true){
			continue;
		}
		arrowsG.header.frame_id = "/velodyne";
		arrowsG.header.stamp = input_header.stamp;
		arrowsG.ns = "arrows";
		arrowsG.action = visualization_msgs::Marker::ADD;
		arrowsG.type   = visualization_msgs::Marker::ARROW;
		// green
		arrowsG.color.g = 1.0f;
		arrowsG.color.a = 1.0;  
		arrowsG.id = i;
		geometry_msgs::Point p;
		// assert(targetPoints[i].size()==4);
		p.x = localPoints[i].x;
		p.y = localPoints[i].y;
		p.z = 0.5;
		double tv   = targetVandYaw[i][0];
		double tyaw = targetVandYaw[i][1] - egoYaw;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		arrowsG.pose.position.x = p.x;
		arrowsG.pose.position.y = p.y;
		arrowsG.pose.position.z = p.z;

		// convert from 3 angles to quartenion
		tf::Matrix3x3 obs_mat;
		obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
		tf::Quaternion q_tf;
		obs_mat.getRotation(q_tf);
		arrowsG.pose.orientation.x = q_tf.getX();
		arrowsG.pose.orientation.y = q_tf.getY();
		arrowsG.pose.orientation.z = q_tf.getZ();
		arrowsG.pose.orientation.w = q_tf.getW();

		// Set the scale of the arrowsG -- 1x1x1 here means 1m on a side
		arrowsG.scale.x = tv;
		arrowsG.scale.y = 0.1;
		arrowsG.scale.z = 0.1;

		vis_arrow.publish(arrowsG);
	}

}

// void Visualization::visualizeArrow()


// visalizing points code for debug 

// tracking points visualizing start---------------------------------------------
  
  // visualization_msgs::Marker pointsY, pointsG, pointsR, pointsB;
  // pointsY.header.frame_id = pointsG.header.frame_id = pointsR.header.frame_id = pointsB.header.frame_id = "velodyne";
  // pointsY.lifetime = pointsG.lifetime = pointsR.lifetime = pointsB.lifetime = ros::Duration(0.25);
  // // pointsY.header.frame_id = pointsG.header.frame_id = pointsR.header.frame_id = pointsB.header.frame_id = "world";
  // // pointsY.header.stamp= pointsG.header.stamp= pointsR.header.stamp =pointsB.header.stamp = ros::Time::now();
  // pointsY.header.stamp= pointsG.header.stamp= pointsR.header.stamp =pointsB.header.stamp = input->header.stamp;
  // pointsY.ns= pointsG.ns = pointsR.ns =pointsB.ns=  "points";
  // pointsY.action = pointsG.action = pointsR.action = pointsB.action = visualization_msgs::Marker::ADD;
  // pointsY.pose.orientation.w = pointsG.pose.orientation.w  = pointsR.pose.orientation.w =pointsB.pose.orientation.w= 1.0;

  // pointsY.id = 1;
  // pointsG.id = 2;
  // pointsR.id = 3;
  // pointsB.id = 4;
  // pointsY.type = pointsG.type = pointsR.type = pointsB.type = visualization_msgs::Marker::POINTS;

  // // POINTS markers use x and y scale for width/height respectively
  // pointsY.scale.x =pointsG.scale.x =pointsR.scale.x = pointsB.scale.x=0.5;
  // pointsY.scale.y =pointsG.scale.y =pointsR.scale.y = pointsB.scale.y = 0.5;

  // // yellow
  // pointsY.color.r = 1.0f;
  // pointsY.color.g = 1.0f;
  // pointsY.color.b = 0.0f;
  // pointsY.color.a = 1.0;

  // // green
  // pointsG.color.g = 1.0f;
  // pointsG.color.a = 1.0;

  // // red
  // pointsR.color.r = 1.0;
  // pointsR.color.a = 1.0;

  // // blue 
  // pointsB.color.b = 1.0;
  // pointsB.color.a = 1.0;

  // for(int i = 0; i < targetPoints.size(); i++){
  //   if(trackManage[i] == 0) continue;
  //   geometry_msgs::Point p;
  //   // p.x = targetPoints[i].x;
  //   // p.y = targetPoints[i].y;
  //   p.x = localPoints[i].x;
  //   p.y = localPoints[i].y;
  //   p.z = -1.73/2;
  //   // cout << trackManage[i] << endl;
  //   if(isStaticVec[i] == true){
  //     pointsB.points.push_back(p); 
  //   }
  //   else if(trackManage[i] < 5 ){
  //     pointsY.points.push_back(p);
  //   }
  //   else if(trackManage[i] == 5){
  //     pointsG.points.push_back(p);
  //   }
  //   else if(trackManage[i] > 5){
  //     pointsR.points.push_back(p); 
  //   }
  // }
  // vis_pub.publish(pointsY);
  // // cout << "pointsG" << pointsG.points[0].x << " "<< pointsG.points[0].y << endl;
  // vis_pub.publish(pointsG);
  // vis_pub.publish(pointsR);
  // vis_pub.publish(pointsB);
  // tracking poiints visualizing end---------------------------------------------
