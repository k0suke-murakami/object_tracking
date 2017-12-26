#ifndef MY_PCL_TUTORIAL_IMM_UKF_JPDAF_H
#define MY_PCL_TUTORIAL_IMM_UKF_JPDAF_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

// void immUkfJpdaf(vector<PointCloud<PointXYZ>> bBoxes);
void immUkfJpdaf(vector<PointCloud<PointXYZ>> bBoxes, double timestamp, 
	vector<PointXY>& targets, vector<int>& trackManage, vector<bool>& isStaticVec);




#endif /* MY_PCL_TUTORIAL_IMM_UKF_JPDAF_H */
