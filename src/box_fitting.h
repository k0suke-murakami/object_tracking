//
// Created by kosuke on 11/29/17.
//

#ifndef OBJECT_TRACKING_BOX_FITTING_H
#define OBJECT_TRACKING_BOX_FITTING_H

#include <array>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <opencv2/opencv.hpp>


#include "component_clustering.h"

using namespace std;
using namespace pcl;
using namespace cv;

// extern float picScale; // picScale * roiM = 30 * 30
// //const float picScale = 30;
// extern int ramPoints;
// extern int lSlopeDist;
// extern int lnumPoints;

// extern float tHeightMin;
// extern float tHeightMax;
// extern float tWidthMin;
// extern float tWidthMax;
// extern float tLenMin;
// extern float tLenMax;
// extern float tAreaMax;
// extern float tRatioMin;
// extern float tRatioMax;
// extern float minLenRatio;
// extern float tPtNumPerVol;

class  BoxFitting{
private:
	float sensorHeight_;
	float roiM_;
	float picScale_; // picScale * roiM = 30 * 30
	//const float picScale = 30;
	int ramPoints_;
	int lSlopeDist_;
	int lnumPoints_;

	float tHeightMin_;
	float tHeightMax_;
	float tWidthMin_;
	float tWidthMax_;
	float tLenMin_;
	float tLenMax_;
	float tAreaMax_;
	float tRatioMin_;
	float tRatioMax_;
	float minLenRatio_;
	float tPtNumPerVol_;


	void getPointsInPcFrame(Point2f rectPoints[], vector<Point2f>& pcPoints, int offsetX, int offsetY);
	bool ruleBasedFilter(vector<Point2f> pcPoints, float maxZ, int numPoints);
	void getBoundingBox(vector<PointCloud<PointXYZRGB>>  clusteredPoints,
                    double timestamp,
                    vector<PointCloud<PointXYZ>>& bbPoints);
public:
	BoxFitting();
	vector<PointCloud<PointXYZ>> getBBoxes(vector<PointCloud<PointXYZRGB>> clusteredPoints,
                                                   double timestamp);

};


#endif //OBJECT_TRACKING_BOX_FITTING_H
