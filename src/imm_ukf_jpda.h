#ifndef OBJECT_TRACKING_IMM_UKF_JPDAF_H
#define OBJECT_TRACKING_IMM_UKF_JPDAF_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "ukf.h"

using namespace std;
using namespace pcl;
using namespace Eigen;



class Tracking{
private:
	bool init_;
	double timestamp_ ;
	double egoVelo_;
	double egoYaw_;
	double egoPreYaw_;
	VectorXd preMeas_;
	vector<UKF> targets_;
	vector<int> trackNumVec_;

	// probabilistic data association params
	double gammaG_;//9.22; // 99%
	double pG_;//0.99;
	// extern double gammaG_ = 5.99; // 99%
	// extern double pG_ = 0.95;
	// extern double gammaG_ = 15.22; // 99%
	double pD_;//0.9;

	//bbox association param
	double distanceThres_;//0.25;
	int lifeTimeThres_;//8;
	//bbox update params
	double bbYawChangeThres_;//0.2; 
	double bbAreaChangeThres_;//0.5;

	// vector<UKF> targets_;
	// vector<int> trackNumVec_;

	void findMaxZandS(const UKF target, VectorXd& maxDetZ, MatrixXd& maxDetS);
	void measurementValidation(const vector<vector<double>> trackPoints, UKF& target, const bool secondInit, 
							   const VectorXd maxDetZ, const MatrixXd maxDetS, vector<VectorXd>& measVec, 
							   vector<VectorXd>& bboxVec, vector<int>& matchingVec);
	void filterPDA(UKF& target, const vector<VectorXd> measVec, vector<double>& lambdaVec);
	void getNearestEuclidBBox(const UKF target, const vector<VectorXd> bboxVec, 
							  vector<double>& Bbox, int& minDist);
	void getRightAngleBBox(const vector<double> nearestBbox, vector<double>& rightAngleBBox);
	void associateBB(const int trackNum, const vector<VectorXd> bboxVec, UKF& target);
	double getBboxArea(const PointCloud<PointXYZ> bBox);
	void updateVisBoxArea(UKF& target, const VectorXd dtCP);
	void updateBoxYaw(UKF& target, const VectorXd cp, const double bestDiffYaw);
	double getBBoxYaw(const UKF target);
	void updateBB(UKF& target);
	double getIntersectCoef(const double vec1x, const double vec1y, const double vec2x, const double vec2y,
                            const double px, const double py, const double cpx, const double cpy);
	void mergeOverSegmentation(const vector<UKF> targets);


public:
	Tracking();
	Eigen::VectorXd getCpFromBbox(const PointCloud<PointXYZ> bBox);
	void immUkfPdaf(const vector<PointCloud<PointXYZ>> bBoxes, const double timestamp, 
	PointCloud<PointXYZ>& targets, vector<vector<double>>& targetVandYaw, 
	vector<int>& trackManage, vector<bool>& isStaticVec,
	vector<bool>& isVisVec, vector<PointCloud<PointXYZ>>& visBB,
	vector<int>& visNumVec);
};




#endif /* OBJECT_TRACKING_IMM_UKF_JPDAF_H */
