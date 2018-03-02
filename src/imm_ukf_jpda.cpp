//
// Created by kosuke on 12/23/17.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <math.h>

#include "ukf.h"
#include "imm_ukf_jpda.h"

using namespace std;
using namespace Eigen;
using namespace pcl;


Tracking::Tracking(){
    //dynamic 
    init_ = false;


    //params--------------------
    // probabilistic data association params
    gammaG_ = 9.22; // 99%
    pG_ = 0.99;
    // const double gammaG_ = 5.99; // 99%
    // const double pG_ = 0.95;
    // const double gammaG_ = 15.22; // 99%
    pD_ = 0.9;

    //bbox association param
    distanceThres_ = 0.25;
    lifeTimeThres_ = 8;
    //bbox update params
    bbYawChangeThres_  = 0.3; 
    bbAreaChangeThres_ = 0.5;

}

void Tracking::findMaxZandS(const UKF target, VectorXd& maxDetZ, MatrixXd& maxDetS){
    double cv_det   = target.lS_cv_.determinant();
    double ctrv_det = target.lS_ctrv_.determinant();
    double rm_det   = target.lS_rm_.determinant();

    if(cv_det > ctrv_det){
        if(cv_det > rm_det)  {
            maxDetZ = target.zPredCVl_;
            maxDetS = target.lS_cv_;
        }
        else                 {
            maxDetZ = target.zPredRMl_;
            maxDetS = target.lS_rm_;
        }
    }
    else{
        if(ctrv_det > rm_det) {
            maxDetZ = target.zPredCTRVl_;
            maxDetS = target.lS_ctrv_;
        }
        else                  {
            maxDetZ = target.zPredRMl_;
            maxDetS = target.lS_rm_;
        }
    }


}

void Tracking::measurementValidation(const vector<vector<double>> trackPoints, UKF& target, 
                                     const bool secondInit,
                                     const VectorXd maxDetZ, const MatrixXd maxDetS,
                                     vector<VectorXd>& measVec, vector<VectorXd>& bboxVec, 
                                     vector<int>& matchingVec){
   
    int count = 0;
    bool secondInitDone = false;
    double smallestNIS = 999;
    VectorXd smallestMeas = VectorXd(2);
    for(int i = 0; i < trackPoints.size(); i++){
        double x = trackPoints[i][0];
        double y = trackPoints[i][1];
        VectorXd meas = VectorXd(2);
        meas << x, y;

        VectorXd bbox = VectorXd(10);
        bbox << x, y, 
                trackPoints[i][2], trackPoints[i][3],
                trackPoints[i][4], trackPoints[i][5], 
                trackPoints[i][6], trackPoints[i][7],
                trackPoints[i][8], trackPoints[i][9];

        VectorXd diff = meas - maxDetZ;
        double nis = diff.transpose()*maxDetS.inverse()*diff;
        // cout <<"nis: " <<nis << endl;
        if(nis < gammaG_){ // x^2 99% range
            count ++;
            if(matchingVec[i] == 0) target.lifetime_ ++;

            // pick one meas with smallest nis
            if(secondInit){
                if(nis < smallestNIS){
                smallestNIS = nis;
                smallestMeas = meas;
                // measVec.push_back(meas);
                matchingVec[i] = 1;
                secondInitDone = true;
                }
            }
            else{
                measVec.push_back(meas);
                bboxVec.push_back(bbox);
                matchingVec[i] = 1;
            }
        }
    }
    if(secondInitDone) measVec.push_back(smallestMeas);
}

void Tracking::filterPDA(UKF& target, const vector<VectorXd> measVec, vector<double>& lambdaVec){
    // calculating association probability
    double numMeas = measVec.size();
    double b = 2*numMeas*(1-pD_*pG_)/(gammaG_*pD_);
    double eCVSum   = 0;
    double eCTRVSum = 0;
    double eRMSum   = 0;

    vector<double> eCvVec;
    vector<double> eCtrvVec;
    vector<double> eRmVec;

    vector<VectorXd> diffCVVec;
    vector<VectorXd> diffCTRVVec;
    vector<VectorXd> diffRMVec;

    for(int i = 0; i < numMeas; i++){
        VectorXd diffCV   = measVec[i] - target.zPredCVl_;
        VectorXd diffCTRV = measVec[i] - target.zPredCTRVl_;
        VectorXd diffRM   = measVec[i] - target.zPredRMl_;

        diffCVVec.push_back(diffCV);
        diffCTRVVec.push_back(diffCTRV);
        diffRMVec.push_back(diffRM);

        double eCV   = exp(  -0.5*diffCV.transpose()*  target.lS_cv_.inverse()  *diffCV);
        double eCTRV = exp(-0.5*diffCTRV.transpose()*  target.lS_ctrv_.inverse()*diffCTRV);
        double eRM   = exp(  -0.5*diffRM.transpose()*  target.lS_rm_.inverse()  *diffRM);

        eCvVec.push_back(eCV);
        eCtrvVec.push_back(eCTRV);
        eRmVec.push_back(eRM);

        eCVSum   += eCV;
        eCTRVSum += eCTRV;
        eRMSum   += eRM;
    }
    double betaCVZero   = b/(b+eCVSum);
    double betaCTRVZero = b/(b+eCTRVSum);
    double betaRMZero   = b/(b+eRMSum);

    vector<double> betaCV;
    vector<double> betaCTRV;
    vector<double> betaRM;
    
    for(int i = 0; i < numMeas; i++){
        double tempCV   = eCvVec[i]/(b+eCVSum);
        double tempCTRV = eCtrvVec[i]/(b+eCTRVSum);
        double tempRM   = eRmVec[i]/(b+eRMSum);

        betaCV.push_back(tempCV);
        betaCTRV.push_back(tempCTRV);
        betaRM.push_back(tempRM);
    }
    VectorXd sigmaXcv;
    VectorXd sigmaXctrv;
    VectorXd sigmaXrm;
    sigmaXcv.setZero(2);
    sigmaXctrv.setZero(2);
    sigmaXrm.setZero(2);

    for(int i = 0; i < numMeas; i++){
        sigmaXcv   += betaCV[i]*diffCVVec[i];
        sigmaXctrv += betaCTRV[i]*diffCTRVVec[i];
        sigmaXrm   += betaRM[i]*diffRMVec[i];
    }

    MatrixXd sigmaPcv;
    MatrixXd sigmaPctrv;
    MatrixXd sigmaPrm;
    sigmaPcv.setZero(2,2);
    sigmaPctrv.setZero(2,2);
    sigmaPrm.setZero(2,2);
    for(int i = 0; i < numMeas; i++){
        sigmaPcv   += (betaCV[i]  *diffCVVec[i]  *diffCVVec[i].transpose()     - sigmaXcv*sigmaXcv.transpose());
        sigmaPctrv += (betaCTRV[i]*diffCTRVVec[i]*diffCTRVVec[i].transpose()   - sigmaXctrv*sigmaXctrv.transpose());
        sigmaPrm   += (betaRM[i]  *diffRMVec[i]  *diffRMVec[i].transpose()     - sigmaXrm*sigmaXrm.transpose());
    }
    
    // update x and P
    target.x_cv_   = target.x_cv_   + target.K_cv_*sigmaXcv;
    target.x_ctrv_ = target.x_ctrv_ + target.K_ctrv_*sigmaXctrv;
    target.x_rm_   = target.x_rm_   + target.K_rm_*sigmaXrm;

    while (target.x_cv_(3)> M_PI) target.x_cv_(3) -= 2.*M_PI;
    while (target.x_cv_(3)<-M_PI) target.x_cv_(3) += 2.*M_PI;
    while (target.x_ctrv_(3)> M_PI) target.x_ctrv_(3) -= 2.*M_PI;
    while (target.x_ctrv_(3)<-M_PI) target.x_ctrv_(3) += 2.*M_PI;
    while (target.x_rm_(3)> M_PI) target.x_rm_(3) -= 2.*M_PI;
    while (target.x_rm_(3)<-M_PI) target.x_rm_(3) += 2.*M_PI;
   
    if(numMeas != 0){
    	target.P_cv_   = betaCVZero*target.P_cv_ +
                  (1-betaCVZero)*(target.P_cv_ - target.K_cv_*target.lS_cv_*target.K_cv_.transpose()) +
                  target.K_cv_*sigmaPcv*target.K_cv_.transpose();
	    target.P_ctrv_ = betaCTRVZero*target.P_ctrv_ +
	                  (1-betaCTRVZero)*(target.P_ctrv_ - target.K_ctrv_*target.lS_ctrv_*target.K_ctrv_.transpose()) +
	                  target.K_ctrv_*sigmaPctrv*target.K_ctrv_.transpose();
	    target.P_rm_   = betaRMZero*target.P_rm_ +
	                  (1-betaRMZero)*(target.P_rm_ - target.K_rm_*target.lS_rm_*target.K_rm_.transpose()) +
	                  target.K_rm_*sigmaPrm*target.K_rm_.transpose();
    }
    else{
    	target.P_cv_   = target.P_cv_   - target.K_cv_  *target.lS_cv_  *target.K_cv_.transpose();
	    target.P_ctrv_ = target.P_ctrv_ - target.K_ctrv_*target.lS_ctrv_*target.K_ctrv_.transpose();
	    target.P_rm_   = target.P_rm_   - target.K_rm_  *target.lS_rm_  *target.K_rm_.transpose();
    }
    
    VectorXd maxDetZ;
    MatrixXd maxDetS;
    
    findMaxZandS(target, maxDetZ, maxDetS);
    double Vk =  M_PI *sqrt(gammaG_ * maxDetS.determinant());

    double lambdaCV, lambdaCTRV, lambdaRM;
    if(numMeas != 0){
    	lambdaCV   = (1 - pG_*pD_)/pow(Vk, numMeas) +
	                        pD_*pow(Vk, 1-numMeas)*eCVSum/(numMeas*sqrt(2*M_PI*target.lS_cv_.determinant()));
	    lambdaCTRV = (1 - pG_*pD_)/pow(Vk, numMeas) +
	                        pD_*pow(Vk, 1-numMeas)*eCTRVSum/(numMeas*sqrt(2*M_PI*target.lS_ctrv_.determinant()));
	    lambdaRM   = (1 - pG_*pD_)/pow(Vk, numMeas) +
	                        pD_*pow(Vk, 1-numMeas)*eRMSum/(numMeas*sqrt(2*M_PI*target.lS_rm_.determinant()));
    }
    else{
    	lambdaCV   = (1 - pG_*pD_)/pow(Vk, numMeas);
	    lambdaCTRV = (1 - pG_*pD_)/pow(Vk, numMeas);
	    lambdaRM   = (1 - pG_*pD_)/pow(Vk, numMeas);
    }
    // cout <<endl<< "lambda: "<<endl<<lambdaCV << " "<< lambdaCTRV<<" "<< lambdaRM << endl;
    lambdaVec.push_back(lambdaCV);
    lambdaVec.push_back(lambdaCTRV);
    lambdaVec.push_back(lambdaRM);
}

void Tracking::getNearestEuclidBBox(const UKF target, const vector<VectorXd> bboxVec, 
                                    vector<double>& Bbox, int& minDist){
    int minInd = 0;
    double px = target.x_merge_(0);
    double py = target.x_merge_(1);
    for (int i = 0; i < bboxVec.size(); i++){
        double measX = bboxVec[i](0);
        double measY = bboxVec[i](1);
        double dist = sqrt((px-measX)*(px-measX)+(py-measY)*(py-measY));
        if(dist < minDist){
            minDist = dist;
            minInd = i;
        }
    }
    assert(bboxVec[minInd].rows() == 10);
    for(int i = 0; i < 10; i++){
        Bbox.push_back(bboxVec[minInd](i));
    }
}




void Tracking::getRightAngleBBox(const vector<double> nearestBbox, 
                                 vector<double>& rightAngleBBox){
    double p1x = nearestBbox[2];
    double p1y = nearestBbox[3];
    double p2x = nearestBbox[4];
    double p2y = nearestBbox[5];
    double p3x = nearestBbox[6];
    double p3y = nearestBbox[7];
    double p4x = nearestBbox[8];
    double p4y = nearestBbox[9];

    double vec1x = p2x - p1x;
    double vec1y = p2y - p1y;
    double vec2x = p3x - p2x;
    double vec2y = p3y - p2y;

    //from the equation of inner product
    double cosTheta = (vec1x*vec2x + vec1y*vec2y)/
                        (sqrt(vec1x*vec1x+vec2x*vec2x)+sqrt(vec1y*vec1y+vec2y*vec2y));
    double theta = acos(cosTheta);
    double diffTheta = theta - M_PI/2;

    // cout << "p1 "<<p1x << " "<<p1y<<endl;
    // cout << "p2 "<<p2x << " "<<p2y<<endl;
    // cout << "p3 "<<p3x << " "<<p3y<<endl;
    // cout << "p4 "<<p4x << " "<<p4y<<endl;
    // cout << "theta "<< theta << endl;
    // cout << "diffTheta "<<diffTheta<<endl;


    rightAngleBBox = nearestBbox;
    if(abs(diffTheta) > 0.1){

        // cout << "p1 "<<p1x << " "<<p1y<<endl;
        // cout << "p2 "<<p2x << " "<<p2y<<endl;
        // cout << "p3 "<<p3x << " "<<p3y<<endl;
        // cout << "p4 "<<p4x << " "<<p4y<<endl;
        // cout << "theta "<< theta << endl;
        // cout << "diffTheta "<<diffTheta<<endl;

        double m1 = vec1y/vec1x;
        double b1 = p3y - m1 * p3x;
        double m2 = -1.0 / m1;
        double b2 = p2y - (m2 * p2x);

        double x = (b2 - b1) / (m1 - m2);
        double y = (b2 * m1 - b1 * m2) / (m1 - m2);

        double deltaX = x - p2x;
        double deltaY = y - p2y;
        

        rightAngleBBox[6] = x; 
        rightAngleBBox[7] = y;
        rightAngleBBox[8] = rightAngleBBox[2] + deltaX;
        rightAngleBBox[9] = rightAngleBBox[3] + deltaY;
    }
}


void Tracking::associateBB(const int trackNum, const vector<VectorXd> bboxVec, UKF& target){
    //skip if no validated measurement
    if(bboxVec.size() == 0) {
        return;
    }
    if(trackNum == 5 && target.lifetime_ > lifeTimeThres_){
        vector<double> nearestBbox;
        int minDist = 999;
        getNearestEuclidBBox(target, bboxVec, nearestBbox, minDist);
        if(minDist < distanceThres_){
            PointCloud<PointXYZ> bbox;
            PointXYZ o;
            vector<double> rightAngleBBox;
            getRightAngleBBox(nearestBbox, rightAngleBBox);

            assert(rightAngleBBox.size() == 10);
            for(int i = 0; i < 2; i++){
                double height;
                if(i == 0) {height = 0;}
                else       {height = 2.35;}
                o.x = rightAngleBBox[2];
                o.y = rightAngleBBox[3];
                o.z = height;
                bbox.push_back(o);
                o.x = rightAngleBBox[4];
                o.y = rightAngleBBox[5];
                o.z = height;
                bbox.push_back(o);
                o.x = rightAngleBBox[6];
                o.y = rightAngleBBox[7];
                o.z = height;
                bbox.push_back(o);
                o.x = rightAngleBBox[8];
                o.y = rightAngleBBox[9];
                o.z = height;
                bbox.push_back(o);
            }
            target.isVisBB_ = true;
            target.BBox_    = bbox;
        }
    }
}

VectorXd Tracking::getCpFromBbox(const PointCloud<PointXYZ> bBox){
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

double Tracking::getBboxArea(const PointCloud<PointXYZ> bBox){
    PointXYZ p1 = bBox[0];
    PointXYZ p2 = bBox[1];
    PointXYZ p3 = bBox[2];
    PointXYZ p4 = bBox[3];

    //S=tri(p1,p2,p3) + tri(p1, p3, p4)
    //s(triangle) = 1/2*|(x1−x3)(y2−y3)−(x2−x3)(y1−y3)|
    double tri1 = 0.5*abs((p1.x - p3.x)*(p2.y - p3.y) - (p2.x - p3.x)*(p1.y - p3.y));
    double tri2 = 0.5*abs((p1.x - p4.x)*(p3.y - p4.y) - (p3.x - p4.x)*(p1.y - p4.y)); 
    double S = tri1 + tri2;
    return S;
}

void Tracking::updateVisBoxArea(UKF& target, const VectorXd dtCP){
    // cout << "calling area update"<<endl;
    int lastInd = target.bb_yaw_history_.size()-1;
    // double diffYaw = target.bb_yaw_history_[lastInd] - target.bb_yaw_history_[lastInd-1];
    // cout << dtCP << endl;
    double area = getBboxArea(target.bestBBox_);
    for(int i = 0; i < target.BBox_.size(); i++){
        target.BBox_[i].x = target.bestBBox_[i].x + dtCP(0);
        target.BBox_[i].y = target.bestBBox_[i].y + dtCP(1);
    }

    double postArea = getBboxArea(target.BBox_);
    assert(abs(area - postArea)< 0.001);

}

void Tracking::updateBoxYaw(UKF& target, const VectorXd cp, const double diffYaw){
    
    for(int i = 0; i < target.BBox_.size(); i++){
        // rotate around cp
        double preX = target.BBox_[i].x;
        double preY = target.BBox_[i].y;
        target.BBox_[i].x = cos(diffYaw)*(preX - cp(0)) - sin(diffYaw)*(preY - cp(1)) + cp(0);
        target.BBox_[i].y = sin(diffYaw)*(preX - cp(0)) + cos(diffYaw)*(preY - cp(1)) + cp(1);
    }
}


double Tracking::getBBoxYaw(const UKF target){
    PointCloud<PointXYZ> bBox = target.BBox_;
    PointXYZ p1 = bBox[0];
    PointXYZ p2 = bBox[1];
    PointXYZ p3 = bBox[2];
    double dist1 = sqrt((p1.x- p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    double dist2 = sqrt((p3.x- p2.x)*(p3.x - p2.x) + (p3.y - p2.y)*(p3.y - p2.y));
    
    double yaw;
    // dist1 is length
    if(dist1>dist2){
        yaw = atan2(p1.y - p2.y, p1.x - p2.x);
    }
    else{
        yaw = atan2(p3.y - p2.y, p3.x - p2.x);   
    }

    double ukfYaw  = target.x_merge_(3);
    double diffYaw = abs(yaw - ukfYaw); 
    if(diffYaw < M_PI*0.5){
        return yaw;
    }
    else{
        yaw += M_PI;
        while (yaw> M_PI) yaw -= 2.*M_PI;
        while (yaw<-M_PI) yaw += 2.*M_PI;
        return yaw;
    }
}

void Tracking::updateBB(UKF& target){
    //to do: initialize target.BBox_ somewhere else

    // skip to prevent memory leak by accessing empty target.bbox_
    if(!target.isVisBB_){
        return;
    }
    // skip the rest of process if it is first bbox associaiton
    if(target.bestBBox_.empty()){
        target.bestBBox_ = target.BBox_;
        target.bestYaw_  = getBBoxYaw(target);
        return;
    }

    // calculate yaw
    VectorXd cp         = getCpFromBbox(target.BBox_);
    VectorXd bestCP     = getCpFromBbox(target.bestBBox_);
    VectorXd dtCP       = cp - bestCP;
    double yaw = getBBoxYaw(target);

    // bbox area
    double area     = getBboxArea(target.BBox_);
    double bestArea = getBboxArea(target.bestBBox_);

    // start updating parameters
    double deltaArea = area - bestArea;

    // when the delta area is under 0, keep best area and relocate(slide) it for current cp
    if( deltaArea < 0 ){
        updateVisBoxArea(target, dtCP);
        // for mergeSegmentation, area comparison
        target.bb_area_ = bestArea;
    }
    else if(deltaArea > 0){
        target.bestBBox_ = target.BBox_;
        // for mergeSegmentation, area comparison
        target.bb_area_  = area;
    }

    double currentYaw  = getBBoxYaw(target);
    double diffYaw = yaw - currentYaw;
    
    cout << "diff yaw: "<< diffYaw<< endl;

    // diffYaw is within the threshold, apply the diffYaw chamge
    if(abs(diffYaw) < 0.01 && abs(diffYaw) > bbYawChangeThres_){
        updateBoxYaw(target, cp, diffYaw);
    }
}

double Tracking::getIntersectCoef(const double vec1x, const double vec1y, const double vec2x, const double vec2y,
                                  const double px, const double py, const double cpx, const double cpy){
    double intersectCoef = (((vec1x-vec2x)*(py - vec1y) + (vec1y - vec2y)*(vec1x - px)) *
        ((vec1x - vec2x)*(cpy - vec1y) + (vec1y - vec2y)*(vec1x - cpx)));
    return intersectCoef;

}

void Tracking::mergeOverSegmentation(const vector<UKF> targets){
    // cout << "mergeOverSegmentation"<<endl;
    for(int i = 0; i < targets.size(); i++){
        if(targets[i].isVisBB_ == true){
            double vec1x = targets[i].BBox_[0].x;
            double vec1y = targets[i].BBox_[0].y;
            double vec2x = targets[i].BBox_[1].x;
            double vec2y = targets[i].BBox_[1].y;
            double vec3x = targets[i].BBox_[2].x;
            double vec3y = targets[i].BBox_[2].y;
            double vec4x = targets[i].BBox_[3].x;
            double vec4y = targets[i].BBox_[3].y;
            double cp1x  = (vec1x+vec2x+vec3x)/3;
            double cp1y  = (vec1y+vec2y+vec3y)/3;
            double cp2x  = (vec1x+vec4x+vec3x)/3;
            double cp2y  = (vec1y+vec4y+vec3y)/3;
            for (int j = 0; j < targets.size(); j++){
                if(i == j) continue;
                // make sure that merged area(i) is larger than compared area(j)
                if(targets_[i].bb_area_ < targets_[j].bb_area_){
                    double px = targets[j].x_merge_(0);
                    double py = targets[j].x_merge_(1);
                    // check if px, py are in triangle vec1, vec2, vec3
                    double cross1 = getIntersectCoef(vec1x, vec1y, vec2x, vec2y, px, py, cp1x, cp1y);
                    double cross2 = getIntersectCoef(vec1x, vec1y, vec3x, vec3y, px, py, cp1x, cp1y);
                    double cross3 = getIntersectCoef(vec3x, vec3y, vec2x, vec2y, px, py, cp1x, cp1y);
                    // check if px, py are in triangle vec1, vec3, vec4
                    double cross4 = getIntersectCoef(vec1x, vec1y, vec4x, vec4y, px, py, cp2x, cp2y);
                    double cross5 = getIntersectCoef(vec1x, vec1y, vec3x, vec3y, px, py, cp2x, cp2y);
                    double cross6 = getIntersectCoef(vec3x, vec3y, vec4x, vec4y, px, py, cp2x, cp2y);
                    // check if inside or not
                    if((cross1 > 0 && cross2>0&&cross3>0)||(cross4>0 && cross5 > 0 && cross6>0)){
                        trackNumVec_[i] = 5;
                        trackNumVec_[j] = 0;
                    }
                }
            }
        }
    }
}


void Tracking::immUkfPdaf(const vector<pcl::PointCloud<pcl::PointXYZ>> bBoxes, const double timestamp,
                     PointCloud<PointXYZ>& targetPoints, vector<vector<double>>& targetVandYaw,
                     vector<int>& trackManage, vector<bool>& isStaticVec,
                     vector<bool>& isVisVec, vector<PointCloud<PointXYZ>>& visBBs,
                     vector<int>& visNumVec){

	// convert from bboxes to cx,cy + each points
    // calculate cx, cy, http://imagingsolution.blog107.fc2.com/blog-entry-137.html
    vector<vector<double>> trackPoints;
    // std::cout<<endl << "inside tracker"<< std::endl;
    // std::cout << "0 cluster pointloud data "<<bBoxes[0] << std::endl;
    std::cout << "cluster num "<<bBoxes.size() << std::endl;
    // std::cout << "0 cluster size "<<bBoxes[0].size() << std::endl;
    // std::cout << "0 cluster first point y element "<<bBoxes[0][0].y << std::endl;
    for(int i = 0; i < bBoxes.size(); i ++){
        PointXYZ p1 = bBoxes[i][0];
        PointXYZ p2 = bBoxes[i][1];
        PointXYZ p3 = bBoxes[i][2];
        PointXYZ p4 = bBoxes[i][3];

        VectorXd cp = getCpFromBbox(bBoxes[i]);

        vector<double> point;
        point.push_back(cp(0));
        point.push_back(cp(1));
        point.push_back(p1.x);
        point.push_back(p1.y);
        point.push_back(p2.x);
        point.push_back(p2.y);
        point.push_back(p3.x);
        point.push_back(p3.y);
        point.push_back(p4.x);
        point.push_back(p4.y);

        trackPoints.push_back(point);
    }

    if(!init_) {
    	for(int i = 0; i < trackPoints.size(); i++){
            double px = trackPoints[i][0];
            double py = trackPoints[i][1];

            PointXYZ o;
            o.x = px;
            o.y = py;
            o.z = -1.73/2;
            targetPoints.push_back(o);

            vector<double> VandYaw;
            VandYaw.push_back(0);
            VandYaw.push_back(0);
            targetVandYaw.push_back(VandYaw);
            isStaticVec.push_back(false);
            isVisVec.push_back(false);

    		VectorXd initMeas = VectorXd(2);
    		initMeas << px, py;

            UKF ukf;
            ukf.Initialize(initMeas, timestamp);
            targets_.push_back(ukf);
            //initialize trackNumVec_ with 1
            trackNumVec_.push_back(1);
    	}
        timestamp_ = timestamp;
        egoPreYaw_ = egoYaw_;
        trackManage = trackNumVec_;
        init_ = true;
        
        assert(targets_.size() == trackNumVec_.size());
        assert(targets_.size() == targetPoints.size());
        assert(targetPoints.size()== targetVandYaw.size());
        return;
    }

    assert (targets_.size() == trackNumVec_.size());

    // used for making new target with no data association
    vector<int> matchingVec(trackPoints.size()); // make 0 vector

    double dt = (timestamp - timestamp_);
    timestamp_ = timestamp;


    // start UKF process
    for(int i = 0; i < targets_.size(); i++){
        //reset isVisBB_ to false
        targets_[i].isVisBB_ = false;

    	//todo: modify here. This skips irregular measurement and nan
    	if(trackNumVec_[i] == 0) continue;
        // prevent ukf not to explode
        if(targets_[i].P_merge_.determinant() > 10 || targets_[i].P_merge_(4,4) > 1000){
            trackNumVec_[i] = 0;
            continue;
        }
        // immukf prediction step
        targets_[i].PredictionIMMUKF(dt);


        VectorXd maxDetZ;
        MatrixXd maxDetS;
    	vector<VectorXd> measVec;
        vector<VectorXd> bboxVec;
    	vector<double> lambdaVec;
        // find maxDetS associated with predZ
        findMaxZandS(targets_[i], maxDetZ, maxDetS);
        // to do: might modufy here: this code ensures that measurement is incorporated
        maxDetS = maxDetS*4;
        double detS = maxDetS.determinant();

        // prevent ukf not to explode
        if(isnan(detS)|| detS > 10) {
            trackNumVec_[i] = 0;
            continue;
        }

        bool secondInit;
        if(trackNumVec_[i] == 1){
            secondInit = true;
        }
        else{
            secondInit = false;
        }

        // measurement gating, get measVec, bboxVec, matchingVec through reference
        measurementValidation(trackPoints, targets_[i], secondInit, maxDetZ, maxDetS,
         measVec, bboxVec, matchingVec);

        // bounding box association if target is stable :plus, right angle correction if its needed
        // input: track number, bbox measurements, &target 
        associateBB(trackNumVec_[i], bboxVec, targets_[i]);

        // bounding box validation
        updateBB(targets_[i]);

        // cout << "validated meas "<<measVec[0][0]<<" "<<measVec[0][1]<<endl; 

        // second detection for a target: update v and yaw
        if(secondInit){
            if(measVec.size() == 0){
                trackNumVec_[i] = 0;
                continue;
            }
            
            assert(measVec.size() == 1);
            // record init measurement for env classification
            targets_[i].initMeas_ << targets_[i].x_merge_(0), targets_[i].x_merge_(1);

            // abs update
            double targetX = measVec[0](0);
            double targetY = measVec[0](1);
            double targetDiffX = targetX - targets_[i].x_merge_(0);
            double targetDiffY = targetY - targets_[i].x_merge_(1);
            double targetYaw = atan2(targetDiffY, targetDiffX);
            double dist      = sqrt(targetDiffX*targetDiffX + targetDiffY* targetDiffY);
            double targetV   = dist/dt;
            // double targetV   = 2;
            
            while (targetYaw> M_PI) targetYaw -= 2.*M_PI;
            while (targetYaw<-M_PI) targetYaw += 2.*M_PI;
            
            targets_[i].x_merge_(0) = targets_[i].x_cv_(0) = targets_[i].x_ctrv_(0) = targets_[i].x_rm_(0) = targetX;
            targets_[i].x_merge_(1) = targets_[i].x_cv_(1) = targets_[i].x_ctrv_(1) = targets_[i].x_rm_(1) = targetY;
            targets_[i].x_merge_(2) = targets_[i].x_cv_(2) = targets_[i].x_ctrv_(2) = targets_[i].x_rm_(2) = targetV;
            targets_[i].x_merge_(3) = targets_[i].x_cv_(3) = targets_[i].x_ctrv_(3) = targets_[i].x_rm_(3) = targetYaw;

            trackNumVec_[i]++;
            continue;
        }

    	// update tracking number 
    	if(measVec.size() > 0) {
    		if(trackNumVec_[i] < 3){
    			trackNumVec_[i]++;
    		}
    		else if(trackNumVec_[i] == 3){
    			trackNumVec_[i] = 5;
    		}
    		else if(trackNumVec_[i] >= 5){
    			trackNumVec_[i] = 5;
    		}
    	}else{
            if(trackNumVec_[i] < 5){
                trackNumVec_[i] = 0;
            }
    		else if(trackNumVec_[i] >= 5 && trackNumVec_[i] < 10){
    			trackNumVec_[i]++;
    		}
    		else if(trackNumVec_[i] = 10){
    			trackNumVec_[i] = 0;
    		}
    	}

        if(trackNumVec_[i] == 0) continue;


	    filterPDA(targets_[i], measVec, lambdaVec);

	    targets_[i].PostProcessIMMUKF(lambdaVec);
    }
    // end UKF process


    // deling with over segmentation, update trackNumVec_
    mergeOverSegmentation(targets_);


    // making new ukf target for no data association clusters
    int addedNum = 0;
    int targetNum = targets_.size();
    
    for(int i = 0; i < matchingVec.size(); i ++){
        if(matchingVec[i] == 0){
            double px = trackPoints[i][0];
            double py = trackPoints[i][1];

            VectorXd initMeas = VectorXd(2);
            initMeas << px, py;

            UKF ukf;
            ukf.Initialize(initMeas, timestamp);
            targets_.push_back(ukf);
            //initialize trackNumVec
            trackNumVec_.push_back(1);
            addedNum ++;
        }
    }
    assert(targets_.size() == (addedNum + targetNum));

    
    // making poitns and arrows for visualization
    int targetNumCount = 0;
    for(int i = 0; i < targets_.size(); i++){
        double tx = targets_[i].x_merge_(0);
        double ty = targets_[i].x_merge_(1);
        double mx = targets_[i].initMeas_(0);
        double my = targets_[i].initMeas_(1);
        
        targets_[i].distFromInit_ = sqrt((tx - mx)*(tx - mx) + (ty - my)*(ty - my));
        vector<double> cp;
        cp.push_back(tx);
        cp.push_back(ty);

        double tv = targets_[i].x_merge_(2);
        double tyaw = targets_[i].x_merge_(3);

        // tyaw += egoPoints_[0][2];
        while (tyaw> M_PI) tyaw -= 2.*M_PI;
        while (tyaw<-M_PI) tyaw += 2.*M_PI;
        // cout << "testing yaw off "<< tyaw << endl;

        PointXYZ o;
        o.x = cp[0];
        o.y = cp[1];
        o.z = -1.73/2;
        targetPoints.push_back(o);

        // for arrow visualization
        vector<double> VandYaw;
        VandYaw.push_back(tv);
        VandYaw.push_back(tyaw);
        targetVandYaw.push_back(VandYaw);


        isStaticVec.push_back(false);

        isVisVec.push_back(targets_[i].isVisBB_);
        if(targets_[i].isVisBB_){
            visBBs.push_back(targets_[i].BBox_);
        }
        if(trackNumVec_[i] != 0){
            targetNumCount ++;
        }
    }

    // static dynamic classification
    for (int i = 0; i < trackNumVec_.size(); i++){
        // once target is static, it is dtatic until lost
        if(targets_[i].isStatic_ ){
            isStaticVec[i] = true;
        } 
       
        if(!targets_[i].isStatic_ && trackNumVec_[i] == 5 && targets_[i].lifetime_ > 8 ){
            double distThres = 3.0;
            if((targets_[i].distFromInit_ < distThres)&&
                    (targets_[i].modeProbRM_ > targets_[i].modeProbCV_ || 
                     targets_[i].modeProbRM_ > targets_[i].modeProbCTRV_ )){
                isStaticVec[i] = true;
                targets_[i].isStatic_ = true;
            }
        }


        // for visBBs visualization
        if(targets_[i].isVisBB_){
            if(targets_[i].isStatic_){
              visNumVec.push_back(99);
            }
            else if(trackNumVec_[i] < 5 ){
              visNumVec.push_back(trackNumVec_[i]);
            }
            else if(trackNumVec_[i] == 5){
              visNumVec.push_back(trackNumVec_[i]);
            }
            else if(trackNumVec_[i] > 5){
              visNumVec.push_back(trackNumVec_[i]);
            }
        }

    }
    assert(isVisVec.size() == targetPoints.size());
    assert(isStaticVec.size() == targetPoints.size());
    assert(targets_.size() == trackNumVec_.size());
    assert(targets_.size() == targetPoints.size());
    assert(targetPoints.size()== targetVandYaw.size());
    assert(visBBs.size() == visNumVec.size());

    trackManage = trackNumVec_;
    egoPreYaw_ = egoYaw_;
}