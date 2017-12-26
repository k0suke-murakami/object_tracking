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


bool init_= false;
double timestamp_ ;
// const double gammaG_ = 15.22; // 99%

const double gammaG_ = 9.22; // 99%
const double pG_ = 0.99;
// const double gammaG_ = 5.99; // 99%
// const double pG_ = 0.95;
const double pD_ = 0.9;
int countIt = 0;
vector<UKF> targets_;
vector<int> trackNumVec_;

ifstream inFile;


// double figure;
// inFile >> figure;
// cout << figure << endl;
// while(inFile >>figure) {
//     cout << figure << endl;
// }

void findMaxZandS(UKF target, VectorXd& maxDetZ, MatrixXd& maxDetS){
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

void measurementValidation(vector<PointXY> trackPoints, UKF& target, VectorXd maxDetZ, MatrixXd maxDetS,
     vector<VectorXd>& measVec, vector<int>& matchingVec){
    // VectorXd maxDetZ;
    // MatrixXd maxDetS;

    // findMaxZandS(target, maxDetZ, maxDetS);
    // maxDetS = maxDetS*4;

    // maxDetZ = target.modeProbCV_*target.zPredCVl_ + target.modeProbCTRV_ *target.zPredCTRVl_ + target.modeProbRM_ * target.zPredRMl_;
    // maxDetS = target.modeProbCV_  * target.lS_cv_ +
    //            target.modeProbCTRV_* target.lS_ctrv_ +
    //            target.modeProbRM_  * target.lS_rm_;

    //todo; maxdetS is too small, track management would make this remove
    // maxDetS = maxDetS*20;
    // cout << endl<<"max determinant: "<<endl<<maxDetS.determinant()<<endl;
    // cout << "maxDetS" <<endl<<maxDetS<<endl;
    // cout << "maxDetz" <<endl<<maxDetZ<<endl;

    int count = 0;
    for(int i = 0; i < trackPoints.size(); i++){
        double x = trackPoints[i].x;
        double y = trackPoints[i].y;
        VectorXd meas = VectorXd(2);
        meas << x, y;

        VectorXd diff = meas - maxDetZ;
        double nis = diff.transpose()*maxDetS.inverse()*diff;
        // cout <<"nis: " <<nis << endl;
        if(nis < gammaG_){ // x^2 99% range
            count ++;
            if(matchingVec[i] == 0) target.lifetime_ ++;
            // cout <<"nis: " <<nis << endl;
            // cout << "meas"<<endl<<meas << endl;
            measVec.push_back(meas);
            matchingVec[i] = 1;
        }
        // cout << "meas"<<endl<<meas << endl;
    }
}

void filterPDA(UKF& target, vector<VectorXd> measVec, vector<double>& lambdaVec){
    // calculating association probability
    // UKF one = targets[0];
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
    
    // cout << "beta cv" << endl;
    for(int i = 0; i < numMeas; i++){
        double tempCV   = eCvVec[i]/(b+eCVSum);
        double tempCTRV = eCtrvVec[i]/(b+eCTRVSum);
        double tempRM   = eRmVec[i]/(b+eRMSum);
        // cout << tempCV << endl;
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

    // cout <<endl<< "diff z cv: "<<endl<<sigmaXcv<<endl;
    // cout <<endl<< "diff z ctrv: "<<endl<<sigmaXctrv<<endl;
    // cout <<endl<< "diff z rm: "<<endl<<sigmaXrm<<endl;
    // cout <<endl<< "K cv: "<<endl<<one.K_cv_<<endl;
    // cout << endl<<"before x" <<endl << one.x_cv_ <<endl;
    
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
   
    // cout << "x cv: "<<endl << target.x_cv_ << endl;
    // cout << "before update p cv: "<<endl << target.P_cv_ << endl;
    // cout << "b: "<<endl << b << endl;
    // cout << "beta cv zero: "<<endl << betaCVZero << endl;
    // cout << "1 "<< endl<<betaCVZero*target.P_cv_ <<endl;
    // cout << "2 "<< endl<<(1-betaCVZero)*(target.P_cv_ - target.K_cv_*target.lS_cv_*target.K_cv_.transpose()) <<endl;
    // cout << "3 "<< endl<<target.K_cv_*sigmaPcv*target.K_cv_.transpose()  <<endl;

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
    // cout << "after update p cv: "<<endl << target.P_cv_ << endl;
     
    VectorXd maxDetZ;
    MatrixXd maxDetS;
    // cout << endl<<"filterPDA" << endl;
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


void immUkfJpdaf(vector<PointCloud<PointXYZ>> bBoxes, double timestamp,
					 vector<PointXY>& targetPoints, vector<int>& trackManage, vector<bool>& isStaticVec){


	// cout << "first in imm-ukf-jpdaf" << endl;
    countIt ++;
	// cout << bBoxes.size()<<endl;
	// convert from bboxes to cx,cy
    // calculate cx, cy, http://imagingsolution.blog107.fc2.com/blog-entry-137.html
    vector<PointXY> trackPoints;
    for(int i = 0; i < bBoxes.size(); i ++){
        PointXYZ p1 = bBoxes[i][0];
        PointXYZ p2 = bBoxes[i][1];
        PointXYZ p3 = bBoxes[i][2];
        PointXYZ p4 = bBoxes[i][3];

        double S1 = ((p4.x -p2.x)*(p1.y - p2.y) - (p4.y - p2.y)*(p1.x - p2.x))/2;
        double S2 = ((p4.x -p2.x)*(p2.y - p3.y) - (p4.y - p2.y)*(p2.x - p3.x))/2;
        double cx = p1.x + (p3.x-p1.x)*S1/(S1+S2);
        double cy = p1.y + (p3.y-p1.y)*S1/(S1+S2);

        PointXY o;
        o.x = cx;
        o.y = cy;
        trackPoints.push_back(o);
    }

    double egoVelo;
    // this is test case, need to be generalized
    if(!init_) {
        inFile.open("./src/my_pcl_tutorial/src/ego_velo.txt");
        inFile >>egoVelo;
        // cout << egoVelo << endl;
    	for(int i = 0; i < trackPoints.size(); i++){
    		// car i = 9, ped i = 8, car(-8 , 1), cyclist i = 10
    		// if(i == 8){
            // i = 11, missing ped
    		// if(i  == 10 ){
            // if(i  == 11 ){
    			PointXY o;
	    		o.x = trackPoints[i].x;
	    		o.y = trackPoints[i].y;
	    		// o.x = -8;
	    		// o.y = 1;
	    		// o.x = 3;
	    		// o.y = -10;
                // o.x = 8;
                // o.y = -7;
                targetPoints.push_back(o);
                isStaticVec.push_back(false);


	    		VectorXd initMeas = VectorXd(2);
	    		initMeas << o.x, o.y;

                UKF ukf;
                ukf.Initialize(initMeas, timestamp);
                targets_.push_back(ukf);
                //initialize trackNumVec
                trackNumVec_.push_back(1);
    		// }
    	}
        timestamp_ = timestamp;
        trackManage = trackNumVec_;
        init_ = true;
        cout << targets_.size() << " " << trackNumVec_.size() << targetPoints.size()<<endl;
        assert(targets_.size() == trackNumVec_.size());
        assert(targets_.size() == targetPoints.size());
        return;
    }
    inFile >>egoVelo;
    // cout << egoVelo << endl;
    // assuming 2011_09_26_drive_0005: 154 frames
    if(countIt == 154) inFile.close();

    double dt = (timestamp - timestamp_)/1000000.0;
    // cout << "time " << dt << endl;
    timestamp_ = timestamp;

    assert (targets_.size() == trackNumVec_.size());
    vector<int> matchingVec(trackPoints.size()); // make 0 vector
    // cout<< "print matching vec "<<endl;
    // for (int i = 0; i < matchingVec.size(); i++){
    //     cout << matchingVec[i]<<endl;
    // }
    int measValiCount = 0;
    for(int i = 0; i < targets_.size(); i++){
    	//todo: modify here. This skips irregular measurement and nan
    	// if(isnan(targets_[i].x_merge_(0))) continue;
    	if(trackNumVec_[i] == 0) continue;
        if(targets_[i].P_merge_.determinant() > 10 || targets_[i].P_merge_(4,4) > 1000){
            trackNumVec_[i] = 0;
            continue;
        }

        // cout << "covariance"<<endl<<targets_[i].P_merge_<<endl;
        VectorXd maxDetZ;
        MatrixXd maxDetS;
    	vector<VectorXd> measVec;
    	vector<double> lambdaVec;
    	// cout << "ProcessIMMUKF" << endl;
    	targets_[i].ProcessIMMUKF(dt);
    	// cout << "measurementValidation" << endl;
        // pre gating
        findMaxZandS(targets_[i], maxDetZ, maxDetS);
        maxDetS = maxDetS*4;
        double detS = maxDetS.determinant();

        if(isnan(detS)|| detS > 10) {
            trackNumVec_[i] = 0;
            continue;
        }
        // measurement gating
    	measurementValidation(trackPoints, targets_[i], maxDetZ, maxDetS, measVec, matchingVec);

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
    	// cout << "trackNum: " << trackNumVec_[i]<<"-------------------"<<endl;

    	// cout << "validated measurement size: "<< measVec.size() << endl;
	    // cout << "filter pda" << endl;
	    filterPDA(targets_[i], measVec, lambdaVec);
	    // cout << "PostIMMUKF" << endl;
	    targets_[i].PostProcessIMMUKF(lambdaVec);
        double targetVelo = egoVelo + targets_[i].x_merge_(2)*cos(targets_[i].x_merge_(3));
        targets_[i].velo_history_.push_back(targetVelo);
        if(targets_[i].velo_history_.size() == 4) {
            targets_[i].velo_history_.erase (targets_[i].velo_history_.begin());
        }
        measValiCount++;
    }
    // cout << "validated meas count "<<measValiCount << endl;
    int notValiNum = 0;
    int valiNum = 0;
    for(int i = 0; i < targets_.size(); i++){
        // cout << trackNumVec_[i] << endl;
        if(trackNumVec_[i] == 0) notValiNum ++;
        else valiNum ++;
    }
    // cout << "not validated meas count "<<notValiNum << endl;
    // cout << "target num " << targets_.size()<<endl;

    // assert(measValiCount == valiNum);
    // cout<< "after print matching vec "<<endl;
    // for (int i = 0; i < matchingVec.size(); i++){
    //     cout << matchingVec[i]<<endl;
    // }
    // making new ukf target
    int addedNum = 0;
    int targetNum = targets_.size();
    for(int i = 0; i < matchingVec.size(); i ++){
        if(matchingVec[i] == 0){
            PointXY o;
            o.x = trackPoints[i].x;
            o.y = trackPoints[i].y;
            // targetPoints.push_back(o);

            VectorXd initMeas = VectorXd(2);
            initMeas << o.x, o.y;

            UKF ukf;
            ukf.Initialize(initMeas, timestamp);
            targets_.push_back(ukf);
            //initialize trackNumVec
            trackNumVec_.push_back(1);
            addedNum ++;
        }
    }
    assert(targets_.size() == (addedNum + targetNum));
    
    // making poitns for visualization
    int targetNumCount = 0;
    for(int i = 0; i < targets_.size(); i++){
    	PointXY o;
	    o.x = targets_[i].x_merge_(0);
	    o.y = targets_[i].x_merge_(1);
        targetPoints.push_back(o);
        isStaticVec.push_back(false);
        // cout << trackNumVec_[i] << " "<<endl;
        if(trackNumVec_[i] != 0){
            // targetPoints.push_back(o);
            targetNumCount ++;
        }
	    // if(!isnan(o.x)){
	    // 		targetPoints.push_back(o);
	    // 		}
    }
    // static dynamic classification
    for (int i = 0; i < trackNumVec_.size(); i++){
        // once target is static, it is dtatic until lost
        if(targets_[i].isStatic_ ){
            isStaticVec[i] = true;
            continue;
        } 
        double veloVariance = targets_[i].P_merge_(2,2);
        if(trackNumVec_[i] == 5 && targets_[i].lifetime_ > 8 && veloVariance < 1){
            double averageVelo = 0;
            for(int j = 0; j < 3; j++ ){
                averageVelo += targets_[i].velo_history_[j];
            }
            averageVelo = averageVelo/3;
            if(averageVelo < 0.2 && 
                (targets_[i].modeProbRM_ < targets_[i].modeProbCV_ || 
                    targets_[i].modeProbRM_ < targets_[i].modeProbCTRV_ )){
                cout << "staaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaatic" << endl;
                isStaticVec[i] = true;
                targets_[i].isStatic_ = true;
                // trackNumVec_[i] = -1;
            }
            else{

            }
        }
    }
    assert(isStaticVec.size() == targetPoints.size());
    assert(targets_.size() == trackNumVec_.size());
    assert(targets_.size() == targetPoints.size());

    cout << "number of tracked object in immukjpdaf " << targetNumCount<< endl;
    cout << "track num "<< trackNumVec_[0] << endl;
    cout << "track lifetime "<<targets_[0].lifetime_ << endl;
    cout << "target conariance: "<<endl<< targets_[0].P_merge_ << endl;

    // cout << "target det: "<< targets_[0].P_merge_.determinant() << endl;
    // cout << "target velocity " << targets_[0].x_merge_(2)<< endl;
    // cout << "target yaw " << targets_[0].x_merge_(3)<< endl;
    // cout << "target velo toward sin: "<< targets_[0].x_merge_(2)*sin(targets_[0].x_merge_(3))<<endl;
    // cout << "target velo toward cos: "<< targets_[0].x_merge_(2)*cos(targets_[0].x_merge_(3))<<endl;
    // cout << "target abs velocity "<< egoVelo + targets_[0].x_merge_(2)*sin(targets_[0].x_merge_(3))<<endl;
    cout << "target abs velocity "<< egoVelo + targets_[0].x_merge_(2)*cos(targets_[0].x_merge_(3))<<endl;
    // assert(targetNumCount == (addedNum+measValiCount));
    trackManage = trackNumVec_;
}