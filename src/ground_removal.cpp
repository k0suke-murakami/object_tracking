//
// Created by kosuke on 11/26/17.
//

#include <pcl/io/pcd_io.h>

#include "ground_removal.h"
#include "gaus_blur.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

// // polar grid params
// float rMin_ = 0.0;
// float rMax_ = 50;

// // ground filter params
// // float gHmin_ = -1.9;
// // float gHmax_ = -1.0;
// float gHmin_   = -2.5;
// float gHmax_   = -1.5;
// float gHdiff_  = 0.3;
// float gHthres_ = 0.25;

// // gaussian blur params
// int gaussSigma_   = 1;
// int gaussSamples_ = 3;

// // sensor height
// // float hSensor_ = 1.73; // kitti setup
// float hSensor_ = 2.35; // suginami estima sensor height

Cell::Cell(){
    minZ = 1000;
    isGround = false;
}

void Cell::updateMinZ(float z) {
    if (z < minZ) minZ = z;
}

GroundRemoval::GroundRemoval(){
    // polar grid params
    rMin_ = 0.0;
    rMax_ = 50;

    // ground filter params
    // gHmin_ = -1.9;
    // gHmax_ = -1.0;
    gHmin_   = -2.5;
    gHmax_   = -1.5;
    gHdiff_  = 0.3;
    gHthres_ = 0.25;

    // gaussian blur params
    gaussSigma_   = 1;
    gaussSamples_ = 3;

    // sensor height
    // hSensor_ = 1.73; // kitti setup
    hSensor_ = 2.35; // suginami estima sensor height
}

void GroundRemoval::filterCloud(PointCloud<PointXYZ> cloud, PointCloud<PointXYZ> & filteredCloud){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        float distance = sqrt(x * x + y * y);
        if(distance <= rMin_ || distance >= rMax_) {
            continue; // filter out
        }
        else{
            pcl::PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            filteredCloud.push_back(o);
        }
    }
}

void GroundRemoval::getCellIndexFromPoints(const float x, const float y, int& chI, int& binI){
    float distance = sqrt(x * x + y * y);
    //normalize :atan2(-pi ~ pi)
    float chP  = (atan2(y, x) + M_PI) / (2 * M_PI);
    float binP = (distance - rMin_) / (rMax_ - rMin_);
    //index
    chI  = floor(chP*numChannel_);
    binI = floor(binP*numBin_);
    // cout << "bin ind: "<<binI << " ch ind: "<<chI <<endl;
}

void GroundRemoval::createAndMapPolarGrid(const PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin_>, numChannel_>& polarData ){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);
        // TODO; modify abobe function so that below code would not need
        // here is to prevent segentation fault
        if(chI < 0 || chI >=numChannel_ || binI < 0 || binI >= numBin_) continue; 
        polarData[chI][binI].updateMinZ(z);
    }
}

// update HDiff with larger value
void GroundRemoval::computeHDiffAdjacentCell(array<Cell, numBin_>& channelData){
    for(int i = 0; i < channelData.size(); i++){
        // edge case
        if(i == 0){
            float hD = channelData[i].getHeight() - channelData[i+1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        else if(i == channelData.size()-1){
            float hD = channelData[i].getHeight() - channelData[i-1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        // non-edge case
        else{
            float preHD  = channelData[i].getHeight() - channelData[i-1].getHeight();
            float postHD = channelData[i].getHeight() - channelData[i+1].getHeight();
            if(preHD > postHD) channelData[i].updateHDiff(preHD);
            else channelData[i].updateHDiff(postHD);
        }

//        cout <<channelData[i].getHeight() <<" " <<channelData[i].gegHdiff_() << endl;
    }
}

void GroundRemoval::applyMedianFilter(array<array<Cell, numBin_>, numChannel_>& polarData){
    // maybe later: consider edge case
    for(int channel = 1; channel < polarData.size()-1; channel++){
        for(int bin = 1; bin < polarData[0].size()-1; bin++){
            if(!polarData[channel][bin].isThisGround()){
                // target cell is non-ground AND surrounded by ground cells
                if(polarData[channel][bin+1].isThisGround()&&
                   polarData[channel][bin-1].isThisGround()&&
                   polarData[channel+1][bin].isThisGround()&&
                   polarData[channel-1][bin].isThisGround()){
                    vector<float> sur{polarData[channel][bin+1].getHeight(),
                                      polarData[channel][bin-1].getHeight(),
                                      polarData[channel+1][bin].getHeight(),
                                      polarData[channel-1][bin].getHeight()};
                    sort(sur.begin(), sur.end());
                    float m1 = sur[1]; float m2 = sur[2];
                    float median = (m1+m2)/2;
                    polarData[channel][bin].updataHeight(median);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}

void GroundRemoval::outlierFilter(array<array<Cell, numBin_>, numChannel_>& polarData){
    for(int channel = 1; channel < polarData.size() - 1; channel++) {
        for (int bin = 1; bin < polarData[0].size() - 2; bin++) {
            if(polarData[channel][bin].isThisGround()&&
               polarData[channel][bin+1].isThisGround()&&
               polarData[channel][bin-1].isThisGround()&&
               polarData[channel][bin+2].isThisGround()){
                float height1 = polarData[channel][bin-1].getHeight();
                float height2 = polarData[channel][bin].getHeight();
                float height3 = polarData[channel][bin+1].getHeight();
                float height4 = polarData[channel][bin+2].getHeight();
                if(height1 != gHmin_ && height2 == gHmin_ && height3 != gHmin_){
                    float newH = (height1 + height3)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
                }
                else if(height1 != gHmin_ && height2 == gHmin_ && height3 == gHmin_ && height4 != gHmin_){
                    float newH = (height1 + height4)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}


void GroundRemoval::groundRemove(const PointCloud<pcl::PointXYZ>       cloud,
                  PointCloud<pcl::PointXYZ>::Ptr  elevatedCloud,
                  PointCloud<pcl::PointXYZ>::Ptr  groundCloud){

    // exclude pointclouds outside target region
    PointCloud<pcl::PointXYZ> filteredCloud;
    filterCloud(cloud, filteredCloud);

    // making polar grid data: calculating minZi for each cell
    array<array<Cell, numBin_>, numChannel_> polarData;
    createAndMapPolarGrid(filteredCloud, polarData);

    
    for (int channel = 0; channel < polarData.size(); channel++){
        for (int bin = 0; bin < polarData[0].size(); bin ++){
            float zi = polarData[channel][bin].getMinZ();
            // zi is within threshold, it become height for the cell
            if(zi > gHmin_ && zi < gHmax_){polarData[channel][bin].updataHeight(zi);}
            // zi is above threshold, height will be the sensor height
            else if(zi > gHmax_){polarData[channel][bin].updataHeight(hSensor_);}
            // zi is below threshold, height will ve gHmin
            else {polarData[channel][bin].updataHeight(gHmin_);}
        }

        //to do: need to measure how effective this method is
        //update polarData[channel] with smoothed value 
        GaussSmooth gs;
        gs.gaussSmoothen(polarData[channel], gaussSigma_, gaussSamples_);

        //std::cout << " finished smoothing at channel "<< channel << std::endl;

        //calculating hDiff value with adjacent cell
        computeHDiffAdjacentCell(polarData[channel]);

        // referencing both smoothed value and actual height
        for (int bin = 0; bin < polarData[0].size(); bin ++){
            if(polarData[channel][bin].getSmoothed() < gHmax_ &&
                    polarData[channel][bin].getHDiff() < gHdiff_){
                // if the condition meets, assigh cell's height to ground height
                polarData[channel][bin].updateGround();
            }
            else if(polarData[channel][bin].getHeight() < gHmax_ &&
                    polarData[channel][bin].getHDiff() < gHdiff_){
                polarData[channel][bin].updateGround();
            }
        }
    }

    // compare ground cell and non-ground cell
    // if non-ground cell is surrounded by groud cell:
    //     the non-ground cell would be ground cell with median height
    applyMedianFilter(polarData);

    // only considering gound cell
    // outlier spot would be smoothen by surrounded cell
    outlierFilter(polarData);

    //making elevatedCloud by using ground height
    for(int i = 0; i < filteredCloud.size(); i++) {
        float x = filteredCloud.points[i].x;
        float y = filteredCloud.points[i].y;
        float z = filteredCloud.points[i].z;

        pcl::PointXYZ o;
        o.x = x;
        o.y = y;
        o.z = z;
        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);
        // prevent segmentation fault
        if(chI < 0 || chI >=numChannel_ || binI < 0 || binI >= numBin_) continue;
        
        if (polarData[chI][binI].isThisGround()) {
            float hGround = polarData[chI][binI].getHGround();
            if (z < (hGround + gHthres_)) {
                groundCloud->push_back(o);
            } else {
                elevatedCloud->push_back(o);
            }
        } else {
            elevatedCloud->push_back(o);
        }
    }
}