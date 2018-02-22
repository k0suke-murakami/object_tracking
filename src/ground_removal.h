//
// Created by kosuke on 11/26/17.
//

#ifndef OBJECT_TRACKING_GROUND_REMOVAL_H
#define OBJECT_TRACKING_GROUND_REMOVAL_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


using namespace std;
using namespace pcl;
using pcl::PointCloud;
using pcl::PointXYZ;


// // define here because array needs to be difined below with these params
const int numChannel_ = 80;
const int numBin_ = 120;

// //const int numMedianKernel = 1;
// extern float rMin_;
// extern float rMax_;
// //const float tHmin = -2.15;

// extern float gHmin_;
// extern float gHmax_;
// //const float tHDiff = 0.3;
// // since estimated ground plane = -1.73 by sensor height,
// // tMin = -2.0
// extern float gHdiff_;
// extern float gHthres_;

// extern int gaussSigma_;
// extern int gaussSamples_;

// extern float hSensor_;

class Cell{
private:
    float smoothed;
    float height;
    float hDiff;
    float hGround;
    float minZ;
    bool isGround;

public:
    Cell();
    void updateMinZ(float z);
    void updataHeight(float h) {height = h;}
    void updateSmoothed(float s) {smoothed = s;}
    void updateHDiff(float hd){hDiff = hd;}
    void updateGround(){isGround = true; hGround = height;}
    bool isThisGround(){return isGround;}
    float getMinZ() {return minZ;}
    float getHeight(){return height;}
    float getHDiff(){ return hDiff;}
    float getSmoothed() {return smoothed;}
    float getHGround() {return hGround;}
};


class GroundRemoval{
private:
    // define here because array needs to be difined below with these params
    // static const int numChannel_ = 80;
    // static const int numBin_ = 120;

     // polar grid params
    float rMin_;
    float rMax_;

    // ground filter params
    // float gHmin_ = -1.9;
    // float gHmax_ = -1.0;
    float gHmin_;
    float gHmax_;
    float gHdiff_;
    float gHthres_;

    // gaussian blur params
    int gaussSigma_;
    int gaussSamples_;

    // sensor height
    // float hSensor_ = 1.73; // kitti setup
    float hSensor_; // suginami estima sensor height

    void filterCloud(PointCloud<PointXYZ> cloud, PointCloud<PointXYZ> & filteredCloud);
    void getCellIndexFromPoints(const float x, const float y, int& chI, int& binI);

    void createAndMapPolarGrid(const PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin_>, numChannel_>& polarData );

    void computeHDiffAdjacentCell(array<Cell, numBin_>& channelData);
    void applyMedianFilter(array<array<Cell, numBin_>, numChannel_>& polarData);
    void outlierFilter(array<array<Cell, numBin_>, numChannel_>& polarData);
public:
    GroundRemoval();
    void groundRemove(const PointCloud<PointXYZ> cloud, 
                  PointCloud<PointXYZ>::Ptr elevatedCloud, 
                  PointCloud<PointXYZ>::Ptr groundCloud); 

};



// void createAndMapPolarGrid(PointCloud<PointXYZ> cloud,
//                            array<array<Cell, numBin_>, numChannel_>& polarData );

// void computeHDiffAdjacentCell(array<Cell, numBin_>& channelData);

// void groundRemove(PointCloud<pcl::PointXYZ> cloud, 
//                   PointCloud<pcl::PointXYZ>::Ptr elevatedCloud, 
//                   PointCloud<pcl::PointXYZ>::Ptr groundCloud); 

#endif //OBJECT_TRACKING_GROUND_REMOVAL_H
