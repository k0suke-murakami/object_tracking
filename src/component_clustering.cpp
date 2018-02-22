//
// Created by kosuke on 11/28/17.
//
#include <array>
#include <pcl/io/pcd_io.h>

#include "component_clustering.h"

using namespace std;
using namespace pcl;

//int numGrid = 2;
// float roiM = 30;
// float roiM = 60;
// int kernelSize = 3;

ComponentClustering::ComponentClustering(){
    kernelSize_ = 3;
    clusterId_  = 0;
    roiM_ = 60;
    // numGrid_ = 400;
}


void ComponentClustering::mapCartesianGrid(const PointCloud<PointXYZ>::Ptr elevatedCloud,
                             array<array<int, numGrid_>, numGrid_> & cartesianData){

    for(int i = 0; i < elevatedCloud->size(); i++){
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float xC = x+roiM_/2;
        float yC = y+roiM_/2;
        // exclude outside roi points
        if(xC < 0 || xC >= roiM_ || yC < 0 || yC >=roiM_) continue;
        int xI = floor(numGrid_*xC/roiM_);
        int yI = floor(numGrid_*yC/roiM_);
        cartesianData[xI][yI] = -1;
    }
}

void ComponentClustering::search(array<array<int, numGrid_>, numGrid_> & cartesianData,  const int cellX, const int cellY){
    cartesianData[cellX][cellY] = clusterId_;
    int mean = kernelSize_/2;
    for (int kX = 0; kX < kernelSize_; kX++){
        int kXI = kX-mean;
        if((cellX + kXI) < 0 || (cellX + kXI) >= numGrid_) continue;
        for( int kY = 0; kY < kernelSize_;kY++){
            int kYI = kY-mean;
            if((cellY + kYI) < 0 || (cellY + kYI) >= numGrid_) continue;

            if(cartesianData[cellX + kXI][cellY + kYI] == -1){
                search(cartesianData,  cellX +kXI, cellY + kYI);
            }

        }
    }
}

void ComponentClustering::findComponent(array<array<int, numGrid_>, numGrid_> & cartesianData){
    for(int cellX = 0; cellX < numGrid_; cellX++){
        for(int cellY = 0; cellY < numGrid_; cellY++){
            if(cartesianData[cellX][cellY] == -1){
                clusterId_ ++;
                search(cartesianData,  cellX, cellY);
            }
        }
    }
}



vector<PointCloud<PointXYZRGB>> ComponentClustering::getClusteredCloud(
    const PointCloud<PointXYZ>::Ptr elevatedCloud,
    const array<array<int, numGrid_>, numGrid_> cartesianData){

    //make numCluster size vector
    vector<PointCloud<PointXYZRGB>> clusteredPoints(clusterId_); 
   
    for(int i = 0; i < elevatedCloud->size(); i++){
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float z = elevatedCloud->points[i].z;
        float xC = x+roiM_/2;
        float yC = y+roiM_/2;
        // exclude outside roi points
        if(xC < 0 || xC >= roiM_ || yC < 0 || yC >=roiM_) continue;
        int xI = floor(numGrid_*xC/roiM_);
        int yI = floor(numGrid_*yC/roiM_);

        int clusterNum = cartesianData[xI][yI];
        if(clusterNum != 0){
            PointXYZRGB o;
            o.x = x;
            o.y = y;
            o.z = z;
            o.r = (500*clusterNum)%255;
            o.g = (100*clusterNum)%255;
            o.b = (150*clusterNum)%255;
            clusteredPoints[clusterNum-1].push_back(o);
        }
    }
    return clusteredPoints;
}

vector<PointCloud<PointXYZRGB>> ComponentClustering::getComponentClustering(
    const PointCloud<PointXYZ>::Ptr elevatedCloud){
    // map 50m radius data(polar grid data) into 100x100 cartesian grid,
    // parameter might need to be modified
    // in this case 60mx60m with 400x400 grid

    // making cartesianData initialized with clusterId
    array<array<int, numGrid_>, numGrid_> cartesianData{};
    mapCartesianGrid(elevatedCloud, cartesianData);
    findComponent(cartesianData);
    vector<PointCloud<PointXYZRGB>> clusteredPoints = getClusteredCloud(elevatedCloud, cartesianData);
    return clusteredPoints;
}