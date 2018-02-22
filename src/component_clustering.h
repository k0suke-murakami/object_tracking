//
// Created by kosuke on 11/28/17.
//

#ifndef OBJECT_TRACKING_COMPONENT_CLUSTERING_H
#define OBJECT_TRACKING_COMPONENT_CLUSTERING_H

#include <array>
#include <vector>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

//const int numGrid = 100;
// (numGrid, roiM) = (200, 30) means 3000(cm)/200, 15cmx15cm grid aprart leads to different cluster
// const int numGrid = 200;

// const int numGrid_ = 400;
// extern float roiM;
// extern int kernelSize;

class ComponentClustering{
private:
	int clusterId_;
	int kernelSize_;
	float roiM_;
	static const int numGrid_ = 400;

	void mapCartesianGrid(const PointCloud<PointXYZ>::Ptr elevatedCloud,
                             array<array<int, numGrid_>, numGrid_> & cartesianData);
	void search(array<array<int, numGrid_>, numGrid_> & cartesianData, const int cellX, const int cellY);
	void findComponent(array<array<int, numGrid_>, numGrid_> & cartesianData);
	vector<PointCloud<PointXYZRGB>> getClusteredCloud(const PointCloud<pcl::PointXYZ>::Ptr elevatedCloud,
                        							  const array<array<int, numGrid_>, numGrid_> cartesianData);
public:
	ComponentClustering();
	vector<PointCloud<PointXYZRGB>> getComponentClustering(const PointCloud<pcl::PointXYZ>::Ptr elevatedCloud);
};


#endif //OBJECT_TRACKING_COMPONENT_CLUSTERING_H
