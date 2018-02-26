//
// Created by kosuke on 11/29/17.
//
#include <array>
#include <random>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include "box_fitting.h"


using namespace std;
using namespace pcl;
using namespace cv;

// // float picScale = 30;
// float roiM_ = 60;
// float picScale = 900/roiM_;
// int ramPoints = 80;

// // l-shape fitting params
// int lSlopeDist = 30;
// int lnumPoints = 300;

// // float sensorHeight = 1.73;
// float sensorHeight = 2.35;

// // rule-based filter params
// float tHeightMin = 1.0;
// // float tHeightMin = 1.2
// float tHeightMax = 2.6;
// // float tWidthMin = 0.5;
// // float tWidthMin = 0.4;
// float tWidthMin = 0.25;
// float tWidthMax = 3.5;
// float tLenMin = 0.5;
// float tLenMax = 14.0;
// float tAreaMax = 20.0;
// float tRatioMin = 1.3;
// float tRatioMax = 5.0;
// float minLenRatio = 3.0;
// float tPtNumPerVol = 8; // point count per bbox volume

BoxFitting::BoxFitting(){
    // float picScale = 30;
    roiM_ = 60;
    picScale_ = 900/roiM_;
    ramPoints_ = 80;

    // l-shape fitting params
    lSlopeDist_ = 2.0;
    lnumPoints_ = 300;

    // float sensorHeight_ = 1.73;
    sensorHeight_ = 2.35;

    // rule-based filter params
    tHeightMin_ = 1.0;
    // tHeightMin_ = 1.2
    tHeightMax_ = 2.6;
    // tWidthMin_ = 0.5;
    // tWidthMin_ = 0.4;
    tWidthMin_ = 0.25;
    tWidthMax_ = 3.5;
    tLenMin_ = 0.5;
    tLenMax_ = 14.0;
    tAreaMax_ = 20.0;
    tRatioMin_ = 1.3;
    tRatioMax_ = 5.0;
    minLenRatio_ = 3.0;
    tPtNumPerVol_ = 8; // point c
}

void BoxFitting::getPointsInPcFrame(Point2f rectPoints[], vector<Point2f>& pcPoints, int offsetX, int offsetY){
    // loop 4 rect points
    for (int pointI = 0; pointI < 4; pointI++){
        float picX = rectPoints[pointI].x;
        float picY = rectPoints[pointI].y;
        // reverse offset
        float rOffsetX = picX - offsetX;
        float rOffsetY = picY - offsetY;
        // reverse from image coordinate to eucledian coordinate
        float rX = rOffsetX;
        float rY = picScale_*roiM_ - rOffsetY;
        // reverse to 30mx30m scale
        float rmX = rX/picScale_;
        float rmY = rY/picScale_;
        // reverse from (0 < x,y < 30) to (-15 < x,y < 15)
        float pcX = rmX - roiM_/2;
        float pcY = rmY - roiM_/2;
        Point2f point(pcX, pcY);
        pcPoints[pointI] = point;
    }
}

bool BoxFitting::ruleBasedFilter(vector<Point2f> pcPoints, float maxZ, int numPoints){
    bool isPromising = false;
    //minnimam points thresh
    if(numPoints < 100) return isPromising;
    // length is longest side of the rectangle while width is the shorter side.
    float width, length, height, area, ratio, mass;

    float x1 = pcPoints[0].x;
    float y1 = pcPoints[0].y;
    float x2 = pcPoints[1].x;
    float y2 = pcPoints[1].y;
    float x3 = pcPoints[2].x;
    float y3 = pcPoints[2].y;

    float dist1 = sqrt((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2));
    float dist2 = sqrt((x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2));
    if(dist1 > dist2){
        length = dist1;
        width = dist2;
    }
    else{
        length = dist2;
        width = dist1;
    }
    // assuming ground = sensor height
    height = maxZ + sensorHeight_;
    // assuming right angle
    area = dist1*dist2;
    mass = area*height;
    ratio = length/width;

    //start rule based filtering
    if(height > tHeightMin_ && height < tHeightMax_){
        if(width > tWidthMin_ && width < tWidthMax_){
            if(length > tLenMin_ && length < tLenMax_){
                if(area < tAreaMax_){
                    if(numPoints > mass*tPtNumPerVol_){
                        if(length > minLenRatio_){
                            if(ratio > tRatioMin_ && ratio < tRatioMax_){
                                isPromising = true;
                                return isPromising;
                            }
                        }
                        else{   
                            isPromising = true;
                            return isPromising;
                        }
                    }
                }
            }
        }
    }
    else return isPromising;
}

void BoxFitting::getBoundingBox(vector<PointCloud<PointXYZRGB>>  clusteredPoints,
                    double timestamp,
                    vector<PointCloud<PointXYZ>>& bbPoints){
    for (int iCluster = 0; iCluster < clusteredPoints.size(); iCluster++){
        // calculating offset so that shape fitting would be visualized nicely 
        Mat m (picScale_*roiM_, picScale_*roiM_, CV_8UC1, Scalar(0));
        float initPX = clusteredPoints[iCluster][0].x + roiM_/2;
        float initPY = clusteredPoints[iCluster][0].y + roiM_/2;
        int initX = floor(initPX*picScale_);
        int initY = floor(initPY*picScale_);
        int initPicX = initX;
        int initPicY = picScale_*roiM_ - initY;
        int offsetInitX = roiM_*picScale_/2 - initPicX;
        int offsetInitY = roiM_*picScale_/2 - initPicY;

        int numPoints = clusteredPoints[iCluster].size();
        vector<Point> pointVec(numPoints);
        vector<Point2f> pcPoints(4);
        float minMx, minMy, maxMx, maxMy;
        float minM = 999; float maxM = -999; float maxZ = -99;

        // for center of gravity
        // float sumX = 0; float sumY = 0;

        for (int iPoint = 0; iPoint < numPoints; iPoint++){
            float pX = clusteredPoints[iCluster][iPoint].x;
            float pY = clusteredPoints[iCluster][iPoint].y;
            float pZ = clusteredPoints[iCluster][iPoint].z;
            // cast (roiM_/2 < x,y < roiM_/2) into (0 < x,y < roiM_)
            float roiX = pX + roiM_/2;
            float roiY = pY + roiM_/2;
            // cast (roiM_)mx(roiM_)m into 900x900 scale
            int x = floor(roiX*picScale_);
            int y = floor(roiY*picScale_);
            // cast into image coordinate
            int picX = x;
            int picY = picScale_*roiM_ - y;
            // offset so that the object would be locate at the center
            int offsetX = picX + offsetInitX;
            int offsetY = picY + offsetInitY;
            m.at<uchar>(offsetY, offsetX) = 255;
            pointVec[iPoint] = Point(offsetX, offsetY);
            // calculate min and max slope for x1, x3(edge points)
            float m = pY/pX;
            if(m < minM) {
                minM = m;
                minMx = pX;
                minMy = pY;
            }
            if(m > maxM) {
                maxM = m;
                maxMx = pX;
                maxMy = pY;
            }

            //get maxZ
            if(pZ > maxZ) maxZ = pZ;

            // for center of gravity
            // sumX += offsetX;
            // sumY += offsetY; 

        }

        // L shape fitting parameters
        float xDist = maxMx - minMx;
        float yDist = maxMy - minMy;
        float slopeDist = sqrt(xDist*xDist + yDist*yDist);
        float slope = (maxMy - minMy)/(maxMx - minMx);

        // random variable
        mt19937_64 mt;
        mt.seed(timestamp);
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // start l shape fitting for car like object
        // lSlopeDist_ = 2.0m
        if(slopeDist > lSlopeDist_ && numPoints > lnumPoints_){
            float maxDist = 0;
            float maxDx, maxDy;

            // 80 random points, get max distance
            for(int i = 0; i < ramPoints_; i++){
                int pInd = randPoints(mt);
                assert(pInd >= 0 && pInd < clusteredPoints[iCluster].size());
                float xI = clusteredPoints[iCluster][pInd].x;
                float yI = clusteredPoints[iCluster][pInd].y;

                // from equation of distance between line and point
                float dist = abs(slope*xI-1*yI+maxMy-slope*maxMx)/sqrt(slope*slope + 1);
                if(dist > maxDist) {
                    maxDist = dist;
                    maxDx = xI;
                    maxDy = yI;
                }
            }

            // for center of gravity
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            float lastX = maxDx + maxMvecX + minMvecX;
            float lastY = maxDy + maxMvecY + minMvecY;

            pcPoints[0] = Point2f(minMx, minMy);
            pcPoints[1] = Point2f(maxDx, maxDy);
            pcPoints[2] = Point2f(maxMx, maxMy);
            pcPoints[3] = Point2f(lastX, lastY);
            bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            if(!isPromising) continue;
            // ------start visualization-----
            // cast (-15 < x,y < 15) into (0 < x,y < 30)
//            float a = maxMx + roiM_/2;
//            float b = maxMy + roiM_/2;
//            float c = minMx + roiM_/2;
//            float d = minMy + roiM_/2;
//            float e = maxDx + roiM_/2;
//            float f = maxDy + roiM_/2;
//            float g = lastX + roiM_/2;
//            float h = lastY + roiM_/2;
//            // cast 30mx30m into 900x900 scale
//            int aa = floor(a*picScale_);
//            int bb = floor(b*picScale_);
//            int cc = floor(c*picScale_);
//            int dd = floor(d*picScale_);
//            int ee = floor(e*picScale_);
//            int ff = floor(f*picScale_);
//            int gg = floor(g*picScale_);
//            int hh = floor(h*picScale_);
//            // cast into image coordinate
//            int aaa = aa;
//            int bbb = picScale_*roiM_ - bb;
//            int ccc = cc;
//            int ddd = picScale_*roiM_ - dd;
//            int eee = ee;
//            int fff = picScale_*roiM_ - ff;
//            int ggg = gg;
//            int hhh = picScale_*roiM_ - hh;
//            // offset so that the object would be locate at the center
//            int aaaa = aaa + offsetInitX;
//            int bbbb = bbb + offsetInitY;
//            int cccc = ccc + offsetInitX;
//            int dddd = ddd + offsetInitY;
//            int eeee = eee + offsetInitX;
//            int ffff = fff + offsetInitY;
//            int gggg = ggg + offsetInitX;
//            int hhhh = hhh + offsetInitY;
//
//            line( m, Point(aaaa, bbbb), Point(cccc, dddd), Scalar(255,255,0), 1, 8 );
//            line( m, Point(aaaa, bbbb), Point(eeee, ffff), Scalar(255,255,0), 1, 8 );
//            line( m, Point(cccc, dddd), Point(eeee, ffff), Scalar(255,255,0), 1, 8 );
//            line( m, Point(aaaa, bbbb), Point(gggg, hhhh), Scalar(255,255,0), 1, 8 );
//            line( m, Point(cccc, dddd), Point(gggg, hhhh), Scalar(255,255,0), 1, 8 );
//
//            imshow("Display Image", m);
//            waitKey(0);
            // --------end visualization -----------

        }
        else{
            //MAR fitting
            RotatedRect rectInfo = minAreaRect(pointVec);
            Point2f rectPoints[4]; rectInfo.points( rectPoints );
            // covert points back to lidar coordinate
            getPointsInPcFrame(rectPoints, pcPoints, offsetInitX, offsetInitY);
            // rule based filter
            bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            if(!isPromising) continue;
            // for visualization
//            for( int j = 0; j < 4; j++ )
//                line( m, rectPoints[j], rectPoints[(j+1)%4], Scalar(255,255,0), 1, 8 );
//            imshow("Display Image", m);
//            waitKey(0);
        }

        // make pcl cloud for 3d bounding box
        PointCloud<PointXYZ> oneBbox;
        for(int pclH = 0; pclH < 2; pclH++){
            for(int pclP = 0; pclP < 4; pclP++){
                PointXYZ o;
                o.x = pcPoints[pclP].x;
                o.y = pcPoints[pclP].y;
                if(pclH == 0) o.z = -sensorHeight_;
                else o.z = maxZ;
                oneBbox.push_back(o);
            }
        }
        bbPoints.push_back(oneBbox);
    }
}

vector<PointCloud<PointXYZ>> BoxFitting::getBBoxes(vector<PointCloud<PointXYZRGB>> clusteredPoints,
                                                   double timestamp){
    // fitting bbox by using minAreaRect or l-shape
    vector<PointCloud<PointXYZ>>  bbPoints;
    getBoundingBox(clusteredPoints, timestamp, bbPoints);
    return bbPoints;
}