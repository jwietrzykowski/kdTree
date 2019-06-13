#include "NNBF.h"
//#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>
#include "math.h"
#include "queue"
using namespace std;
//Compare class to make reverse order in queue
class Compare
{
public:
    bool operator() (const pair <double, unsigned long> &a,const pair <double, unsigned long> &b)
    {
        return a.first < b.first;
    }
};

NNBF::NNBF(const pcl::PointCloud<PointType>::ConstPtr &pts, float igridSize)
{
    setInputCloud(pts, igridSize);
}

NNBF::NNBF(float ixMin, float ixMax, float iyMin, float iyMax, float izMin, float izMax, float igridSize) {

    gridSize = igridSize;
    //searach for min and max (x,y,z) from pts
    xBeg = ixMin;
    yBeg = iyMin;
    zBeg = izMin;

    xSize = (unsigned long) ((ixMax - xBeg) / gridSize + 1);
    ySize = (unsigned long) ((iyMax - yBeg) / gridSize + 1);
    zSize = (unsigned long) ((izMax - zBeg) / gridSize + 1);

    //create vector with points
    voxelGrid.resize(xSize * ySize * zSize);

//    memset(voxelGrid.data(), 0, voxelGrid.size() * sizeof(Point));
}


void NNBF::setInputCloud(const pcl::PointCloud<PointType>::ConstPtr &pts, float ngridSize) {

    gridSize = ngridSize;
    //searach for min and max (x,y,z) from pts
    xBeg = yBeg = zBeg = numeric_limits<float>::max();
    float xMax = numeric_limits<float>::lowest();
    float yMax = numeric_limits<float>::lowest();
    float zMax = numeric_limits<float>::lowest();
    for (unsigned int i = 0; i < pts->points.size(); i++) {
        float xCoord = floor(pts->points[i].x/gridSize) * gridSize;
        float yCoord = floor(pts->points[i].y/gridSize) * gridSize;
        float zCoord = floor(pts->points[i].z/gridSize) * gridSize;

        if (xBeg > xCoord) xBeg = xCoord;
        if (yBeg > yCoord) yBeg = yCoord;
        if (zBeg > zCoord) zBeg = zCoord;
        if (xMax < xCoord) xMax = xCoord;
        if (yMax < yCoord) yMax = yCoord;
        if (zMax < zCoord) zMax = zCoord;
    }

    xSize = (unsigned long) ((xMax - xBeg) / gridSize + 1);
    ySize = (unsigned long) ((yMax - yBeg) / gridSize + 1);
    zSize = (unsigned long) ((zMax - zBeg) / gridSize + 1);

    if(xSize * ySize * zSize > voxelGrid.size()){
        cout << "Reallocating" << endl;
        voxelGrid.resize(xSize * ySize * zSize);
    }

    memset(voxelGrid.data(), 0, xSize * ySize * zSize * sizeof(Point));

    //add points to vector
    for(unsigned int i = 0; i < pts->points.size(); i++) {
        unsigned long idx = compIndex(pts->points[i]);
        voxelGrid[idx].x = pts->points[i].x;
        voxelGrid[idx].y = pts->points[i].y;
        voxelGrid[idx].z = pts->points[i].z;

        voxelGrid[idx].flag = 1;
    }
}

void NNBF::mergeCloud(const pcl::PointCloud<PointType>::ConstPtr &pts) {
    for(unsigned int i = 0; i < pts->points.size(); i++) {
        unsigned long idx = compIndex(pts->points[i]);

        if(voxelGrid[idx].flag == 0){
            voxelGrid[idx].x = pts->points[i].x;
            voxelGrid[idx].y = pts->points[i].y;
            voxelGrid[idx].z = pts->points[i].z;

            voxelGrid[idx].flag = 1;
        }
        else if(voxelGrid[idx].flag == 1) {
            // assuming only one point per voxel in new cloud, so simple average of both points is enough
            voxelGrid[idx].x = (voxelGrid[idx].x + pts->points[i].x) / 2;
            voxelGrid[idx].y = (voxelGrid[idx].y + pts->points[i].y) / 2;
            voxelGrid[idx].z = (voxelGrid[idx].z + pts->points[i].z) / 2;
        }
    }
}

std::vector<NNBF::Point> NNBF::nearestKSearch(const PointType &_pt, int numPoints, std::vector<int> &nhs, std::vector<float> &sqDist, float maxDist)
{
    NNBF::Point pt;
    pt.x = _pt.x;
    pt.y = _pt.y;
    pt.z = _pt.z;

    nhs.clear();
    sqDist.clear();

    //calculate indexes to check
    unsigned long xIndexMin,xIndexMax,yIndexMin,yIndexMax,zIndexMin,zIndexMax;

    xIndexMin = (unsigned long)(std::max((pt.x - xBeg - maxDist)/gridSize, 0.0f));
    yIndexMin = (unsigned long)(std::max((pt.y - yBeg - maxDist)/gridSize, 0.0f));
    zIndexMin = (unsigned long)(std::max((pt.z - zBeg - maxDist)/gridSize, 0.0f));
    xIndexMax = (unsigned long)(std::min((pt.x - xBeg + maxDist)/gridSize + 1, (float)xSize));
    yIndexMax = (unsigned long)(std::min((pt.y - yBeg + maxDist)/gridSize + 1, (float)ySize));
    zIndexMax = (unsigned long)(std::min((pt.z - zBeg + maxDist)/gridSize + 1, (float)zSize));


    //brute force search
//    unsigned long currentIndex;
//    pair <float, unsigned long> distIdx;
//    float xdist,ydist,zdist;

    priority_queue<pair<float, unsigned long>> qu;
    for(unsigned long i = xIndexMin; i < xIndexMax; i++)
    {
        for(unsigned long j = yIndexMin; j < yIndexMax; j++)
        {
            for(unsigned long k = zIndexMin; k < zIndexMax; k++)
            {
                unsigned long currentIndex = xSize*ySize * k + xSize * j + i;
                if(voxelGrid[currentIndex].flag == 1)
                {
                    //calculate distance
                    float xdist = (pt.x - voxelGrid[currentIndex].x) * (pt.x - voxelGrid[currentIndex].x);
                    float ydist = (pt.y - voxelGrid[currentIndex].y) * (pt.y - voxelGrid[currentIndex].y);
                    float zdist = (pt.z - voxelGrid[currentIndex].z) * (pt.z - voxelGrid[currentIndex].z);
                    pair <float, unsigned long> distIdx(xdist + ydist + zdist, currentIndex);

                    if(distIdx.first < maxDist*maxDist) {
                        if(qu.size() >= numPoints) {
                            if (qu.top().first > distIdx.first) {
                                qu.pop();
                                qu.push(distIdx);
                            }
                        }
                        else {
                            qu.push(distIdx);
                        }

                    }
                }
            }
        }
    }

    std::vector<Point> results;

    while(!qu.empty()){
        results.push_back(voxelGrid[qu.top().second]);
        sqDist.push_back(qu.top().first);
        qu.pop();
    }
    reverse(results.begin(), results.end());

    return results;
}



