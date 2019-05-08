#include "NNBF.h"
//#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>
#include "math.h"
#include "queue"
using namespace std;

//not used anymore
/*bool compareFunc(pair <unsigned long, float> &a, pair <unsigned long, float> &b) {
    return a.second < b.second;
}*/
bool operator ==( const NNBF::Point & a, const NNBF::Point & b)
{
    if(a.x==b.x && a.y==b.y && a.z == b.z) return true;
    return false;
}
//for queue to have "worst" point on top to pop
bool operator < (pair <unsigned long, float> &a, pair <unsigned long, float> &b){
    return a.second > b.second;
}

NNBF::NNBF(const pcl::PointCloud<PointType>::ConstPtr &pts, float igridSize)
{
    gridSize = igridSize;
    //searach for min and max (x,y,z) from pts
    xBeg = yBeg = zBeg = numeric_limits<float>::max();
    float xMax = numeric_limits<float>::lowest();
    float yMax = numeric_limits<float>::lowest();
    float zMax = numeric_limits<float>::lowest();
    for (unsigned int i = 0; i < pts->points.size(); i++) {
        if (xBeg > pts->points[i].x) xBeg = pts->points[i].x;
        if (yBeg > pts->points[i].y) yBeg = pts->points[i].y;
        if (zBeg > pts->points[i].z) zBeg = pts->points[i].z;
        if (xMax < pts->points[i].x) xMax = pts->points[i].x;
        if (yMax < pts->points[i].y) yMax = pts->points[i].y;
        if (zMax < pts->points[i].z) zMax = pts->points[i].z;
    }
    // TODO Dobrze by bylo przedtem zaokraglic w dol xBeg do wielokrotnosci igridSize
    // TODO i zaokraglic w gore xMax
    xSize = (unsigned long) ((xMax - xBeg) / igridSize + 1);
    ySize = (unsigned long) ((yMax - yBeg) / igridSize + 1);
    zSize = (unsigned long) ((zMax - zBeg) / igridSize + 1);

    //create vector with points
    voxelGrid.resize(xSize * ySize * zSize);

    //add points to vector
    unsigned long xIndex, yIndex, zIndex, Index;
    for (unsigned int i = 0; i < pts->points.size(); i++) {
        xIndex = (unsigned long) ((pts->points[i].x - xBeg)/ igridSize);
        yIndex = (unsigned long) ((pts->points[i].y - yBeg)/ igridSize);
        zIndex = (unsigned long) ((pts->points[i].z - zBeg)/ igridSize);
        Index = zIndex * xSize * ySize + yIndex * xSize + xIndex;
        voxelGrid[Index].x = pts->points[i].x;
        voxelGrid[Index].y = pts->points[i].y;
        voxelGrid[Index].z = pts->points[i].z;
        voxelGrid[Index].flag = 1;
    }
}

std::vector<NNBF::Point> NNBF::nearestKSearch(const PointType &_pt, int numPoints, /*std::vector<int> &nhs, std::vector<float> &sqDist, */float maxDist)
{
    NNBF::Point pt;
    pt.x = _pt.x;
    pt.y = _pt.y;
    pt.z = _pt.z;

    //calculate indexes to check
    unsigned long xIndexMin,xIndexMax,yIndexMin,yIndexMax,zIndexMin,zIndexMax;

    xIndexMin = (unsigned long)((pt.x-xBeg - maxDist)/gridSize);
    yIndexMin = (unsigned long)((pt.y-yBeg - maxDist)/gridSize);
    zIndexMin = (unsigned long)((pt.z-zBeg - maxDist)/gridSize);
    xIndexMax = (unsigned long)((pt.x-xBeg + maxDist)/gridSize ) + 1;
    yIndexMax = (unsigned long)((pt.y-yBeg + maxDist)/gridSize ) + 1;
    zIndexMax = (unsigned long)((pt.z-zBeg + maxDist)/gridSize ) + 1;

    //constraints
    if(xIndexMin < 0 || xIndexMin >= 2 * xSize) xIndexMin = 0;
    if(yIndexMin < 0 || yIndexMin >= 2 * ySize) yIndexMin = 0;
    if(zIndexMin < 0 || zIndexMin >= 2 * zSize) zIndexMin = 0;
    if(xIndexMax > xSize) xIndexMax = xSize;
    if(yIndexMax > ySize) yIndexMax = ySize;
    if(zIndexMax > zSize) zIndexMax = zSize;

    //brute force search
    unsigned long currentIndex;
    //V is a vector with index and distance squared from selected point
    std::vector<pair <unsigned long, float>> V;
    pair <unsigned long, float> IndexDistPair;
    float xdist,ydist,zdist;

    for(unsigned long i = xIndexMin; i < xIndexMax; i++)
    {
        for(unsigned long j = yIndexMin; j < yIndexMax; j++)
        {
            for(unsigned long k = zIndexMin; k < zIndexMax; k++)
            {
                //if voxel is empty -> skip iteration
                currentIndex = xSize*ySize * k + xSize * j + i;
                if(voxelGrid[currentIndex].flag == 1)
                {
                    //calculate distance
                    xdist = (pt.x - voxelGrid[currentIndex].x) * (pt.x - voxelGrid[currentIndex].x);
                    ydist = (pt.y - voxelGrid[currentIndex].y) * (pt.y - voxelGrid[currentIndex].y);
                    zdist = (pt.z - voxelGrid[currentIndex].z) * (pt.z - voxelGrid[currentIndex].z);
                    IndexDistPair.first = currentIndex;
                    IndexDistPair.second = xdist + ydist + zdist;
                    V.push_back(IndexDistPair);
                }
            }
        }
    }
    // TODO Sortowanie moze byc waskim gardlem tego rozwiazania. ok
    //search for the closest neighbours
    //sort(V.begin(), V.end(), compareFunc);
    std::priority_queue<pair <unsigned long, float>> Q;
    for(int i = 0; i < min(int(V.size()),numPoints); i++)
    {
        if(i<numPoints)
        {
            Q.push(V[i]);
        }
        else
        {
            if(Q.top().second > V[i].second)
            {
                Q.pop();
                Q.push(V[i]);
            }
        }
    }

    //create results vector
    std::vector<Point> results;
    for(int i = 0; i < Q.size(); i++)
    {
        results.push_back(voxelGrid[Q.top().first]);
        Q.pop();
    }

    return results;
}