#include "NNBF.h"
//#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>

using namespace std;

bool compareFunc(pair <unsigned long, float> &a, pair <unsigned long, float> &b) {
    return a.second < b.second;
}

NNBF::NNBF(pcl::PointCloud<PointType>::Ptr pts, float igridSize) {
    gridSize = igridSize;
    //searach for min and max (x,y,z) from pts
    xBeg = yBeg = zBeg = MAXFLOAT;
    float xMax = -MAXFLOAT;
    float yMax = -MAXFLOAT;
    float zMax = -MAXFLOAT;
    for (unsigned int i = 0; i < pts->points.size(); i++) {
        if (xBeg > pts->points[i].x) xBeg = pts->points[i].x;
        if (yBeg > pts->points[i].y) yBeg = pts->points[i].y;
        if (zBeg > pts->points[i].z) zBeg = pts->points[i].z;
        if (xMax < pts->points[i].x) xMax = pts->points[i].x;
        if (yMax < pts->points[i].y) yMax = pts->points[i].y;
        if (zMax < pts->points[i].z) zMax = pts->points[i].z;
    }
    xSize = (unsigned long) ((xMax - xBeg) / igridSize + 1);
    ySize = (unsigned long) ((yMax - yBeg) / igridSize + 1);
    zSize = (unsigned long) ((zMax - zBeg) / igridSize + 1);


    //create vector with points
    std::vector<Point> voxelGridTemp(xSize * ySize * zSize);

    //add points to vector
    unsigned long xIndex, yIndex, zIndex, Index;
    for (unsigned int i = 0; i < pts->points.size(); i++) {
        xIndex = (unsigned long) (pts->points[i].x / igridSize);
        yIndex = (unsigned long) (pts->points[i].y / igridSize);
        zIndex = (unsigned long) (pts->points[i].z / igridSize);
        Index = zIndex * xSize * ySize + yIndex * xSize + xIndex;
        voxelGridTemp[Index].x = pts->points[i].x;
        voxelGridTemp[Index].y = pts->points[i].y;
        voxelGridTemp[Index].z = pts->points[i].z;
        voxelGridTemp[Index].flag = 1;
    }
    voxelGrid = voxelGridTemp;

}

vector<Point> NNBF::nearestKSearch(const PointType &_pt, int numPoints, std::vector<int> &nhs, std::vector<float> &sqDist, float maxDist)
{
    NNBF::Point pt;
    pt.x = _pt.x;
    pt.y = _pt.y;
    pt.z = _pt.z;
    
    //calculate indexes to check
    unsigned long xIndexMin,xIndexMax,yIndexMin,yIndexMax,zIndexMin,zIndexMax;
    xIndexMin = (unsigned long)((pt.x-xBeg)/gridSize - maxDist/gridSize);
    yIndexMin = (unsigned long)((pt.y-yBeg)/gridSize - maxDist/gridSize);
    zIndexMin = (unsigned long)((pt.z-zBeg)/gridSize - maxDist/gridSize);
    xIndexMax = (unsigned long)((pt.x-xBeg)/gridSize + maxDist/gridSize);
    yIndexMax = (unsigned long)((pt.y-yBeg)/gridSize + maxDist/gridSize);
    zIndexMax = (unsigned long)((pt.z-zBeg)/gridSize + maxDist/gridSize);
    
    //constraints
    if(xIndexMin < 0) xIndexMin = 0;
    if(yIndexMin < 0) yIndexMin = 0;
    if(zIndexMin < 0) zIndexMin = 0;
    if(xIndexMax > xSize) xIndexMax = xSize;
    if(yIndexMax > ySize) yIndexMax = ySize;
    if(zIndexMax > zSize) zIndexMax = zSize;
    
    //brute force search
    unsigned long currentIndex;
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
                if(voxelGrid[currentIndex].flag == 0) continue;
                //calculate distance
                xdist = (pt.x-voxelGrid[currentIndex].x)*(pt.x-voxelGrid[currentIndex].x);
                ydist = (pt.y-voxelGrid[currentIndex].y)*(pt.y-voxelGrid[currentIndex].y);
                zdist = (pt.z-voxelGrid[currentIndex].z)*(pt.z-voxelGrid[currentIndex].z);
                IndexDistPair.first = currentIndex;
                IndexDistPair.second = xdist+ydist+zdist;
                V.push_back(IndexDistPair);
            }
        }
    }
    //search for the closest neighbours
    sort(V.begin(), V.end(), compareFunc);
    
    //print
    if(false)
    {
        cout<<"The closest neighbours of point: "<<pt.x<<" "<<pt.y<<" "<<pt.z<<" are:"<<endl;
        for(int i = 0; i < numPoints; i++)
        {
            Point a;
            a = voxelGrid[V[i].first];
            cout<<"Point: "<<a.x<<" "<<a.y<<" "<<a.z<<" with squared distance = "<<V[i].second<<endl;

        }
    }
    
    //create results vector
    std::vector<Point> results;
    for(int i = 0; i < numPoints; i++)
    {
        results.push_back(voxelGrid[V[i].first]);
    }
    
    return results;
}
