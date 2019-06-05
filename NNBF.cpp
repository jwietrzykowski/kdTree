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
    gridSize = igridSize;
    //searach for min and max (x,y,z) from pts
    xBeg = yBeg = zBeg = numeric_limits<float>::max();
    float xMax = numeric_limits<float>::lowest();
    float yMax = numeric_limits<float>::lowest();
    float zMax = numeric_limits<float>::lowest();
    for (unsigned int i = 0; i < pts->points.size(); i++) {
        if (xBeg > (float)((int)((pts->points[i].x)/gridSize-1))*gridSize) xBeg = (float)((int)((pts->points[i].x)/gridSize-1))*gridSize;
        if (yBeg > (float)((int)((pts->points[i].y)/gridSize-1))*gridSize) yBeg = (float)((int)((pts->points[i].y)/gridSize-1))*gridSize;
        if (zBeg > (float)((int)((pts->points[i].z)/gridSize-1))*gridSize) zBeg = (float)((int)((pts->points[i].z)/gridSize-1))*gridSize;
        if (xMax < (float)((int)((pts->points[i].x)/gridSize))*gridSize) xMax = (float)((int)((pts->points[i].x)/gridSize))*gridSize;
        if (yMax < (float)((int)((pts->points[i].y)/gridSize))*gridSize) yMax = (float)((int)((pts->points[i].y)/gridSize))*gridSize;
        if (zMax < (float)((int)((pts->points[i].z)/gridSize))*gridSize) zMax = (float)((int)((pts->points[i].z)/gridSize))*gridSize;
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
        if(voxelGrid[Index].flag == 1)
        {
            bool ustawiono = 1;

        }
        voxelGrid[Index].flag = 1;
    }
}

std::vector<NNBF::Point> NNBF::nearestKSearch(const PointType &_pt, int numPoints, float maxDist, int sort_method)
{
    NNBF::Point pt;
    pt.x = _pt.x;
    pt.y = _pt.y;
    pt.z = _pt.z;
    maxDist += this->gridSize;
    //calculate indexes to check
    unsigned long xIndexMin,xIndexMax,yIndexMin,yIndexMax,zIndexMin,zIndexMax;

    xIndexMin = (unsigned long)((pt.x - xBeg - maxDist)/gridSize);
    yIndexMin = (unsigned long)((pt.y - yBeg - maxDist)/gridSize);
    zIndexMin = (unsigned long)((pt.z - zBeg - maxDist)/gridSize);
    xIndexMax = (unsigned long)((pt.x - xBeg + maxDist)/gridSize ) + 1;
    yIndexMax = (unsigned long)((pt.y - yBeg + maxDist)/gridSize ) + 1;
    zIndexMax = (unsigned long)((pt.z - zBeg + maxDist)/gridSize ) + 1;

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
    std::vector<pair <double, unsigned long>> V;
    pair <double, unsigned long> IndexDistPair;
    double xdist,ydist,zdist;

    for(unsigned long i = xIndexMin; i < xIndexMax; i++)
    {
        for(unsigned long j = yIndexMin; j < yIndexMax; j++)
        {
            for(unsigned long k = zIndexMin; k < zIndexMax; k++)
            {
                currentIndex = xSize*ySize * k + xSize * j + i;
                if(voxelGrid[currentIndex].flag == 1)
                {
                    //calculate distance
                    xdist = (pt.x - voxelGrid[currentIndex].x) * (pt.x - voxelGrid[currentIndex].x);
                    ydist = (pt.y - voxelGrid[currentIndex].y) * (pt.y - voxelGrid[currentIndex].y);
                    zdist = (pt.z - voxelGrid[currentIndex].z) * (pt.z - voxelGrid[currentIndex].z);
                    IndexDistPair.first = xdist + ydist + zdist;
                    IndexDistPair.second = currentIndex;
                    V.push_back(IndexDistPair);
                }
            }
        }
    }
    // TODO Sortowanie moze byc waskim gardlem tego rozwiazania. ok

    std::vector<Point> results;
    //queue solution
    //===============================================================================
    if(sort_method == 0)
    {
        std::priority_queue<pair<double, unsigned long>, vector<pair<double, unsigned long>>, Compare> Qout;
        for (int i = 0; i < V.size(); i++) {
            if (i < numPoints) {
                Qout.push(V[i]);
            } else {
                if (V[i].first < Qout.top().first) {
                    Qout.push(V[i]);
                    Qout.pop();
                }
            }
        }
        int loopsize = Qout.size();
        for (int i = 0; i < loopsize; i++) {
            results.push_back(voxelGrid[Qout.top().second]);
            Qout.pop();
        }
        reverse(results.begin(), results.end());
    }
    //vector solution
    //===============================================================================
    if(sort_method == 1)
    {
        vector<pair<double, unsigned long>> Vout;
        for (int i = 0; i < V.size(); i++)
        {
            Vout.push_back(V[i]);
        }
        sort(Vout.begin(), Vout.end());
        //create results vector
        for (int i = 0; i < numPoints; i++)
        {
            if(i >= Vout.size())
            {
                break;
            }
            results.push_back(voxelGrid[Vout[i].second]);

        }
    }
    return results;
}