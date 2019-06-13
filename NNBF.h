#ifndef NNBF_H
#define NNBF_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*bool operator ==( const NNBF::Point & a, const NNBF::Point & b)
{
    if(a.x==b.x && a.y==b.y && a.z == b.z) return true;
    return false;
}*/

typedef pcl::PointXYZI PointType;
class NNBF {
public:
    //NNBF();
    struct Point {
        // 0 - empty, 1 - occupied
        int flag = 0;
        float x, y, z;
    };

    NNBF(float ixMin, float ixMax, float iyMin, float iyMax, float izMin, float izMax, float igridSize);

    NNBF(const pcl::PointCloud<PointType>::ConstPtr &pts, float igridSize);

    void setInputCloud(const pcl::PointCloud<PointType>::ConstPtr &pts, float ngridSize);

    void mergeCloud(const pcl::PointCloud<PointType>::ConstPtr &pts);

    std::vector<NNBF::Point> nearestKSearch(const PointType &pt, int numPoints, std::vector<int> &nhs, std::vector<float> &sqDist, float maxDist = 1.0);

private:

    inline unsigned long compIndex(const PointType &pt){
        unsigned long xIndex = (unsigned long) ((pt.x - xBeg)/ gridSize);
        unsigned long yIndex = (unsigned long) ((pt.y - yBeg)/ gridSize);
        unsigned long zIndex = (unsigned long) ((pt.z - zBeg)/ gridSize);

        return zIndex * xSize * ySize + yIndex * xSize + xIndex;
    }


    std::vector<Point> voxelGrid;

    float gridSize;

    float xBeg;
    float yBeg;
    float zBeg;
    unsigned long xSize;
    unsigned long ySize;
    unsigned long zSize;
};


#endif //LOAM_VELODYNE_NNTREE_HPP
