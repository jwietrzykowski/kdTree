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

    NNBF(const pcl::PointCloud<PointType>::ConstPtr &pts, float igridSize);

    std::vector<NNBF::Point> nearestKSearch(const PointType &pt, int numPoints,/* std::vector<int> &nhs, std::vector<float> &sqDist, */float maxDist = 1.0);

private:

    unsigned long compIndex(const PointType &pt);


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
