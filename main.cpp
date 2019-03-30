//NNBF algorithm

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <chrono>
#include <iostream>
#include <vector>
#include <ctime>
#include "NNBF.cpp"
typedef pcl::PointXYZI PointType;

// adds n random points with values 0-max_v to existing map of points 
pcl::PointCloud<PointType>::Ptr generate_points_map(int numberOfPoints, int max_v)
{
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());

    pcl::PointXYZI temp;
    for (unsigned i = 0; i < numberOfPoints; i++)
    {
        temp.x = max_v * rand () / (RAND_MAX + 1.0f);
        temp.y = max_v * rand () / (RAND_MAX + 1.0f);
        temp.z = max_v * rand () / (RAND_MAX + 1.0f);
        map->points.push_back(temp);
    }

    return map;
}

// filters map to create second map with only one point in each voxel
pcl::PointCloud<PointType>::Ptr create_filtered_map(pcl::PointCloud<PointType>::Ptr &map, float gridSize)
{
    pcl::PointCloud<PointType>::Ptr mapFiltered(new pcl::PointCloud<PointType>());

    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(gridSize, gridSize, gridSize);
    downSizeFilter.setInputCloud(map);
    downSizeFilter.filter(*mapFiltered);

    return mapFiltered;
}

// creates point with random values
PointType get_random_point(int max_v)
{
    PointType rand_p;

    // assign coordinates
    rand_p.x = rand() % max_v;
    rand_p.y = rand() % max_v;
    rand_p.z = rand() % max_v;

    return rand_p;
}

int main(){
    // create a kdTree
    srand (time (NULL));
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap(new pcl::KdTreeFLANN<PointType>());

    // create points map
    int numberOfPoints = 100000;
    int maxValue = 1000;
    pcl::PointCloud<PointType>::Ptr map = generate_points_map(numberPoints, maxValue);

    // voxel filter the map with specyfied grid size
    float gridSize = 10;
    pcl::PointCloud<PointType>::Ptr mapFiltered = filterMap(map, gridSize);

    // add points to kdTree
    kdtreeMap->setInputCloud(mapFiltered);

    // create NNBF class instance and insert
    NNBF* nnbf = new NNBF(mapFiltered, gridSizeTemp);

    // point for which search nearest neighbours
    PointType pointSel = get_random_point(maxValue);

    // number of points to find
    int K = 10;

    // brute force algorithm with exceptions handling    
    std::vector<int> lastCornerNeighbours(K);
    std::vector<float> pointSearchSqDis(K);
    try
    {
        // getting results by brute force algorithm
        std::vector<Point> BF_results = nnbf->nearestKSearch(pointSel,K,lastCornerNeighbours,pointSearchSqDis,250);
    }
    catch (const std::exception& e)
    {
        std::cout << "Failed using Brute Force algorithm \n";
        std::cout << e.what();
    }

    // KDT Algorithm with exceptions handling

    std::vector<int> lastCornerNeighbours2(K);
    std::vector<float> pointSearchSqDis2(K);

    try
    {
        // getting results by KDT algorithm
        std::vector<Point> KDT_results;
        Point temp_p;
        if ( kdtreeMap->nearestKSearch (pointSel, K, lastCornerNeighbours2, pointSearchSqDis2) > 0 )
        {
            for (size_t i = 0; i < lastCornerNeighbours2.size (); ++i)
            {
                temp_p.x = mapFiltered->points[ lastCornerNeighbours2[i] ].x;
                temp_p.y = mapFiltered->points[ lastCornerNeighbours2[i] ].y;
                temp_p.z = mapFiltered->points[ lastCornerNeighbours2[i] ].z;

                KDT_results.push_back(temp_p);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "Failed using KDT algorithm \n";
        std::cout << e.what();
    }

    //TODO: comparing results and creating bigger test
}
