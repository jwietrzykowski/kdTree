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

int main(){
    // create a kdTree
    srand (time (NULL));
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap(new pcl::KdTreeFLANN<PointType>());

    // map point cloud
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr mapFiltered(new pcl::PointCloud<PointType>());

    // insert points to map
    int numberPoints = 1000; //1 mil
    pcl::PointXYZI temp;
    for (unsigned i = 0; i < numberPoints; i++)
    {
        temp.x = 1000.0f * rand () / (RAND_MAX + 1.0f);
        temp.y = 1000.0f * rand () / (RAND_MAX + 1.0f);
        temp.z = 1000.0f * rand () / (RAND_MAX + 1.0f);
        map->points.push_back(temp);
    }

    // voxel filter the map with 0.4 grid size
    pcl::VoxelGrid<PointType> downSizeFilter;
    float gridSizeTemp = 10;
    downSizeFilter.setLeafSize(gridSizeTemp, gridSizeTemp, gridSizeTemp);
    downSizeFilter.setInputCloud(map);
    downSizeFilter.filter(*mapFiltered);

    // add points to kdTree
    kdtreeMap->setInputCloud(mapFiltered);

    // create NNBF class instance and insert
    NNBF* nnbf = new NNBF(mapFiltered, gridSizeTemp);
 
    // point for which search nearest neighbours
    PointType pointSel;

    // assign coordinates
    pointSel.x = 250;
    pointSel.y = 250;
    pointSel.z = 250;

    //number of points to find
    int K = 10;
    
    //brute force algorithm with time measurement
    
    std::vector<int> lastCornerNeighbours(K);
    std::vector<float> pointSearchSqDis(K);
    
    auto start = std::chrono::system_clock::now();
    
    nnbf->nearestKSearch(pointSel,K,lastCornerNeighbours,pointSearchSqDis,250);

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "elapsed time for brute force: " << elapsed_seconds.count() << "s\n";

    //KDT Algorithm with time measurement

    std::vector<int> lastCornerNeighbours2(K);
    std::vector<float> pointSearchSqDis2(K);

    // accept points within 1 m
    cout<<"KDTREE algorithm"<<endl;
    start = std::chrono::system_clock::now();
    if ( kdtreeMap->nearestKSearch (pointSel, K, lastCornerNeighbours2, pointSearchSqDis2) > 0 )
    {
        for (size_t i = 0; i < lastCornerNeighbours2.size (); ++i)
            std::cout << "    "  <<   mapFiltered->points[ lastCornerNeighbours2[i] ].x
                      << " " << mapFiltered->points[ lastCornerNeighbours2[i] ].y
                      << " " << mapFiltered->points[ lastCornerNeighbours2[i] ].z
                      << " (squared distance: " << pointSearchSqDis2[i] << ")" << std::endl;
    }

    end = std::chrono::system_clock::now();
    elapsed_seconds = end-start;
    std::cout << "elapsed time for KDT: " << elapsed_seconds.count() << "s\n";
}
