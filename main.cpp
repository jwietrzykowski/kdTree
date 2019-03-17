#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/impl/normal_3d.hpp>

#include <iostream>
#include <vector>
#include <ctime>


typedef pcl::PointXYZI PointType;

int main(){
    srand (time(NULL));

    // create a kdTree
    pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());

    // map point cloud
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr mapFiltered(new pcl::PointCloud<PointType>());

    // insert points to map
    // resize the map
    map->width = 1000;
    map->height = 1;
    map->points.resize(map->width * map->height);
    for (unsigned int i = 0; i < 1000; i++)
    {
        //map->points.push_back(rand (), rand (), rand ()));
        map->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
        map->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        map->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    }

    // voxel filter the map with 0.5 grid size
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(1,1,1);
    downSizeFilter.setInputCloud(map);
    downSizeFilter.filter(*mapFiltered);

    // add points to kdTree
    kdtree->setInputCloud(mapFiltered);

    // point for which search nearest neighbours
    PointType pointSel;

    // assign coordinates
    // ...
    pointSel.x = 100;
    pointSel.y = 100;
    pointSel.z = 100;
    //number of neighbours
    int K = 10;

    std::vector<int> lastCornerNeighbours(K);
    std::vector<float> pointSearchSqDis(K);

    int radius = 100;
    kdtree->nearestKSearch(pointSel, radius, lastCornerNeighbours, pointSearchSqDis);

    if ( kdtree->radiusSearch (pointSel, radius, lastCornerNeighbours, pointSearchSqDis) > 0 )
    {
        for (size_t i = 0; i < lastCornerNeighbours.size (); ++i)
            std::cout << "    "  <<   map->points[ lastCornerNeighbours[i] ].x
                      << " " << map->points[ lastCornerNeighbours[i] ].y
                      << " " << map->points[ lastCornerNeighbours[i] ].z
                      << " (squared distance: " << pointSearchSqDis[i] << ")" << std::endl;
    }
    // accept points within 1 m
}
