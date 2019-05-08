//NNBF algorithm

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <chrono>
#include <iostream>
#include <vector>
#include <ctime>
#include <random>
#include "NNBF.cpp"

typedef pcl::PointXYZI PointType;

// adds n random points with values 0-max_v to existing map of points
pcl::PointCloud<PointType>::Ptr generate_points_map(int numberOfPoints, int max_v,
        std::default_random_engine generator, std::uniform_real_distribution<double> distribution)
{
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());

    pcl::PointXYZI temp;
    for (unsigned i = 0; i < numberOfPoints; i++)
    {
        temp.x = distribution(generator);
        temp.y = distribution(generator);
        temp.z = distribution(generator);
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
PointType get_random_point(int max_v,std::default_random_engine generator,std::uniform_real_distribution<double> distribution)
{
    PointType rand_p;

    // assign coordinates
    rand_p.x = distribution(generator);
    rand_p.y = distribution(generator);
    rand_p.z = distribution(generator);

    return rand_p;
}

int main(){
    // create a kdTree
    srand (time (NULL));

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap(new pcl::KdTreeFLANN<PointType>());

    // create points map
    int numberOfPoints = 1000000;
    double maxValue = 1000;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(100.0,maxValue);
    pcl::PointCloud<PointType>::Ptr map = generate_points_map(numberOfPoints, maxValue, generator, distribution);

    // voxel filter the map with specified grid size
    float gridSize = 10;
    pcl::PointCloud<PointType>::Ptr mapFiltered = create_filtered_map(map, gridSize);

    // add points to kdTree
    kdtreeMap->setInputCloud(mapFiltered);

    // create NNBF class instance and insert
    NNBF* nnbf = new NNBF(mapFiltered, gridSize);

    int number_of_tests = 1000;
    int number_of_matches = 0;
    for (int i = 0; i < number_of_tests; i++)
    {
        // point for which search nearest neighbours
        PointType pointSel = get_random_point(maxValue, generator, distribution);

        // number of points to find
        int K = 10;

        // brute force algorithm with exceptions handling
        std::vector<int> lastCornerNeighbours(K);
        std::vector<float> pointSearchSqDis(K);
        std::vector<NNBF::Point> BF_results;
        try
        {
            // getting results by brute force algorithm
            BF_results = nnbf->nearestKSearch(pointSel,K,250);
        }
        catch (const std::exception& e)
        {
            std::cout << "Failed using Brute Force algorithm \n";
            std::cout << e.what();
        }

        // KDT Algorithm with exceptions handling

        std::vector<int> lastCornerNeighbours2(K);
        std::vector<float> pointSearchSqDis2(K);

        std::vector<NNBF::Point> KDT_results;
        try
        {
            // getting results by KDT algorithm
            NNBF::Point temp_p;
            if ( kdtreeMap->nearestKSearch (pointSel, K, lastCornerNeighbours2, pointSearchSqDis2) > 0 )
            {
                for (size_t j = 0; j < lastCornerNeighbours2.size (); j++)
                {
                    temp_p.x = mapFiltered->points[ lastCornerNeighbours2[j] ].x;
                    temp_p.y = mapFiltered->points[ lastCornerNeighbours2[j] ].y;
                    temp_p.z = mapFiltered->points[ lastCornerNeighbours2[j] ].z;

                    KDT_results.push_back(temp_p);
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "Failed using KDT algorithm \n";
            std::cout << e.what();
        }

        // comparing results
        if (KDT_results == BF_results)
            number_of_matches++;
    }

    std::cout << "Number of result matches: " << number_of_matches << " of " <<  number_of_tests << "\n";

}
