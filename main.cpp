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
#include <fstream>

using namespace std;

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

void TestAlgorithms(int numberOfPoints = 1000000, double minValue = 10, double maxValue = 1000,
                    float gridSize= 10, int K = 15, float Radius = 250)
{
    // create a kdTree
    srand (time (NULL));

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap(new pcl::KdTreeFLANN<PointType>());

    // create map points
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(minValue,maxValue);
    pcl::PointCloud<PointType>::Ptr map = generate_points_map(numberOfPoints, maxValue, generator, distribution);

    // voxel filter the map with specified grid size
    //float gridSize = 10;
    pcl::PointCloud<PointType>::Ptr mapFiltered = create_filtered_map(map, gridSize);

    // add points to kdTree
    kdtreeMap->setInputCloud(mapFiltered);

    // create NNBF class instance and insert
    NNBF* nnbf = new NNBF(mapFiltered, gridSize);

    int number_of_tests = 100;
    /*int number_of_matches = 0;*/
    double KDTDuration = 0;
    double NNBFDuration = 0;
    for (int i = 0; i < number_of_tests; i++) {
        // point for which search nearest neighbours
        PointType pointSel = get_random_point(maxValue, generator, distribution);

        // number of points to find
        //int K = 10;

        // brute force algorithm with exceptions handling
        std::vector<int> lastCornerNeighbours(K);
        std::vector<float> pointSearchSqDis(K);
        std::vector<NNBF::Point> BF_results;
        try {
            // getting results by brute force algorithm
            std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
            BF_results = nnbf->nearestKSearch(pointSel, K, Radius);
            std::chrono::steady_clock::time_point end1= std::chrono::steady_clock::now();
            NNBFDuration += std::chrono::duration_cast<std::chrono::microseconds>( end1 - begin1 ).count();
        }
        catch (const std::exception &e) {
            std::cout << "Failed using Brute Force algorithm \n";
            std::cout << e.what();
        }

        // KDT Algorithm with exceptions handling

        std::vector<int> lastCornerNeighbours2(K);
        std::vector<float> pointSearchSqDis2(K);

        std::vector<NNBF::Point> KDT_results;
        try {
            // getting results by KDT algorithm
            NNBF::Point temp_p;
            std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
            if (kdtreeMap->nearestKSearch(pointSel, K, lastCornerNeighbours2, pointSearchSqDis2) > 0) {
                for (size_t j = 0; j < lastCornerNeighbours2.size(); j++) {
                    temp_p.x = mapFiltered->points[lastCornerNeighbours2[j]].x;
                    temp_p.y = mapFiltered->points[lastCornerNeighbours2[j]].y;
                    temp_p.z = mapFiltered->points[lastCornerNeighbours2[j]].z;

                    KDT_results.push_back(temp_p);
                }
            }
            std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
            KDTDuration += std::chrono::duration_cast<std::chrono::microseconds>( end2 - begin2 ).count();
        }
        catch (const std::exception &e) {
            std::cout << "Failed using KDT algorithm \n";
            std::cout << e.what();
        }
    }
    cout<<"number of points = "<<numberOfPoints<< " min = "<<minValue << " max = "<<maxValue<<endl;
    cout<<"grid size = "<<gridSize<<" K = "<<K<<" Radius = "<<Radius<<endl;

    cout<<"NNBFDuration: "<<NNBFDuration<<endl;
    cout<<"KDTDuration: "<<KDTDuration<<endl;
}

int main()
{
    //numberOfPoint,min,max,gridSize,K,Radius
    TestAlgorithms(10000000,0,5000,15,10,60);
    ofstream myfile;
    myfile.open("tests_results.csv");
    myfile<<"test";
    myfile.close();

}