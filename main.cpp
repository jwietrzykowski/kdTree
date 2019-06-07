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
#include "NNBF.h"
#include <fstream>
#include <cmath>

using namespace std;

typedef pcl::PointXYZI PointType;

bool operator ==( const NNBF::Point & a, const NNBF::Point & b)
{
    static constexpr double eps = 1e-5;
    if(abs(a.x - b.x) < eps && (a.y - b.y) < eps && (a.z - b.z) < eps) return true;
    return false;
}

// adds n random points with values 0-max_v to existing map of points
pcl::PointCloud<PointType>::Ptr generatePointsMap(int numberOfPoints,
                                                  default_random_engine &generator,
                                                  uniform_real_distribution<double> &distribution,
                                                  double &duration)
{
    chrono::high_resolution_clock::time_point begin = chrono::high_resolution_clock::now();
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());

    pcl::PointXYZI temp;
    for (unsigned i = 0; i < numberOfPoints; i++)
    {
        temp.x = distribution(generator);
        temp.y = distribution(generator);
        temp.z = distribution(generator);
        map->points.push_back(temp);
    }

    chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::nanoseconds>( end - begin).count();
    return map;
}


// filters map to create second map with only one point in each voxel
pcl::PointCloud<PointType>::Ptr createFilteredMap(pcl::PointCloud<PointType>::Ptr &map,
                                                  float gridSize,
                                                  double &duration)
{
    chrono::high_resolution_clock::time_point begin = chrono::high_resolution_clock::now();
    pcl::PointCloud<PointType>::Ptr mapFiltered(new pcl::PointCloud<PointType>());

    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(gridSize, gridSize, gridSize);
    downSizeFilter.setInputCloud(map);
    downSizeFilter.filter(*mapFiltered);
    chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::nanoseconds>( end - begin).count();
    return mapFiltered;
}

// creates point with random values
PointType randomPoint(default_random_engine &generator, uniform_real_distribution<double> &distribution)
{
    PointType rand_p;

    // assign coordinates
    rand_p.x = distribution(generator);
    rand_p.y = distribution(generator);
    rand_p.z = distribution(generator);

    return rand_p;
}

int checkResults(vector<NNBF::Point> &NNBFresults, vector<NNBF::Point> &KDTresults)
{
    int output = 0;
    for(int i = 0; i < NNBFresults.size(); i++)
    {
        if(NNBFresults[i] == KDTresults[i])
        {
            output += 1;
        }
    }
    return output;
}


void test(int numMapPoints = 10000, double minValue = -25, double maxValue = 25,
          float gridSize = 0.4, int K = 10, float Radius = 1.2, int sort_method = 0) {
    default_random_engine generator;
    uniform_real_distribution<double> distribution(minValue, maxValue);

    int number_of_tests = 10;

    double kdtCreate = 0;
    double nnbfCreate = 0;
    double kdtSearch = 0;
    double nnbfSearch = 0;
    double temp;
    //int correct_points;
    int numSearchPoints = 10000;

    for (int t = 0; t < number_of_tests; t++) {
        vector<PointType> pointSelVect;
        for (int j = 0; j < numSearchPoints; j++) {
            pointSelVect.push_back(randomPoint(generator, distribution));
        }

        pcl::PointCloud<PointType>::Ptr map = generatePointsMap(numMapPoints, generator, distribution, temp);
        pcl::PointCloud<PointType>::Ptr mapFiltered = createFilteredMap(map, gridSize, temp);
        cout << "mapFileterd.size() = " << mapFiltered->size() << endl;


        NNBF* nnbf = new NNBF(minValue, maxValue, minValue, maxValue, minValue, maxValue, gridSize);
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap(new pcl::KdTreeFLANN<PointType>());

        vector<NNBF::Point> kdtResults;
        vector<NNBF::Point> nnbfResults;
        {
            chrono::high_resolution_clock::time_point nnbfCreateStart = chrono::high_resolution_clock::now();
            nnbf->setInputCloud(mapFiltered, gridSize);
            chrono::high_resolution_clock::time_point nnbfCreateEnd = chrono::high_resolution_clock::now();
            nnbfCreate += chrono::duration_cast<chrono::nanoseconds>(nnbfCreateEnd - nnbfCreateStart).count();

            chrono::high_resolution_clock::time_point kdtCreateStart = chrono::high_resolution_clock::now();
            kdtreeMap->setInputCloud(mapFiltered);
            chrono::high_resolution_clock::time_point kdtCreateEnd = chrono::high_resolution_clock::now();
            kdtCreate += chrono::duration_cast<chrono::nanoseconds>(kdtCreateEnd - kdtCreateStart).count();

            int incorrCnt = 0;
            for(int i = 0; i < pointSelVect.size(); i++) {

                kdtResults.clear();
                vector<int> lastCornerNeighbours2(K);
                vector<float> pointSearchSqDis2(K);
                NNBF::Point temp_p;
                chrono::high_resolution_clock::time_point kdtSearchStart = chrono::high_resolution_clock::now();
                if (kdtreeMap->nearestKSearch(pointSelVect[i], K, lastCornerNeighbours2, pointSearchSqDis2) > 0) {
                    for (size_t j = 0; j < lastCornerNeighbours2.size(); j++) {
                        if(pointSearchSqDis2[j] < Radius * Radius) {
                            temp_p.x = mapFiltered->points[lastCornerNeighbours2[j]].x;
                            temp_p.y = mapFiltered->points[lastCornerNeighbours2[j]].y;
                            temp_p.z = mapFiltered->points[lastCornerNeighbours2[j]].z;

                            kdtResults.push_back(temp_p);
                        }
                    }
                }
                chrono::high_resolution_clock::time_point kdtSearchEnd = chrono::high_resolution_clock::now();
                kdtSearch += chrono::duration_cast<chrono::nanoseconds>(kdtSearchEnd - kdtSearchStart).count();

                chrono::high_resolution_clock::time_point nnbfSearchStart = chrono::high_resolution_clock::now();
                nnbfResults = nnbf->nearestKSearch(pointSelVect[i], K, Radius,sort_method);
                chrono::high_resolution_clock::time_point nnbfSearchEnd = chrono::high_resolution_clock::now();
                nnbfSearch += chrono::duration_cast<chrono::nanoseconds>(nnbfSearchEnd - nnbfSearchStart).count();

                int correct_points = checkResults(nnbfResults, kdtResults);
                if(correct_points < kdtResults.size()){
                    cout << "ERROR, points do not match, nnbfResults.size() = " << nnbfResults.size()
                         << ", kdtResults.size() = " << kdtResults.size() << endl;
                    ++incorrCnt;
                }
            }
            cout << incorrCnt << " out of " << pointSelVect.size() << " incorrect" << endl;

            delete nnbf;
        }
    }

    cout << "kdtCreate = " << kdtCreate / 1e6 << " [ms]" << endl;
    cout << "nnbfCreate = " << nnbfCreate / 1e6 << " [ms]" << endl;
    cout << "kdtSearch = " << kdtSearch / 1e6 << " [ms]" << endl;
    cout << "nnbfSearch = " << nnbfSearch / 1e6 << " [ms]" << endl;
}

int main()
{
    //numberOfPoint,min,max,
    //gridSize,K,Radius

    test(1000000, -25, 25, 0.4, 5, 1.0);

    cout<<"KONIEC TESTOW"<<endl;
}