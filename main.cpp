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
pcl::PointCloud<PointType>::Ptr generate_points_map(int numberOfPoints,
        default_random_engine &generator, uniform_real_distribution<double> &distribution,
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
pcl::PointCloud<PointType>::Ptr create_filtered_map(pcl::PointCloud<PointType>::Ptr &map, float gridSize, double &duration)
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
PointType get_random_point(default_random_engine &generator, uniform_real_distribution<double> &distribution)
{
    PointType rand_p;

    // assign coordinates
    rand_p.x = distribution(generator);
    rand_p.y = distribution(generator);
    rand_p.z = distribution(generator);

    return rand_p;
}

vector<double> Test_KDT(int K, vector<NNBF::Point> &resultsVector,pcl::PointCloud<PointType>::Ptr mapFiltered,
                vector<PointType> pointSelVect)
{
    //start clock
    chrono::high_resolution_clock::time_point begin1 = chrono::high_resolution_clock::now();
    //generate points and filter
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap(new pcl::KdTreeFLANN<PointType>());
    kdtreeMap->setInputCloud(mapFiltered);
    vector<int> lastCornerNeighbours2(K);
    vector<float> pointSearchSqDis2(K);
    NNBF::Point temp_p;
    chrono::high_resolution_clock::time_point begin2 = chrono::high_resolution_clock::now();
    for(int i = 0; i < pointSelVect.size(); i++)
    if (kdtreeMap->nearestKSearch(pointSelVect[i], K, lastCornerNeighbours2, pointSearchSqDis2) > 0)
    {
        for (size_t j = 0; j < kdtreeMap->nearestKSearch(pointSelVect[i], K, lastCornerNeighbours2, pointSearchSqDis2); j++)
        {
            temp_p.x = mapFiltered->points[lastCornerNeighbours2[j]].x;
            temp_p.y = mapFiltered->points[lastCornerNeighbours2[j]].y;
            temp_p.z = mapFiltered->points[lastCornerNeighbours2[j]].z;

            resultsVector.push_back(temp_p);
        }
    }
    //end clock
    chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
    vector<double> Vout;
    Vout.push_back(chrono::duration_cast<chrono::milliseconds>( end - begin1).count());
    Vout.push_back(chrono::duration_cast<chrono::milliseconds>( end - begin2).count());
    return Vout;
}

vector<double> Test_NNBF(int K, float Radius, float gridSize, vector<NNBF::Point> &resultsVector,pcl::PointCloud<PointType>::Ptr mapFiltered,
                 vector<PointType> pointSelVect, int sort_method)
{
    //start clock
    chrono::high_resolution_clock::time_point begin1 = chrono::high_resolution_clock::now();
    NNBF* nnbf = new NNBF(mapFiltered, gridSize);
    chrono::high_resolution_clock::time_point begin2 = chrono::high_resolution_clock::now();
    for(int i = 0; i < pointSelVect.size(); i++)
        resultsVector = nnbf->nearestKSearch(pointSelVect[i], K, Radius,sort_method);
    //end clock
    chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
    delete nnbf;
    vector<double> Vout;
    Vout.push_back(chrono::duration_cast<chrono::milliseconds>( end - begin1).count());
    Vout.push_back(chrono::duration_cast<chrono::milliseconds>( end - begin2).count());
    return Vout;

}

int Check_Results(vector<NNBF::Point> &NNBFresults, vector<NNBF::Point> &KDTresults)
{
    int output = 0;
    for(int i = 0; i < NNBFresults.size(); i++)
    {
        /*float kdistx, kdisty, kdistz;
        kdistx = (p.x - KDTresults[i].x)*(p.x - KDTresults[i].x);
        kdisty = (p.y - KDTresults[i].y)*(p.y - KDTresults[i].y);
        kdistz = (p.z - KDTresults[i].z)*(p.z - KDTresults[i].z);

        float ndistx, ndisty, ndistz;
        ndistx = (p.x - NNBFresults[i].x)*(p.x - NNBFresults[i].x);
        ndisty = (p.y - NNBFresults[i].y)*(p.y - NNBFresults[i].y);
        ndistz = (p.z - NNBFresults[i].z)*(p.z - NNBFresults[i].z);*/

        //cout<<"KDT DIST = " <<kdistx+kdisty+kdistz << " NNBF DIST = " << ndistx+ndisty+ndistz<<endl;
        if(NNBFresults[i] == KDTresults[i])
        {
            output += 1;
        }
    }
    return output;
}

void TestAlgorithms(int numberOfPoints = 10000, double minValue = -25, double maxValue = 25,
                    float gridSize= 0.4, int K = 10, float Radius = 1.2, int sort_method = 0)
{
    // create a kdTree
//    srand (time (NULL));
    default_random_engine generator;
    uniform_real_distribution<double> distribution(minValue,maxValue);

    int number_of_tests = 1;
    double KDTDurationFull = 0;
    double NNBFDurationFull = 0;
    double KDTDurationPart = 0;
    double NNBFDurationPart = 0;
    double commonDuration = 0;
    double temp;
    vector<NNBF::Point> KDT_results;
    vector<NNBF::Point> NNBF_results;
    //int correct_points;
    int number_of_points = 1000;
    vector<PointType> pointSelVect;
    for (int j = 0; j < number_of_points; j++)
    {
        pointSelVect.push_back(get_random_point(generator, distribution));
    }
    for (int i = 0; i < number_of_tests; i++)
    {
        pcl::PointCloud<PointType>::Ptr map = generate_points_map(numberOfPoints, generator, distribution, temp);
        commonDuration += temp;
        pcl::PointCloud<PointType>::Ptr mapFiltered = create_filtered_map(map, 0.4, temp);
        commonDuration += temp;
        KDTDurationFull += Test_KDT(K, KDT_results, mapFiltered, pointSelVect)[0];
        NNBFDurationFull += Test_NNBF(K, Radius, gridSize, NNBF_results, mapFiltered, pointSelVect, sort_method)[0];
        KDTDurationPart += Test_KDT(K, KDT_results, mapFiltered, pointSelVect)[1];
        NNBFDurationPart += Test_NNBF(K, Radius, gridSize, NNBF_results, mapFiltered, pointSelVect, sort_method)[1];
//        int correct_points = Check_Results(NNBF_results, KDT_results);
//        if(correct_points < KDT_results.size()){
//            cout << endl << endl << "ERROR, points do not match" << endl << endl;
//        }
    }
    cout << KDTDurationFull << "," << NNBFDurationFull << "," << KDTDurationPart << "," << NNBFDurationPart << "," <<commonDuration /*<< correct_points << " of " << NNBF_results.size() */<< endl;
}

void TestAlgorithms2(int numMapPoints = 10000, double minValue = -25, double maxValue = 25,
                    float gridSize= 0.4, int K = 10, float Radius = 1.2, int sort_method = 0) {
    default_random_engine generator;
    uniform_real_distribution<double> distribution(minValue, maxValue);

    int number_of_tests = 1;

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
            pointSelVect.push_back(get_random_point(generator, distribution));
        }

        pcl::PointCloud<PointType>::Ptr map = generate_points_map(numMapPoints, generator, distribution, temp);
        pcl::PointCloud<PointType>::Ptr mapFiltered = create_filtered_map(map, gridSize, temp);
        cout << "mapFileterd.size() = " << mapFiltered->size() << endl;


        NNBF* nnbf = new NNBF(minValue, maxValue, minValue, maxValue, minValue, maxValue, gridSize);
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeMap(new pcl::KdTreeFLANN<PointType>());

        vector<NNBF::Point> KDT_results;
        vector<NNBF::Point> NNBF_results;
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

                KDT_results.clear();
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

                            KDT_results.push_back(temp_p);
                        }
                    }
                }
                chrono::high_resolution_clock::time_point kdtSearchEnd = chrono::high_resolution_clock::now();
                kdtSearch += chrono::duration_cast<chrono::nanoseconds>(kdtSearchEnd - kdtSearchStart).count();

                chrono::high_resolution_clock::time_point nnbfSearchStart = chrono::high_resolution_clock::now();
                NNBF_results = nnbf->nearestKSearch(pointSelVect[i], K, Radius,sort_method);
                chrono::high_resolution_clock::time_point nnbfSearchEnd = chrono::high_resolution_clock::now();
                nnbfSearch += chrono::duration_cast<chrono::nanoseconds>(nnbfSearchEnd - nnbfSearchStart).count();

                int correct_points = Check_Results(NNBF_results, KDT_results);
                if(correct_points < KDT_results.size()){
                    cout << "ERROR, points do not match, NNBF_results.size() = " << NNBF_results.size()
                         << ", KDT_results.size() = " << KDT_results.size() << endl;
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

    TestAlgorithms2(1000000, -125, 125, 0.4, 5, 1.0);

//    cout<<"Zmiana liczby punktow"<<endl;
//    cout<<"METODA SORTOWANIA: KOLEJKA"<<endl;
//    TestAlgorithms(1,-25,25,0.4,10, 1.2);
//    TestAlgorithms(10,-25,25,0.4,10,1.2);
//    TestAlgorithms(100,-25,25,0.4,10,1.2);
//    TestAlgorithms(1000,-25,25,0.4,10,1.2);
//    TestAlgorithms(10000,-25,25,0.4,10,1.2);
//    TestAlgorithms(100000,-25,25,0.4,10,1.2);
//    TestAlgorithms(1000000,-25,25,0.4,10,1.2);
//    TestAlgorithms(10000000,-25,25,0.4,10,1.2);
//    TestAlgorithms(100000000,-25,25,0.4,10,1.2);
//    cout<<"Zmiana rozmiarow przestrzeni"<<endl;
//    TestAlgorithms(10000,-1.2,1.2,0.4,10,1.2);
//    TestAlgorithms(10000,-2.4,2.4,0.4,10,1.2);
//    TestAlgorithms(10000,-3.6,3.6,0.4,10,1.2);
//    TestAlgorithms(10000,-5,5,0.4,10,1.2);
//    TestAlgorithms(10000,-10,10,0.4,10,1.2);
//    TestAlgorithms(10000,-15,15,0.4,10,1.2);
//    TestAlgorithms(10000,-25,25,0.4,10,1.2);
//    TestAlgorithms(10000,-35,35,0.4,10,1.2);
//    TestAlgorithms(10000,-50,50,0.4,10,1.2);
//    cout<<"Zmiana liczby szukanych sasiadow"<<endl;
//    TestAlgorithms(100000,-25,25,0.4,1,1.2);
//    TestAlgorithms(100000,-25,25,0.4,2,1.2);
//    TestAlgorithms(100000,-25,25,0.4,3,1.2);
//    TestAlgorithms(100000,-25,25,0.4,4,1.2);
//    TestAlgorithms(100000,-25,25,0.4,5,1.2);
//    TestAlgorithms(100000,-25,25,0.4,10,1.2);
//    TestAlgorithms(100000,-25,25,0.4,15,1.2);
//    TestAlgorithms(100000,-25,25,0.4,20,1.2);
//    TestAlgorithms(100000,-25,25,0.4,25,1.2);
//    TestAlgorithms(100000,-25,25,0.4,50,1.2);
//    TestAlgorithms(100000,-25,25,0.4,100,1.2);
//    TestAlgorithms(100000,-25,25,0.4,200,1.2);
//    TestAlgorithms(100000,-25,25,0.4,350,1.2);
//    TestAlgorithms(100000,-25,25,0.4,500,1.2);
    //TestAlgorithms(100000,-25,25,0.4,1000,1.2);
    //TestAlgorithms(100000,-25,25,0.4,2000,1.2);
    //TestAlgorithms(100000,-25,25,0.4,4000,1.2);
    //TestAlgorithms(100000,-25,25,0.4,8000,1.2);

//    cout<<"METODA SORTOWANIA: WEKTOR"<<endl;
//    cout<<"Zmiana liczby punktow"<<endl;
//    TestAlgorithms(1,-25,25,0.4,10, 1.2 ,1);
//    TestAlgorithms(10,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(100,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(1000,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(1000000,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(10000000,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(100000000,-25,25,0.4,10,1.2 ,1);
//    cout<<"Zmiana rozmiarow przestrzeni"<<endl;
//    TestAlgorithms(10000,-1.2,1.2,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-2.4,2.4,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-3.6,3.6,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-5,5,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-10,10,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-15,15,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-20,20,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-35,35,0.4,10,1.2 ,1);
//    TestAlgorithms(10000,-50,50,0.4,10,1.2 ,1);
//    cout<<"Zmiana liczby szukanych sasiadow"<<endl;
//    TestAlgorithms(100000,-25,25,0.4,1,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,2,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,3,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,4,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,5,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,10,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,15,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,20,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,25,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,50,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,100,1.2 ,1);
//    TestAlgorithms(100000,-25,25,0.4,200,1.2);
//    TestAlgorithms(100000,-25,25,0.4,350,1.2);
//    TestAlgorithms(100000,-25,25,0.4,500,1.2 ,1);
    //TestAlgorithms(100000,-25,25,0.4,1000,1.2 ,1);
    //TestAlgorithms(100000,-25,25,0.4,2000,1.2 ,1);
    //TestAlgorithms(100000,-25,25,0.4,4000,1.2 ,1);
    //TestAlgorithms(100000,-25,25,0.4,8000,1.2 ,1);
    cout<<"KONIEC TESTOW"<<endl;
}