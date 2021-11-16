#include "ikd_Tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using PointType = pcl::PointXYZINormal;
using PointVector = std::vector<PointType,Eigen::aligned_allocator<PointType>>;

int main(int argc, char** argv){
    ros::init(argc, argv, "speed_demo");
    ros::NodeHandle nh;
    ros::Publisher MapPub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);

    srand((unsigned) time(NULL));

    pcl::PointCloud<PointType>::Ptr src (new pcl::PointCloud<PointType>);

//    string filename = "/media/shapelim/UX960NVMe1/deep_express/LIO-SAM/211013_bongeunsa_align/dense_global_map.pcd";
    string filename = "/home/shapelim/catkin_ws/src/ikd-Tree/sample/global_map.pcd";
    if (pcl::io::loadPCDFile<PointType> (filename, *src) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    cout<<"# of pts: "<<src->points.size()<<endl;

    KD_TREE<PointType>::Ptr kdtree_ptr (new KD_TREE<PointType>(0.3,0.6,0.2));
    KD_TREE<PointType> & ikd_Tree = *kdtree_ptr;

    pcl::KdTreeFLANN<PointType> FLANNKdTree;

    auto t1 = chrono::high_resolution_clock::now();
    ikd_Tree.Build((*src).points);
    auto t2 = chrono::high_resolution_clock::now();    
    auto build_duration1 = chrono::duration_cast<chrono::microseconds>(t2-t1).count();

    auto t3 = chrono::high_resolution_clock::now();
    FLANNKdTree.setInputCloud(src);
    auto t4 = chrono::high_resolution_clock::now();
    auto build_duration2 = chrono::duration_cast<chrono::microseconds>(t4-t2).count();

    cout<<float(build_duration1) / 1000000 << "sec. vs " << float(build_duration2) / 1000000 <<" sec."<<endl;

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*src, map_msg);
    PointType target;
    int num = 5000;

    build_duration1 = 0;
    build_duration2 = 0;
    vector<float> PointDist;
    vector<int> indices;
    PointVector search_result;
    int K = 20;
    for (int i=0;i<num;i++) {
        int rand_idx = rand() % src->points.size();
        target = src->points[rand_idx];

        t1 = chrono::high_resolution_clock::now();
        ikd_Tree.Nearest_Search(target, 20, search_result, PointDist);
        t2 = chrono::high_resolution_clock::now();
        build_duration1 += chrono::duration_cast<chrono::microseconds>(t2 - t1).count();

        t3 = chrono::high_resolution_clock::now();
        FLANNKdTree.nearestKSearch(target, 20, indices, PointDist);
        t4 = chrono::high_resolution_clock::now();
        build_duration2 += chrono::duration_cast<chrono::microseconds>(t4 - t3).count();
    }

    cout<<float(build_duration1)/num << " vs "<< float(build_duration2)/num<<endl;
    // Multi thread started
    // 2.32867sec. vs 1.23031 sec.
    // 8.6566 vs 3.3622

    return 0;
}