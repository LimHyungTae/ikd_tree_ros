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
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


using PointType = pcl::PointXYZ;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

float epsilon = 0.00000001;
PointVector removed_points;

void generate_box(vector<BoxPointType> & boxes, const PointType& center_pt, vector<float> box_lengths){
    vector<BoxPointType> ().swap(boxes);

    BoxPointType boxpoint;
    float & x_dist = box_lengths[0];
    float & y_dist = box_lengths[1];
    float & z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z - z_dist;

    boxes.emplace_back(boxpoint);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcd_viz_demo");
    ros::NodeHandle nh;
    ros::Publisher  MapPub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);

    srand((unsigned) time(NULL));

    pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>);

    string HOME     = std::getenv("HOME");
    string filename = HOME + "/catkin_ws/src/ikd_tree_ros/sample/global_map.pcd";
    if (pcl::io::loadPCDFile<PointType>(filename, *src) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    cout << "# of pts: " << src->points.size() << endl;

    KD_TREE<PointType>::Ptr kdtree_ptr(new KD_TREE<PointType>(0.3, 0.6, 0.2));
    KD_TREE<PointType> &ikd_Tree = *kdtree_ptr;

    auto t1 = chrono::high_resolution_clock::now();
    ikd_Tree.Build((*src).points);
    auto t2              = chrono::high_resolution_clock::now();

    auto build_duration1 = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
    cout << "It takes " << float(build_duration1) / 1000000 << "sec" << endl;

    cout << "# of valid pts: " << ikd_Tree.validnum() << endl;

    // Set box
    vector<BoxPointType> boxes;
    PointType center_pt(100.0, 0.0, 0.0);
    generate_box(boxes, center_pt, {50.0, 50.0, 3.0});

    t1 = chrono::high_resolution_clock::now();
    int num_deleted = ikd_Tree.Delete_Point_Boxes(boxes);
    t2 = chrono::high_resolution_clock::now();

    auto delete_duration = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
    cout << "It takes " << float(delete_duration) / 1000000 << "sec" << endl;

    cout << "# of deleted points: " << num_deleted << endl;

    ikd_Tree.acquire_removed_points(removed_points);

    pcl::PointCloud<PointType> pts_deleted;
    for (const auto& pt: removed_points){
        pts_deleted.points.emplace_back(pt);
    }
    cout<<"Total "<<removed_points.size()<<" are removed"<<endl;

    sensor_msgs::PointCloud2 map_msg;
//    pcl::toROSMsg(*src, map_msg);
    pcl::toROSMsg(pts_deleted, map_msg);
    map_msg.header.frame_id = "/map";

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        MapPub.publish(map_msg);
        cout<<"Pub!"<<endl;
        loop_rate.sleep();
    }

    return 0;
}