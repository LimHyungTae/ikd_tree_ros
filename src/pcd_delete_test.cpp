#include "ikd_Tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointType = pcl::PointXYZI;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

#define Box_Length 50.0
#define Box_Num 1

PointVector point_cloud;
PointVector cloud_increment;
PointVector cloud_decrement;
PointVector cloud_deleted;
PointVector search_result;
PointVector raw_cmp_result;
PointVector DeletePoints;
PointVector removed_points;

void colorize(
        const PointVector &pc,
        pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
        const std::vector<int> &color) {

    int N              = pc.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int         i = 0; i < N; ++i) {
        const auto &pt = pc[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

void generate_box(vector<BoxPointType> &boxes, const PointType &center_pt, vector<float> box_lengths) {
    boxes.clear();

    BoxPointType boxpoint;
    float &x_dist = box_lengths[0];
    float &y_dist = box_lengths[1];
    float &z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;

    cout << "For debugging" << endl;
    cout << boxpoint.vertex_min[0] << ", " << boxpoint.vertex_max[0] << endl;
    cout << boxpoint.vertex_min[1] << ", " << boxpoint.vertex_max[1] << endl;
    cout << boxpoint.vertex_min[2] << ", " << boxpoint.vertex_max[2] << endl;

    boxes.emplace_back(boxpoint);
}

float rand_float(float x_min, float x_max) {
    float rand_ratio = rand() / (float) RAND_MAX;
    return (x_min + rand_ratio * (x_max - x_min));
}

void generate_initial_point_cloud(int num) {
    PointVector().swap(point_cloud);
    PointType new_point;
    for (int  i = 0; i < num; i++) {
        new_point.x = rand_float(-10, 10);
        new_point.y = rand_float(-10, 10);
        new_point.z = rand_float(-10, 10);
        point_cloud.push_back(new_point);
    }
    return;
}

int main(int argc, char **argv) {
    srand((unsigned) time(NULL));

    // KD_TREE<PointType> ikd_Tree(0.3,0.6,0.2);
    KD_TREE<PointType>::Ptr kdtree_ptr(new KD_TREE<PointType>(0.3, 0.6, 0.2));
    KD_TREE <PointType>     &ikd_Tree = *kdtree_ptr;
    vector <BoxPointType>   Delete_Boxes;
    vector <BoxPointType>   Add_Boxes;
    vector<float>           PointDist;

    // Initialize k-d tree
    pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud <PointType>);

    string HOME     = std::getenv("HOME");
    string filename = HOME + "/catkin_ws/src/ikd_tree_ros/sample/global_map.pcd";
    if (pcl::io::loadPCDFile<PointType>(filename, *src) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    cout << "Original: " << src->points.size() << " points are loaded" << endl;
    point_cloud = (*src).points;

//    generate_initial_point_cloud(2000);

    PointType minPt, maxPt;
    pcl::getMinMax3D(*src, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    auto t1 = chrono::high_resolution_clock::now();
    ikd_Tree.Build(point_cloud);
    auto t2       = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
    printf("Building tree takes: %0.3f ms\n", float(duration) / 1e3);

    cout << "# of valid pts: " << ikd_Tree.validnum() << endl;
    removed_points.clear();
    ikd_Tree.acquire_removed_points(removed_points);
    cout << "\033[1;32m[Debug] Removed pts?: " << removed_points.size() << "\033[0m" << endl;

    PointType center_pt;
    center_pt.x = 100.0;
    center_pt.y = 0.0;
    center_pt.z = 0.0;
    generate_box(Delete_Boxes, center_pt, {50.0, 50.0, 100.0});

    t1 = chrono::high_resolution_clock::now();

    int num_deleted = ikd_Tree.Delete_Point_Boxes(Delete_Boxes);
    t2       = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
    printf("Radius delete takes: %0.3f ms\n", float(duration) / 1e3);

    removed_points.clear();
    ikd_Tree.acquire_removed_points(removed_points);
    cout << "\033[1;32m # of deleted points - " << num_deleted << "\033[0m" << endl;
    cout << "\033[1;32m # of removed points - " << removed_points.size() << "\033[0m" << endl;

    pcl::PointCloud<PointType>::Ptr dst(new pcl::PointCloud <PointType>);

    ikd_Tree.flatten(ikd_Tree.Root_Node, ikd_Tree.PCL_Storage, NOT_RECORD);
    dst->points = ikd_Tree.PCL_Storage;
    cout << "Finally, " << dst->points.size() << " points remain" << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_colored(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_colored(new pcl::PointCloud <pcl::PointXYZRGB>);

    // Below lines are for visualization
    cout << point_cloud.size() << endl;
    cout << removed_points.size() << endl;

    colorize(point_cloud, *src_colored, {255, 0, 0});
    colorize(removed_points, *removed_colored, {0, 255, 0});
    colorize(dst->points, *dst_colored, {0, 0, 255});

    pcl::visualization::PCLVisualizer viewer("Raw");

//    viewer.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
//    viewer.addPointCloud<pcl::PointXYZRGB>(removed_colored, "vox_green");
    viewer.addPointCloud<pcl::PointXYZRGB>(dst_colored, "complement");

    auto &bpt = Delete_Boxes[0];
    Eigen::Vector3f    t((bpt.vertex_min[0] + bpt.vertex_max[0]) / 2,
                         (bpt.vertex_min[1] + bpt.vertex_max[1]) / 2,
                         (bpt.vertex_min[2] + bpt.vertex_max[2]) / 2);
    Eigen::Quaternionf q(1.0, 0.0, 0.0, 0.0);

//    viewer.addCube(t, q, Box_Length, Box_Length, Box_Length, "cube");
    while (!viewer.wasStopped()) {// } && !viewer2.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}