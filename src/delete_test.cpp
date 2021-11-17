#include "ikd_Tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/visualization/pcl_visualizer.h>


using PointType = pcl::PointXYZINormal;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;


#define X_MAX 5.0
#define X_MIN -5.0
#define Y_MAX 5.0
#define Y_MIN -5.0
#define Z_MAX 5.0
#define Z_MIN -5.0

#define Point_Num 20000
#define New_Point_Num 200
#define Delete_Point_Num 100
#define Nearest_Num 5
#define Test_Time 1000
#define Search_Counter 200
#define Box_Length 4
#define Box_Num 1
#define Delete_Box_Switch true
#define Add_Box_Switch true

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

float rand_float(float x_min, float x_max) {
    float rand_ratio = rand() / (float) RAND_MAX;
    return (x_min + rand_ratio * (x_max - x_min));
}


/*
   Generate the points to initialize an incremental k-d tree
*/

void generate_initial_point_cloud(int num) {
    PointVector().swap(point_cloud);
    PointType new_point;
    for (int  i = 0; i < num; i++) {
        new_point.x = rand_float(X_MIN, X_MAX);
        new_point.y = rand_float(Y_MIN, Y_MAX);
        new_point.z = rand_float(Z_MIN, Z_MAX);
        point_cloud.push_back(new_point);
    }
    return;
}

/*
    Generate random boxes for box-wise delete on the incremental k-d tree
*/

void generate_box_decrement(vector<BoxPointType> &Delete_Boxes, float box_length, int box_num) {
    vector<BoxPointType>().swap(Delete_Boxes);
    float        d = box_length / 2;
    float        x_p, y_p, z_p;
    BoxPointType boxpoint;
    for (int     k = 0; k < box_num; k++) {
//        x_p = rand_float(X_MIN, X_MAX);
//        y_p = rand_float(Y_MIN, Y_MAX);
//        z_p = rand_float(Z_MIN, Z_MAX);

        x_p = 0.0;
        y_p = 0.0;
        z_p = rand_float(Z_MIN, Z_MAX);
        boxpoint.vertex_min[0] = x_p - d;
        boxpoint.vertex_max[0] = x_p + d;
        boxpoint.vertex_min[1] = y_p - d;
        boxpoint.vertex_max[1] = y_p + d;
        boxpoint.vertex_min[2] = z_p - d;
        boxpoint.vertex_max[2] = z_p + d;
        Delete_Boxes.push_back(boxpoint);
    }
}


/*
    Generate target point for nearest search on the incremental k-d tree
*/

PointType generate_target_point() {
    PointType point;
    point.x = rand_float(X_MIN, X_MAX);;
    point.y = rand_float(Y_MIN, Y_MAX);
    point.z = rand_float(Z_MIN, Z_MAX);
    return point;
}

int main(int argc, char **argv) {
    srand((unsigned) time(NULL));

    KD_TREE<PointType>::Ptr kdtree_ptr(new KD_TREE<PointType>(0.3, 0.6, 0.2));
    KD_TREE<PointType>      &ikd_Tree = *kdtree_ptr;
    printf("Testing ...\n");
    vector<BoxPointType> Delete_Boxes;
    vector<BoxPointType> Add_Boxes;
    vector<float>        PointDist;

    // Initialize k-d tree
    generate_initial_point_cloud(Point_Num);
    cout << ": # of point cloud: " << point_cloud.size() << endl;

    auto t1 = chrono::high_resolution_clock::now();
    ikd_Tree.Build(point_cloud);
    auto t2             = chrono::high_resolution_clock::now();
    auto build_duration = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
    cout << "Before: # of valid pts: " << ikd_Tree.validnum() << endl;

    cout << "Conducting deleting..." << endl;

    generate_box_decrement(Delete_Boxes, Box_Length, Box_Num);
    int num_deleted = ikd_Tree.Delete_Point_Boxes(Delete_Boxes);

    cout << "After: # of valid pts: " << ikd_Tree.validnum() << endl;

    removed_points.clear();
    ikd_Tree.acquire_removed_points(removed_points);
    cout << "\033[1;32m # of deleted points - " << num_deleted << "\033[0m" << endl;
    cout << "\033[1;32m # of removed points - " << removed_points.size() << "\033[0m" << endl;

    cout<<point_cloud.size()<<endl;
    cout<<removed_points.size()<<endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Point cloud XYZ에 RGB 칼라 추가하기
    colorize(point_cloud, *src_colored, {255, 0, 0});
    colorize(removed_points, *vox_colored, {0, 255, 0});

    pcl::visualization::PCLVisualizer viewer("Raw");

    viewer.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
    viewer.addPointCloud<pcl::PointXYZRGB>(vox_colored, "vox_green");
//    viewer2.addPointCloud<pcl::PointXYZRGB>(vox_colored, "vox_green");

    auto               &bpt = Delete_Boxes[0];
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