#ifndef OURUTIL_COMMON_H
#define OURUTIL_COMMON_H
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/png_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <math.h>
using namespace cv;
using namespace std;
using namespace pcl;

extern string source_dir;
extern string target_dir;
extern int img_width;
extern int img_height;
extern double cx, cy, fx, fy;

const double cam_factor = 1000;
const float empty_num = 666;
const int max_size = 1280 * 2;
const float extern_rate = 1.2;

extern float record[max_size][max_size];
extern int source_cols[max_size][max_size];
extern int target_cols[max_size][max_size];
extern float max_distance[max_size][max_size];
extern float min_distance[max_size][max_size];
typedef Point_<float> Point2;
// show.cpp
void show_3D_pair_point(const PointCloud<PointXYZ>::Ptr& source_cloud, const PointCloud<PointXYZ>::Ptr& target_cloud);
void show_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud);
void show_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud, const PointCloud<PointXYZRGB>::Ptr& view_cloud);
void show_match_distance(const PointCloud<PointXYZ>::Ptr& source_cloud, const PointCloud<PointXYZ>::Ptr& target_cloud, const Eigen::Matrix4f& transform, string label="Match Distance:");
void show_split_view(const PointCloud<PointXYZRGB>::Ptr& source_cloud, const PointCloud<PointXYZRGB>::Ptr& target_cloud, const Eigen::Matrix4f& source_transform, const Eigen::Matrix4f& target_transform,
                                        const int& source_col, const int& target_col, const float& x_offset, const float& z_offset);

//reconstruct_3D.cpp
Mat read_dir_img(string root_dir, string img_name);
PointCloud<PointXYZRGB>::Ptr construct_point_cloud(const Mat& rgb, const Mat& depth);
PointCloud<PointXYZRGB>::Ptr construct_point_cloud(const Mat& rgb, const Mat& depth, vector<Point2>& left_points, vector<Point2>& right_points);
void construct_sift_point_cloud(const Mat& source_depth, const Mat& target_depth, PointCloud<PointXYZ>::Ptr& source_sift_cloud, PointCloud<PointXYZ>::Ptr& target_sift_cloud);
void construct_sift_point_cloud(const vector<pair<Point2, Point2> >& match_pairs, const Mat& source_depth, const Mat& target_depth, PointCloud<PointXYZ>::Ptr& source_sift_cloud, PointCloud<PointXYZ>::Ptr& target_sift_cloud);
Mat get_filter_depth(string root_dir);


//stitching_3D.cpp
Eigen::Matrix4f calculate_stitch_transformation(const PointCloud<PointXYZ>::Ptr& source_cloud, const PointCloud<PointXYZ>::Ptr& target_cloud);


//spliting_3D.cpp
Mat get_coordinate_image(const PointCloud<PointXYZRGB>::Ptr& cloud);
Mat draw_cloud_image(const PointCloud<PointXYZRGB>::Ptr& cloud);
Mat draw_cloud_image(const PointCloud<PointXYZRGB>::Ptr& cloud, const vector<Point2> origin_points, vector<Point2>& view_source_points, vector<Point2>& view_target_points);
Mat draw_cloud_image(const PointCloud<PointXYZRGB>::Ptr& cloud, const vector<Point2> origin_points, vector<Point2>& view_source_points, vector<Point2>& view_target_points, const int& start_col, const int& end_col);
void calculate_cloud_overlap_edge(const Mat& source_coordinate_img, const Mat& target_coordinate_img, float& min_col, float& max_col, float x_offset, float z_offset);
float calculate_interval_score(const Mat& source_coordinate_img, const Mat& target_coordinate_img, int source_start_col, int target_end_col, float x_offset, float z_offset);
float findBestInterval(const Mat& source_coordinate_img, const Mat& target_coordinate_img, int& source_col, int& target_col, float x_offset, float z_offset, float source_max_x, float target_min_x);
void calculate_according_right_edge(const Mat& source_coordinate_img, const Mat& target_coordinate_img, int& source_col, int& target_col);


//stitching_view.cpp
void calculate_overlap_width(const int& source_left_col, const int& target_left_col, const int& source_right_col, const int& target_right_col, int& left_overlap_width, int& right_overlap_width);
void append_overlap_image(const PointCloud<PointXYZRGB>::Ptr& source_cloud, const PointCloud<PointXYZRGB>::Ptr& target_cloud, const float& x_offset, const float& z_offset, const int& source_max_col, const int& target_min_col, const int& overlap_width,
    const vector<Point2> source_origin_points, vector<Point2>& view_source_origin_points, vector<Point2>& view_source_dest_points,
    const vector<Point2> target_origin_points, vector<Point2>& view_target_origin_points, vector<Point2>& view_target_dest_points,
    Mat& source_overlap_image, Mat& target_overlap_image
);
Mat blending_images(const Mat& source_img, const Mat& target_img, const int overlap_width);
#endif