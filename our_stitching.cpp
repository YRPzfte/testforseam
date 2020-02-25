#include "./OurUtil/common.h"
#include "./Debugger/TimeCalculator.h"
#include "./Stitching/Consistent_Stitching.h"
string source_dir = "";
string target_dir = "";
int img_width = 0;
int img_height = 0;
double cx = 635.645, cy = 365.768, fx = 712.309, fy = 712.112;
float record[max_size][max_size];
int source_cols[max_size][max_size];
int target_cols[max_size][max_size];
float max_distance[max_size][max_size];
float min_distance[max_size][max_size];

vector<pair<Point2, Point2> > join_pairs(const vector<Point2>& origin_points, const vector<Point2>& dest_points) {
    vector<pair<Point2, Point2> > pairs;
    for (int i = 0 ; i < origin_points.size() ; ++i) {
        pairs.push_back(make_pair(origin_points[i], dest_points[i]));
    }
    return pairs;
}

Mat copyImg(Mat img) {
    Mat color_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_8UC3);
    for (int row = 0 ; row < color_img.rows ; ++row) {
        for (int col = 0 ; col < color_img.cols ; ++col) {
            color_img.at<Vec3b>(int(row), int(col)) = Vec3b(255, 255, 255);
        }
    }
    Mat imageROI = color_img(Rect(img_width * (extern_rate - 1) / 2, img_height * (extern_rate - 1) / 2, img.cols, img.rows));
    img.copyTo(imageROI);
    return color_img;
}


int main (int argc, char** argv) {
    if (argc != 3) {
        std::cout << "params not content, should be: ./stitch_based_on_3D source_dir target_dir out_depth" << std::endl;
        return 0;
    }
    source_dir = argv[1];
    target_dir = argv[2];
    std::cout << "[基于3D场景引导的立体图像拼接:]" << endl;
    TimeCalculator timer;
    timer.start();

    ostringstream param_oss;
    param_oss << source_dir << "/param.txt";
	ifstream paramRead(param_oss.str().c_str());
    paramRead >> cx  >> cy >> fx >> fy;

    //3D场景重建
    Mat source_rgb = read_dir_img(source_dir, "left");
    Mat source_depth = get_filter_depth(source_dir);
    vector<Point2> source_left_points, source_right_points;
    PointCloud<PointXYZRGB>::Ptr source_cloud = construct_point_cloud(source_rgb, source_depth, source_left_points, source_right_points);
    Mat target_rgb = read_dir_img(target_dir, "left");
    Mat target_depth = get_filter_depth(target_dir);
    vector<Point2> target_left_points, target_right_points;
    PointCloud<PointXYZRGB>::Ptr target_cloud = construct_point_cloud(target_rgb, target_depth, target_left_points, target_right_points);
    timer.end("[完成3D场景重建]");


    //3D场景拼接
    PointCloud<PointXYZ>::Ptr source_sift_cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr target_sift_cloud(new PointCloud<PointXYZ>);
    MultiImages multi_left_images(source_dir, target_dir, "left", LINES_FILTER_WIDTH, LINES_FILTER_LENGTH);
    construct_sift_point_cloud(source_depth, target_depth, source_sift_cloud, target_sift_cloud);
    Eigen::Matrix4f transform = calculate_stitch_transformation(source_sift_cloud, target_sift_cloud);

//    //拼接结果展示
//    transformPointCloud(*source_cloud, *source_cloud, transform);
//    *source_cloud += *target_cloud;
//    show_cloud(source_cloud);

    timer.end("[完成3D场景拼接]");
    //最佳视点区域分割
    Eigen::Matrix4f source_transform = transform;
    source_transform(0, 3) = 0;
    source_transform(1, 3) = 0.5 * transform(1, 3);
    source_transform(2, 3) = 0;
    transformPointCloud(*source_cloud, *source_cloud, source_transform);
    Mat source_coordinate_img = get_coordinate_image(source_cloud);

    Eigen::Matrix4f target_transform = Eigen::Matrix4f::Identity();
    target_transform(1, 3) = -0.5 * transform(1, 3);
    transformPointCloud(*target_cloud, *target_cloud, target_transform);
    Mat target_coordinate_img = get_coordinate_image(target_cloud);

//    Mat source_view_img = draw_cloud_image(source_cloud);
//    Mat target_view_img = draw_cloud_image(target_cloud);
//    imwrite("source_view_img.png", source_view_img);
//    imwrite("target_view_img.png", target_view_img);

    float x_offset = transform(0, 3);
    float z_offset = transform(2, 3);

    float source_min_col, target_max_col;
    calculate_cloud_overlap_edge(source_coordinate_img, target_coordinate_img, source_min_col, target_max_col, x_offset, z_offset);
    int source_left_col = 850;//int(source_min_col);
    int target_left_col = 800;//int(target_max_col);

    for (int i = 0 ; i < max_size ; ++i) {
        for (int j = 0 ; j < max_size ; ++j) {
            record[i][j] = -1;
        }
    }
    //计算左视点分割结果
    findBestInterval(source_coordinate_img, target_coordinate_img, source_left_col, target_left_col, x_offset, z_offset, -max_size, max_size);

    //计算对应的右视点分割结果
    int source_right_col = source_left_col;
    int target_right_col = target_left_col;
    calculate_according_right_edge(source_coordinate_img, target_coordinate_img, source_right_col, target_right_col);


    std::cout << source_left_col << "--" << source_right_col << " | " << target_left_col << "--" << target_right_col << std::endl;
    std::cout << "overlap outer width:" << max_distance[source_left_col][target_left_col] << " overlap inner width:" << min_distance[source_left_col][target_left_col] << " overlap_rate:" << record[source_left_col][target_left_col] << std::endl;
    timer.end("[完成最佳视点区域分割]");


    //逆映射
    //计算左右视图中视点迁移区域宽度
    int left_overlap_width, right_overlap_width;
    calculate_overlap_width(source_left_col, target_left_col, source_right_col, target_right_col, left_overlap_width, right_overlap_width);
    std::cout << "Left overlap width:" << left_overlap_width << " | " << "Right overlap width:" << right_overlap_width << std::endl;

//    show_split_view(source_cloud, target_cloud, source_transform, target_transform, source_right_col, target_right_col, x_offset, z_offset);

    //---左视图
    //---source image
    vector<Point2> source_left_origin_points, source_left_dest_points;
    Mat source_left_img = draw_cloud_image(source_cloud, source_left_points, source_left_origin_points, source_left_dest_points, 0, source_left_col);


    //---target image
    vector<Point2> target_left_origin_points, target_left_dest_points;
    Mat target_left_img = draw_cloud_image(target_cloud, target_left_points, target_left_origin_points, target_left_dest_points, target_left_col, img_width * extern_rate);

    //---overlap image
    vector<Point2> view_source_origin_points, view_source_dest_points;
    Mat source_left_overlap_img = Mat::zeros(img_height * extern_rate, left_overlap_width, CV_8UC3);
    Mat target_left_overlap_img = Mat::zeros(img_height * extern_rate, left_overlap_width, CV_8UC3);

    append_overlap_image(source_cloud, target_cloud, x_offset, z_offset, source_left_col, target_left_col, left_overlap_width,
                                               source_left_points, source_left_origin_points, source_left_dest_points,
                                               target_left_points, target_left_origin_points, target_left_dest_points,
                                               source_left_overlap_img, target_left_overlap_img);
    Consistent_Stitching left_consist(multi_left_images);
    left_consist.setSize(img_width * extern_rate, img_height * extern_rate);
    vector<pair<Point2, Point2> > source_left_mapping_pairs = join_pairs(source_left_origin_points, source_left_dest_points);
    vector<pair<Point2, Point2> > target_left_mapping_pairs = join_pairs(target_left_origin_points, target_left_dest_points);
    left_consist.set_mapping_pair(source_left_mapping_pairs, target_left_mapping_pairs);
    vector<vector<pair<Point2, Point2> > > left_boundary_pairs = left_consist.calulate_boundary_pair();
    left_consist.set_boundary_pair(left_boundary_pairs);
    left_consist.setWeightToAlignmentTerm(1);
    left_consist.setWeightToSimilarityTerm(0.75, GLOBAL_ROTATION_3D_METHOD);

    Mat source_left_view, target_left_view;
    Mat stitch_left_img = left_consist.applyWarping(BLEND_AVERAGE, left_overlap_width, source_left_col, target_left_col);


//    imwrite("stitching_left.png", stitch_left_img);
    timer.end("[完成左视图的拼接]");
//    return 0;
    //---右视图
    Eigen::Matrix4f left2right_transform = Eigen::Matrix4f::Identity();
    left2right_transform(0, 3) = -0.12001928710937500000;
    transformPointCloud(*source_cloud, *source_cloud, left2right_transform);
    transformPointCloud(*target_cloud, *target_cloud, left2right_transform);

    //---source image
    vector<Point2> source_right_origin_points, source_right_dest_points;
    Mat source_right_img = draw_cloud_image(source_cloud, source_right_points, source_right_origin_points, source_right_dest_points, 0, source_right_col);

    //---target image
    vector<Point2> target_right_origin_points, target_right_dest_points;
    Mat target_right_img = draw_cloud_image(target_cloud, target_right_points, target_right_origin_points, target_right_dest_points, target_right_col, img_width * extern_rate);

    //---overlap image
    Mat source_right_overlap_img = Mat::zeros(img_height * extern_rate, right_overlap_width, CV_8UC3);
    Mat target_right_overlap_img = Mat::zeros(img_height * extern_rate, right_overlap_width, CV_8UC3);
    append_overlap_image(source_cloud, target_cloud, x_offset, z_offset, source_right_col, target_right_col, right_overlap_width,
                                               source_right_points, source_right_origin_points, source_right_dest_points,
                                               target_right_points, target_right_origin_points, target_right_dest_points,
                                               source_right_overlap_img, target_right_overlap_img);
    MultiImages multi_rigth_images(source_dir, target_dir, "right", LINES_FILTER_WIDTH, LINES_FILTER_LENGTH);

    Consistent_Stitching right_consist(multi_rigth_images);
    right_consist.setPaths(left_consist.getMappingPaths());
    right_consist.setSize(img_width * extern_rate, img_height * extern_rate);
    vector<pair<Point2, Point2> > source_right_mapping_pairs = join_pairs(source_right_origin_points, source_right_dest_points);
    vector<pair<Point2, Point2> > target_right_mapping_pairs = join_pairs(target_right_origin_points, target_right_dest_points);
    right_consist.set_mapping_pair(source_right_mapping_pairs, target_right_mapping_pairs);
    vector<vector<pair<Point2, Point2> > > right_boundary_pairs = right_consist.calulate_boundary_pair();

    right_consist.set_boundary_pair(right_boundary_pairs);
    right_consist.setWeightToAlignmentTerm(1);
    right_consist.setWeightToSimilarityTerm(0.75, GLOBAL_ROTATION_3D_METHOD);

    Mat stitch_right_img = right_consist.applyWarping(BLEND_AVERAGE, right_overlap_width, source_right_col, target_right_col);
//    imwrite("stitching_right.png", stitch_right_img);
    timer.end("[完成右视图的拼接]");

    crop_stitching_result(stitch_left_img, stitch_right_img);
    imwrite("our_left.png", stitch_left_img);
    imwrite("our_right.png", stitch_right_img);
    timer.end("[完成视图的裁剪]");

}