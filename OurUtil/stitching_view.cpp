#include "common.h"
#include "../CommonUtil/function.h"
void calculate_overlap_width(const int& source_left_col, const int& target_left_col, const int& source_right_col, const int& target_right_col, int& left_overlap_width, int& right_overlap_width) {
    int common_overlap = min_distance[source_left_col][target_left_col];
    int left_without_overlap_width = (source_left_col + img_width * extern_rate - target_left_col);
    int right_without_overlap_width = (source_right_col + img_width * extern_rate - target_right_col);
    left_overlap_width = common_overlap;
    right_overlap_width = common_overlap;
    if (left_without_overlap_width < right_without_overlap_width) {
        left_overlap_width = right_without_overlap_width - left_without_overlap_width + common_overlap;
        right_overlap_width = common_overlap;
    } else {
        left_overlap_width = common_overlap;
        right_overlap_width = left_without_overlap_width - right_without_overlap_width + common_overlap;
    }
}

Mat draw_overlap_image(const PointCloud<PointXYZRGB>::Ptr& source_cloud, const PointCloud<PointXYZRGB>::Ptr& target_cloud, const float& x_offset, const float& z_offset, const int& source_max_col, const int& target_min_col, const int& overlap_width) {
    PointCloud<PointXYZRGB>::Ptr overlap_cloud(new PointCloud<PointXYZRGB>);
    for (int i = 0 ; i < source_cloud->points.size() ; ++i) {
	    PointXYZRGB p = source_cloud->points[i];
        float origin_col = p.x * fx / -p.z + img_width * extern_rate / 2;
        float dest_col = (p.x + x_offset) * fx / -(p.z + z_offset) + img_width * extern_rate / 2;
        if (origin_col >= source_max_col && dest_col <= target_min_col) {
            overlap_cloud->points.push_back(p);
        }
    }
    for (int i = 0 ; i < target_cloud->points.size() ; ++i) {
	    PointXYZRGB p = target_cloud->points[i];
        float origin_col = p.x * fx / -p.z + img_width * extern_rate / 2;
        float dest_col = (p.x - x_offset) * fx / -(p.z - z_offset) + img_width * extern_rate / 2;
        if (dest_col >= source_max_col && origin_col <= target_min_col) {
            p.x -= x_offset;
            p.z -= z_offset;
            overlap_cloud->points.push_back(p);
        }
    }
    Eigen::Matrix4f transform_offset = Eigen::Matrix4f::Identity();
    transform_offset(0, 3) = x_offset / (overlap_width + 0.0);
    transform_offset(2, 3) = z_offset / (overlap_width + 0.0);

    Mat overlap_img = Mat::zeros(img_height * extern_rate, overlap_width, CV_8UC3);
    for (int i = 0 ; i <= overlap_width ; ++i) {
        Mat tmp_img = draw_cloud_image(overlap_cloud);
        for (int row = 0 ; row < tmp_img.rows ; ++row) {
            int according_col = source_max_col - i / (overlap_width + 0.0) * (source_max_col - target_min_col);
            overlap_img.ptr<uchar>(row)[i * 3] = tmp_img.ptr<uchar>(row)[according_col * 3];
            overlap_img.ptr<uchar>(row)[i * 3 + 1] = tmp_img.ptr<uchar>(row)[according_col * 3 + 1];
            overlap_img.ptr<uchar>(row)[i * 3 + 2] = tmp_img.ptr<uchar>(row)[according_col * 3 + 2];


            tmp_img.ptr<uchar>(row)[according_col * 3] = 0;
            tmp_img.ptr<uchar>(row)[according_col * 3 + 1] = 0;
            tmp_img.ptr<uchar>(row)[according_col * 3 + 2] = 255;
        }
        transformPointCloud(*overlap_cloud, *overlap_cloud, transform_offset);
//        ostringstream out_oss;
//        out_oss << i << "overlap.png";
//        imwrite(out_oss.str().c_str(), tmp_img);
    }
    return overlap_img;
}

Mat scan_overlap_cloud(PointCloud<PointXYZRGB>::Ptr& overlap_cloud, const vector<Point2>& overlap_points, vector<Point2>& view_origin_points, vector<Point2>& view_dest_points,
                        const int& source_max_col, const int& target_min_col, const int& overlap_width, const Eigen::Matrix4f& transform_offset, Mat& overlap_img, bool is_source=true) {
    vector<bool> label(overlap_points.size(), false);
    float row, col, floor_row, floor_col, ceil_row, ceil_col;
    ushort history_d, current_d;
    int mod_y = int(img_height * extern_rate / 50);
    int mod_i = int(overlap_width / 10);
    mod_i = mod_i > int(img_width * extern_rate / 100) ? int(img_width * extern_rate / 100) : mod_i;
    for (int i = 0 ; i <= overlap_width ; ++i) {
        int according_col = source_max_col - i / (overlap_width + 0.0) * (source_max_col - target_min_col);
        Mat depth_img = Mat::zeros(img_height * extern_rate, 1, CV_16UC1);
        for (int j = 0 ; j < overlap_cloud->points.size() ; ++j) {
            PointXYZRGB p = overlap_cloud->points[j];
            p.z = -p.z;
            p.y = -p.y;
            row = p.y * fy / p.z + img_height * extern_rate / 2;
            col = p.x * fx / p.z + img_width * extern_rate / 2;
            floor_row = floor(row);
            floor_col = floor(col);
            if (int(floor_row) >= 0 && int(floor_row) < img_height * extern_rate && int(floor_col) == according_col) {
                history_d = depth_img.ptr<ushort>(int(floor_row))[0];
                current_d = ushort(p.z * cam_factor);
                if (history_d == 0 || current_d < history_d) {
                    depth_img.ptr<ushort>(int(floor_row))[0] = current_d;
                    overlap_img.ptr<uchar>(int(floor_row))[i * 3] = p.b;
                    overlap_img.ptr<uchar>(int(floor_row))[i * 3 + 1] = p.g;
                    overlap_img.ptr<uchar>(int(floor_row))[i * 3 + 2] = p.r;
                }
            }
        }
        assert(overlap_cloud->points.size() == overlap_points.size());
        for (int j = 0 ; j < overlap_cloud->points.size() ; ++j) {
            if (i % mod_i != 0) continue;
            PointXYZRGB p = overlap_cloud->points[j];
            p.z = -p.z;
            p.y = -p.y;
            row = p.y * fy / p.z + img_height * extern_rate / 2;
            col = p.x * fx / p.z + img_width * extern_rate / 2;
            if (int(row) % mod_y != 0) continue;
            floor_row = floor(row);
            floor_col = floor(col);
            if (int(floor_row) >= 0 && int(floor_row) < img_height * extern_rate && int(floor_col) == according_col) {
                history_d = depth_img.ptr<ushort>(int(floor_row))[0];
                current_d = ushort(p.z * cam_factor);
                if (current_d <= history_d && !label[j]) {
                    view_origin_points.push_back(overlap_points[j]);
                    if (is_source) {
                        view_dest_points.emplace_back(source_max_col + i, row);
                    } else {
                        view_dest_points.emplace_back(target_min_col - (overlap_width - i), row);
                    }
                    label[j] = true;
                }
            }
        }
        transformPointCloud(*overlap_cloud, *overlap_cloud, transform_offset);
    }
    return overlap_img;
}

void append_overlap_image(const PointCloud<PointXYZRGB>::Ptr& source_cloud, const PointCloud<PointXYZRGB>::Ptr& target_cloud, const float& x_offset, const float& z_offset, const int& source_max_col, const int& target_min_col, const int& overlap_width,
    const vector<Point2> source_origin_points, vector<Point2>& view_source_origin_points, vector<Point2>& view_source_dest_points,
    const vector<Point2> target_origin_points, vector<Point2>& view_target_origin_points, vector<Point2>& view_target_dest_points,
    Mat& source_overlap_image, Mat& target_overlap_image
) {
    vector<Point2> source_overlap_points;
    PointCloud<PointXYZRGB>::Ptr source_overlap_cloud(new PointCloud<PointXYZRGB>);
    for (int i = 0 ; i < source_cloud->points.size() ; ++i) {
	    PointXYZRGB p = source_cloud->points[i];
        float origin_col = p.x * fx / -p.z + img_width * extern_rate / 2;
        float dest_col = (p.x + x_offset) * fx / -(p.z + z_offset) + img_width * extern_rate / 2;
        if (origin_col >= source_max_col && dest_col <= target_min_col) {
            source_overlap_cloud->points.push_back(p);
            source_overlap_points.push_back(source_origin_points[i]);
        }
    }
    vector<Point2> target_overlap_points;
    PointCloud<PointXYZRGB>::Ptr target_overlap_cloud(new PointCloud<PointXYZRGB>);
    for (int i = 0 ; i < target_cloud->points.size() ; ++i) {
	    PointXYZRGB p = target_cloud->points[i];
        float origin_col = p.x * fx / -p.z + img_width * extern_rate / 2;
        float dest_col = (p.x - x_offset) * fx / -(p.z - z_offset) + img_width * extern_rate / 2;
        if (dest_col >= source_max_col && origin_col <= target_min_col) {
            p.x -= x_offset;
            p.z -= z_offset;
            target_overlap_cloud->points.push_back(p);
            target_overlap_points.push_back(target_origin_points[i]);
        }
    }
    Eigen::Matrix4f transform_offset = Eigen::Matrix4f::Identity();
    transform_offset(0, 3) = x_offset / (overlap_width + 0.0);
    transform_offset(2, 3) = z_offset / (overlap_width + 0.0);
    scan_overlap_cloud(source_overlap_cloud, source_overlap_points, view_source_origin_points, view_source_dest_points, source_max_col, target_min_col, overlap_width, transform_offset, source_overlap_image, true);
    scan_overlap_cloud(target_overlap_cloud, target_overlap_points, view_target_origin_points, view_target_dest_points, source_max_col, target_min_col, overlap_width, transform_offset, target_overlap_image, false);

}


Mat blending_images(const Mat& source_img, const Mat& target_img, const int overlap_width) {
    Mat source_result = Mat::zeros(source_img.rows, source_img.cols + target_img.cols - overlap_width, CV_8UC3);
    Mat source_imageROI = source_result(Rect(0, 0, source_img.cols, source_img.rows));
    source_img(Rect(0, 0, source_img.cols, source_img.rows)).copyTo(source_imageROI);

    Mat target_result = Mat::zeros(source_img.rows, source_img.cols + target_img.cols - overlap_width, CV_8UC3);
    Mat target_imageROI = target_result(Rect(source_img.cols - overlap_width, 0, target_img.cols, target_img.rows));
    target_img(Rect(0, 0, target_img.cols, target_img.rows)).copyTo(target_imageROI);

    vector<int> paths;
    findBestLine(source_result, target_result, paths, source_img.cols - overlap_width, source_img.cols);
    Mat result = stitchingByBestLine(paths, source_result, target_result);
    return result;
}

//Mat blending_images(const Mat& source_img, const Mat& target_img, const int overlap_width) {
//    Mat result = Mat::zeros(source_img.rows, source_img.cols + target_img.cols - overlap_width, CV_8UC3);
//    Mat source_imageROI = result(Rect(0, 0, source_img.cols - overlap_width, source_img.rows));
//    source_img(Rect(0, 0, source_img.cols - overlap_width, source_img.rows)).copyTo(source_imageROI);
//
//    Mat target_imageROI = result(Rect(source_img.cols, 0, target_img.cols - overlap_width, target_img.rows));
//    target_img(Rect(overlap_width, 0, target_img.cols - overlap_width, target_img.rows)).copyTo(target_imageROI);
//
//    for (int row = 0 ; row < result.rows ; ++row) {
//        for (int col = 0 ; col < overlap_width ; ++col) {
//            float source_weight = (overlap_width - col + 0.0) / overlap_width;
//            float target_weight = (col + 0.0) / overlap_width;
//
//            int source_b = source_img.ptr<uchar>(row)[(source_img.cols - overlap_width + col) * 3];
//			int source_g = source_img.ptr<uchar>(row)[(source_img.cols - overlap_width + col) * 3 + 1];
//			int source_r = source_img.ptr<uchar>(row)[(source_img.cols - overlap_width + col) * 3 + 2];
//
//
//            int target_b = target_img.ptr<uchar>(row)[col * 3];
//			int target_g = target_img.ptr<uchar>(row)[col * 3 + 1];
//			int target_r = target_img.ptr<uchar>(row)[col * 3 + 2];
//
//            if (source_b == 0 && source_g == 0 && source_r == 0) {
//                source_weight = 0;
//                target_weight = 1;
//            } else if (target_b == 0 && target_g == 0 && target_r == 0) {
//                source_weight = 1;
//                target_weight = 0;
//            }
//
//			int b = round(source_weight * source_b + target_weight * target_b);
//			int g = round(source_weight * source_g + target_weight * target_g);
//			int r = round(source_weight * source_r + target_weight * target_r);
//
//            result.ptr<uchar>(row)[(source_img.cols - overlap_width + col) * 3] = b;
//            result.ptr<uchar>(row)[(source_img.cols - overlap_width + col) * 3 + 1] = g;
//            result.ptr<uchar>(row)[(source_img.cols - overlap_width + col) * 3 + 2] = r;
//        }
//    }
////    for (int row = 0 ; row < result.rows ; ++row) {
////        result.ptr<uchar>(row)[(source_img.cols) * 3] = 0;
////        result.ptr<uchar>(row)[(source_img.cols) * 3 + 1] = 0;
////        result.ptr<uchar>(row)[(source_img.cols) * 3 + 2] = 255;
////
////        result.ptr<uchar>(row)[(source_img.cols - overlap_width) * 3] = 255;
////        result.ptr<uchar>(row)[(source_img.cols - overlap_width) * 3 + 1] = 0;
////        result.ptr<uchar>(row)[(source_img.cols - overlap_width) * 3 + 2] = 0;
////    }
//    return result;
//}