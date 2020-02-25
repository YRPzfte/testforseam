#include "common.h"
#include "../CommonUtil/function.h"


Mat read_dir_img(string root_dir, string img_name) {
    ostringstream file_oss;
    file_oss << root_dir << "/" << img_name << ".png";
    if (img_name == "left" || img_name == "right") {
        return imread(file_oss.str().c_str()); // RGB
    } else {
        assert(img_name == "depth");
        return imread(file_oss.str().c_str(), IMREAD_UNCHANGED);
    }
}

//验证输入图像尺寸一致
void test_img_size(const Mat img) {
    if (img_width == 0) {
        img_width = img.cols;
    } else {
        assert(img_width == img.cols);
    }

    if (img_height == 0) {
        img_height = img.rows;
    } else {
        assert(img_height == img.rows);
    }
}

PointCloud<PointXYZRGB>::Ptr construct_point_cloud(const Mat& rgb, const Mat& depth) {
    test_img_size(rgb);
    test_img_size(depth);
	// 相机坐标系下的点云
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	for (int row = 0; row < depth.rows; ++row) {
		for (int col = 0; col < depth.cols; ++col) {
			ushort d = depth.ptr<ushort>(row)[col];
			if (d <= 0) continue; //剔除深度值为空的点
			PointXYZRGB p;
			//二维坐标与三维坐标转换关系
			p.z = static_cast<float>(d) / cam_factor;
			p.x = (col - cx) * p.z / fx;
			p.y = (row - cy) * p.z / fy;

            // 为便于显示，绕x轴三维旋转180°
			p.y = -p.y;
			p.z = -p.z;

			// 赋予对应颜色值
			p.b = rgb.ptr<uchar>(row)[col * 3];
			p.g = rgb.ptr<uchar>(row)[col * 3 + 1];
			p.r = rgb.ptr<uchar>(row)[col * 3 + 2];
			cloud->points.push_back(p);
		}
	}
    return cloud;
}

PointCloud<PointXYZRGB>::Ptr construct_point_cloud(const Mat& rgb, const Mat& depth, vector<Point2>& left_points, vector<Point2>& right_points) {
    float left2right_offset = -0.12001928710937500000;
    test_img_size(rgb);
    test_img_size(depth);
	// 相机坐标系下的点云
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	float right_col, right_row;
	for (int row = 0; row < depth.rows; ++row) {
		for (int col = 0; col < depth.cols; ++col) {
			ushort d = depth.ptr<ushort>(row)[col];
			if (d <= 0) continue; //剔除深度值为空的点
			PointXYZRGB p;
			//二维坐标与三维坐标转换关系
			p.z = static_cast<float>(d) / cam_factor;
			p.x = (col - cx) * p.z / fx;
			p.y = (row - cy) * p.z / fy;

            // 为便于显示，绕x轴三维旋转180°
			p.y = -p.y;
			p.z = -p.z;

			// 赋予对应颜色值
			p.b = rgb.ptr<uchar>(row)[col * 3];
			p.g = rgb.ptr<uchar>(row)[col * 3 + 1];
			p.r = rgb.ptr<uchar>(row)[col * 3 + 2];
			cloud->points.push_back(p);

			left_points.emplace_back(col, row);
			right_col = (p.x + left2right_offset) * fx / -p.z + cx;
			right_row = -p.y * fy / -p.z + cy;
			right_points.emplace_back(right_col, right_row);
		}
	}
    return cloud;
}

ushort get_bilinear_depth(const Mat& depth, const float& col, const float& row) {
    int int_col = floor(col);
    float dot_col = col - int_col;
    int int_row = floor(row);
    float dot_row = row - int_row;
    if (int_row + 1 >= img_height || int_col + 1 >= img_width) return depth.ptr<ushort>(int_row)[int_col];
    ushort dxy = depth.ptr<ushort>(int_row)[int_col];
    ushort dxy_ = depth.ptr<ushort>(int_row)[int_col + 1];
    ushort dx_y = depth.ptr<ushort>(int_row + 1)[int_col];
    ushort dx_y_ = depth.ptr<ushort>(int_row + 1)[int_col + 1];
    if (dxy == 0 || dxy_ == 0 || dx_y == 0 || dx_y_ == 0) return 0;
    return (1 - dot_col) * (1 - dot_row) * dxy + (1 - dot_col) * dot_row * dxy_
                + dot_col * (1 - dot_row) * dx_y + dot_col * dot_row * dx_y_;
}

void construct_sift_point_cloud(const Mat& source_depth, const Mat& target_depth, PointCloud<PointXYZ>::Ptr& source_sift_cloud, PointCloud<PointXYZ>::Ptr& target_sift_cloud) {
    ostringstream source_sift_oss;
    source_sift_oss << "build/" << source_dir << "/sift.txt";
	ifstream sourceSiftRead(source_sift_oss.str().c_str());

	ostringstream target_sift_oss;
    target_sift_oss << "build/" << target_dir <<  "/sift.txt";
	ifstream targetSiftRead(target_sift_oss.str().c_str());

	int source_row, source_col, target_row, target_col;
	ushort source_d, target_d;
	while (sourceSiftRead >> source_col >> source_row) {
	    if (sourceSiftRead.eof()) break;
	    targetSiftRead >> target_col >> target_row;
        source_d = source_depth.ptr<ushort>(source_row)[source_col];
        target_d = target_depth.ptr<ushort>(target_row)[target_col];
        if (source_d == 0 || target_d == 0) continue;
        PointXYZ source_p, target_p;

        // source sift point
        source_p.z = static_cast<float>(source_d) / cam_factor;
        source_p.x = (source_col - cx) * source_p.z / fx;
        source_p.y = (source_row - cy) * source_p.z / fy;
        // 为便于显示，绕x轴三维旋转180°
        source_p.y = -source_p.y;
        source_p.z = -source_p.z;

        source_sift_cloud->points.push_back(source_p);

        // target sift point
        target_p.z = static_cast<float>(target_d) / cam_factor;
        target_p.x = (target_col - cx) * target_p.z / fx;
        target_p.y = (target_row - cy) * target_p.z / fy;
        // 为便于显示，绕x轴三维旋转180°
        target_p.y = -target_p.y;
        target_p.z = -target_p.z;

        target_sift_cloud->points.push_back(target_p);
	}
}

void construct_sift_point_cloud(const vector<pair<Point2, Point2> >& match_pairs, const Mat& source_depth, const Mat& target_depth, PointCloud<PointXYZ>::Ptr& source_sift_cloud, PointCloud<PointXYZ>::Ptr& target_sift_cloud) {
    int filter_sift_count = 0;
	float source_row, source_col, target_row, target_col;
	ushort source_d, target_d;
	for (int i = 0 ; i < match_pairs.size() ; ++i) {
        Point2 source_feature_p = match_pairs[i].first;
        Point2 target_feature_p = match_pairs[i].second;

	    source_col = source_feature_p.x;
	    source_row = source_feature_p.y;

	    target_col  = target_feature_p.x;
	    target_row = target_feature_p.y;

        source_d = get_bilinear_depth(source_depth, source_col, source_row);
        target_d = get_bilinear_depth(target_depth, target_col, target_row);
        if (source_d == 0 || target_d == 0) continue;
        PointXYZ source_p, target_p;

        // source sift point
        source_p.z = static_cast<float>(source_d) / cam_factor;
        source_p.x = (source_col - cx) * source_p.z / fx;
        source_p.y = (source_row - cy) * source_p.z / fy;
        // 为便于显示，绕x轴三维旋转180°
        source_p.y = -source_p.y;
        source_p.z = -source_p.z;

        source_sift_cloud->points.push_back(source_p);

        // target sift point
        target_p.z = static_cast<float>(target_d) / cam_factor;
        target_p.x = (target_col - cx) * target_p.z / fx;
        target_p.y = (target_row - cy) * target_p.z / fy;
        // 为便于显示，绕x轴三维旋转180°
        target_p.y = -target_p.y;
        target_p.z = -target_p.z;

        target_sift_cloud->points.push_back(target_p);
	    filter_sift_count += 1;
	}
	 std::cout << "Detect Sift pair Num:" << filter_sift_count << std::endl;
}

Mat get_filter_depth(string root_dir) {
    Mat left_rgb = read_dir_img(root_dir, "left");
    Mat depth = read_dir_img(root_dir, "depth");
	PointCloud<PointXYZRGB>::Ptr cloud = construct_point_cloud(left_rgb, depth);

	//创建滤波器
	PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlierRemovaler;   //创建滤波器对象
    outlierRemovaler.setInputCloud(cloud);                           //设置待滤波的点云
    outlierRemovaler.setMeanK(50);                               //设置在进行统计时考虑查询点临近点数
    outlierRemovaler.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值
    outlierRemovaler.filter(*filtered_cloud);
    //生成滤波后深度图
    Mat filtered_depth = Mat::zeros(depth.rows, depth.cols, CV_16UC1);
    float row, col;
    for (int i = 0 ; i < filtered_cloud->size() ; ++i) {
        PointXYZRGB p = filtered_cloud->points[i];
        p.y = -p.y;
        p.z = -p.z;
        row = (p.y * fy) / p.z + cy;
        col = (p.x * fx) / p.z + cx;
        ushort d = depth.ptr<ushort>(int(round(row)))[int(round(col))];
        filtered_depth.ptr<ushort>(int(round(row)))[int(round(col))] =  d;
    }
    return filtered_depth;
}