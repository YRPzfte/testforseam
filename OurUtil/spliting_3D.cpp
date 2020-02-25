#include "common.h"

Mat get_coordinate_image(const PointCloud<PointXYZRGB>::Ptr& cloud) {
    Mat coordinate_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_32FC3);
    Mat depth_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_16UC1);

	float row, col, floor_row, floor_col, ceil_row, ceil_col;
	ushort history_d, current_d;
	for (int i = 0 ; i < cloud->points.size() ; ++i) {
	    PointXYZRGB p = cloud->points[i];
        p.z = -p.z;
        p.y = -p.y;
        row = p.y * fy / p.z + coordinate_img.rows / 2;
        col = p.x * fx / p.z + coordinate_img.cols / 2;
        floor_row = floor(row);
        floor_col = floor(col);
        ceil_row = ceil(row);
        ceil_col = ceil(col);

        if (int(floor_row) >= 0 && int(floor_row) < coordinate_img.rows && int(floor_col) >= 0 && int(floor_col) < coordinate_img.cols) {
            history_d = depth_img.ptr<ushort>(int(floor_row))[int(floor_col)];
            current_d = ushort(p.z * cam_factor);
            if (history_d == 0 || current_d < history_d) {
                depth_img.ptr<ushort>(int(floor_row))[int(floor_col)] = current_d;
                coordinate_img.ptr<float>(int(floor_row))[int(floor_col) * 3] = p.x;
                coordinate_img.ptr<float>(int(floor_row))[int(floor_col) * 3 + 1] = -p.y;
                coordinate_img.ptr<float>(int(floor_row))[int(floor_col) * 3 + 2] = -p.z;
            }
        }
        if (int(floor_row) >= 0 && int(floor_row) < coordinate_img.rows && int(ceil_col) >= 0 && int(ceil_col) < coordinate_img.cols) {
            history_d = depth_img.ptr<ushort>(int(floor_row))[int(ceil_col)];
            current_d = ushort(p.z * cam_factor);
            if (history_d == 0 || current_d < history_d) {
                depth_img.ptr<ushort>(int(floor_row))[int(ceil_col)] = current_d;
                coordinate_img.ptr<float>(int(floor_row))[int(ceil_col) * 3] = p.x;
                coordinate_img.ptr<float>(int(floor_row))[int(ceil_col) * 3 + 1] = -p.y;
                coordinate_img.ptr<float>(int(floor_row))[int(ceil_col) * 3 + 2] = -p.z;
            }
        }
        if (int(ceil_row) >= 0 && int(ceil_row) < coordinate_img.rows && int(floor_col) >= 0 && int(floor_col) < coordinate_img.cols) {
            history_d = depth_img.ptr<ushort>(int(ceil_row))[int(floor_col)];
            current_d = ushort(p.z * cam_factor);
            if (history_d == 0 || current_d < history_d) {
                depth_img.ptr<ushort>(int(ceil_row))[int(floor_col)] = current_d;
                coordinate_img.ptr<float>(int(ceil_row))[int(floor_col) * 3] = p.x;
                coordinate_img.ptr<float>(int(ceil_row))[int(floor_col) * 3 + 1] = -p.y;
                coordinate_img.ptr<float>(int(ceil_row))[int(floor_col) * 3 + 2] = -p.z;
            }
        }
        if (int(ceil_row) >= 0 && int(ceil_row) < coordinate_img.rows && int(ceil_col) >= 0 && int(ceil_col) < coordinate_img.cols) {
            history_d = depth_img.ptr<ushort>(int(ceil_row))[int(ceil_col)];
            current_d = ushort(p.z * cam_factor);
            if (history_d == 0 || current_d < history_d) {
                depth_img.ptr<ushort>(int(ceil_row))[int(ceil_col)] = current_d;
                coordinate_img.ptr<float>(int(ceil_row))[int(ceil_col) * 3] = p.x;
                coordinate_img.ptr<float>(int(ceil_row))[int(ceil_col) * 3 + 1] = -p.y;
                coordinate_img.ptr<float>(int(ceil_row))[int(ceil_col) * 3 + 2] = -p.z;
            }
        }
	}
	for (int row = 0 ; row < coordinate_img.rows ; ++row) {
	    for (int col = 0 ; col < coordinate_img.cols ; ++col) {
	        history_d = depth_img.ptr<ushort>(row)[col];
	        if (history_d == 0) {
	            coordinate_img.ptr<float>(row)[col * 3] = empty_num;
	            coordinate_img.ptr<float>(row)[col * 3 + 1] = empty_num;
	            coordinate_img.ptr<float>(row)[col * 3 + 2] = empty_num;
	        }
	    }
	}
	return coordinate_img;
}

Mat draw_cloud_image(const PointCloud<PointXYZRGB>::Ptr& cloud) {
    Mat color_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_8UC3);
    for (int row = 0 ; row < color_img.rows ; ++row) {
        for (int col = 0 ; col < color_img.cols ; ++col) {
            color_img.at<Vec3b>(int(row), int(col)) = Vec3b(255, 255, 255);
        }
    }
	Mat depth_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_16UC1);

	float row, col, floor_row, floor_col, ceil_row, ceil_col;
	ushort history_d, current_d;
	for (int i = 0 ; i < cloud->points.size() ; ++i) {
	    PointXYZRGB p = cloud->points[i];
        p.z = -p.z;
        p.y = -p.y;
        row = p.y * fy / p.z + color_img.rows / 2;
        col = p.x * fx / p.z + color_img.cols / 2;
        floor_row = floor(row);
        floor_col = floor(col);
        ceil_row = ceil(row);
        ceil_col = ceil(col);

        if (int(floor_row) >= 0 && int(floor_row) < color_img.rows && int(floor_col) >= 0 && int(floor_col) < color_img.cols) {
            history_d = depth_img.ptr<ushort>(int(floor_row))[int(floor_col)];
            current_d = ushort(p.z * cam_factor);
            if (history_d == 0 || current_d < history_d) {
                depth_img.ptr<ushort>(int(floor_row))[int(floor_col)] = current_d;
                color_img.at<Vec3b>(int(floor_row), int(floor_col)) = Vec3b(p.b, p.g, p.r);
            }
        }
//        if (int(floor_row) >= 0 && int(floor_row) < color_img.rows && int(ceil_col) >= 0 && int(ceil_col) < color_img.cols) {
//            history_d = depth_img.ptr<ushort>(int(floor_row))[int(ceil_col)];
//            current_d = ushort(p.z * cam_factor);
//            if (history_d == 0 || current_d < history_d) {
//                depth_img.ptr<ushort>(int(floor_row))[int(ceil_col)] = current_d;
//                color_img.at<Vec3b>(int(floor_row), int(ceil_col)) = Vec3b(p.b, p.g, p.r);
//            }
//        }
//        if (int(ceil_row) >= 0 && int(ceil_row) < color_img.rows && int(floor_col) >= 0 && int(floor_col) < color_img.cols) {
//            history_d = depth_img.ptr<ushort>(int(ceil_row))[int(floor_col)];
//            current_d = ushort(p.z * cam_factor);
//            if (history_d == 0 || current_d < history_d) {
//                depth_img.ptr<ushort>(int(ceil_row))[int(floor_col)] = current_d;
//                color_img.at<Vec3b>(int(ceil_row), int(floor_col)) = Vec3b(p.b, p.g, p.r);
//            }
//        }
//        if (int(ceil_row) >= 0 && int(ceil_row) < color_img.rows && int(ceil_col) >= 0 && int(ceil_col) < color_img.cols) {
//            history_d = depth_img.ptr<ushort>(int(ceil_row))[int(ceil_col)];
//            current_d = ushort(p.z * cam_factor);
//            if (history_d == 0 || current_d < history_d) {
//                depth_img.ptr<ushort>(int(ceil_row))[int(ceil_col)] = current_d;
//                color_img.at<Vec3b>(int(ceil_row), int(ceil_col)) = Vec3b(p.b, p.g, p.r);
//            }
//        }
	}
    return color_img;
}

Mat draw_cloud_image(const PointCloud<PointXYZRGB>::Ptr& cloud, const vector<Point2> origin_points, vector<Point2>& view_source_points, vector<Point2>& view_target_points) {
    Mat color_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_8UC3);
	Mat depth_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_16UC1);

	float row, col, floor_row, floor_col, ceil_row, ceil_col;
	ushort history_d, current_d;
	for (int i = 0 ; i < cloud->points.size() ; ++i) {
	    PointXYZRGB p = cloud->points[i];
        p.z = -p.z;
        p.y = -p.y;
        row = p.y * fy / p.z + color_img.rows / 2;
        col = p.x * fx / p.z + color_img.cols / 2;
        floor_row = floor(row);
        floor_col = floor(col);

        if (int(floor_row) >= 0 && int(floor_row) < color_img.rows && int(floor_col) >= 0 && int(floor_col) < color_img.cols) {
            history_d = depth_img.ptr<ushort>(int(floor_row))[int(floor_col)];
            current_d = ushort(p.z * cam_factor);
            if (history_d == 0 || current_d < history_d) {
                depth_img.ptr<ushort>(int(floor_row))[int(floor_col)] = current_d;
                color_img.at<Vec3b>(int(floor_row), int(floor_col)) = Vec3b(p.b, p.g, p.r);
            }
        }
	}
	assert(cloud->points.size() == origin_points.size());
	int mod_x = int(img_width * extern_rate / 100.0);
    int mod_y = int(img_height * extern_rate / 50);
	for (int i = 0 ; i < cloud->points.size() ; ++i) {
	    PointXYZRGB p = cloud->points[i];
	    p.z = -p.z;
        p.y = -p.y;
        row = p.y * fy / p.z + color_img.rows / 2;
        col = p.x * fx / p.z + color_img.cols / 2;

        floor_row = floor(row);
        floor_col = floor(col);
        if (int(floor_row) % mod_y != 0 || int(floor_col) % mod_x != 0) continue;
        history_d = depth_img.ptr<ushort>(int(floor_row))[int(floor_col)];
        current_d = ushort(p.z * cam_factor);
        if (current_d <= history_d) {
            view_source_points.emplace_back(origin_points[i]);
            view_target_points.emplace_back(col, row);
        }
	}
    return color_img;
}

Mat draw_cloud_image(const PointCloud<PointXYZRGB>::Ptr& cloud, const vector<Point2> origin_points, vector<Point2>& view_source_points, vector<Point2>& view_target_points, const int& start_col, const int& end_col) {
    Mat color_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_8UC3);
	Mat depth_img = Mat::zeros(img_height * extern_rate, img_width * extern_rate, CV_16UC1);
	float row, col, floor_row, floor_col, ceil_row, ceil_col;
	ushort history_d, current_d;
	for (int i = 0 ; i < cloud->points.size() ; ++i) {
	    PointXYZRGB p = cloud->points[i];
        p.z = -p.z;
        p.y = -p.y;
        row = p.y * fy / p.z + color_img.rows / 2;
        col = p.x * fx / p.z + color_img.cols / 2;
        floor_row = floor(row);
        floor_col = floor(col);

        if (int(floor_row) >= 0 && int(floor_row) < color_img.rows && int(floor_col) >= 0 && int(floor_col) < color_img.cols && int(floor_col) >= start_col && int(floor_col) <= end_col) {
            history_d = depth_img.ptr<ushort>(int(floor_row))[int(floor_col)];
            current_d = ushort(p.z * cam_factor);
            if (history_d == 0 || current_d < history_d) {
                depth_img.ptr<ushort>(int(floor_row))[int(floor_col)] = current_d;
                color_img.at<Vec3b>(int(floor_row), int(floor_col)) = Vec3b(p.b, p.g, p.r);
            }
        }
	}
	assert(cloud->points.size() == origin_points.size());
	int mod_x = int(img_width * extern_rate / 100.0);
    int mod_y = int(img_height * extern_rate / 50);
	for (int i = 0 ; i < cloud->points.size() ; ++i) {
	    PointXYZRGB p = cloud->points[i];
	    p.z = -p.z;
        p.y = -p.y;
        row = p.y * fy / p.z + color_img.rows / 2;
        col = p.x * fx / p.z + color_img.cols / 2;

        floor_row = floor(row);
        floor_col = floor(col);
        if (int(floor_row) % mod_y != 0 || int(floor_col) % mod_x != 0) continue;
        if (int(floor_row) >= 0 && int(floor_row) < color_img.rows && int(floor_col) >= 0 && int(floor_col) < color_img.cols && int(floor_col) >= start_col && int(floor_col) <= end_col) {
            history_d = depth_img.ptr<ushort>(int(floor_row))[int(floor_col)];
            current_d = ushort(p.z * cam_factor);
            if (current_d <= history_d) {
                view_source_points.emplace_back(origin_points[i]);
                view_target_points.emplace_back(col, row);
            }
        }
	}
    return color_img;
}

void calculate_according_right_edge(const Mat& source_coordinate_img, const Mat& target_coordinate_img, int& source_col, int& target_col) {
    float left2right_offset = -0.12001928710937500000;
    float source_right_col = max_size, target_right_col = max_size;
    float source_x, source_z, target_x, target_z;
    float col;
    for (int row = 0 ; row < source_coordinate_img.rows ; ++row) {
        source_x = source_coordinate_img.ptr<float>(row)[source_col * 3];
        source_z = source_coordinate_img.ptr<float>(row)[source_col * 3 + 2];
        col = (source_x + left2right_offset) * fx / -source_z + img_width * extern_rate / 2;
        if (col < source_right_col && source_x != empty_num) {
            source_right_col = col;
        }

        target_x = target_coordinate_img.ptr<float>(row)[target_col * 3];
        target_z = target_coordinate_img.ptr<float>(row)[target_col * 3 + 2];
        col = (target_x + left2right_offset) * fx / -target_z + img_width * extern_rate / 2;
        if (col < target_right_col && target_x != empty_num) {
            target_right_col = col;
        }
    }
    source_col = source_right_col;
    target_col = target_right_col;
}

double cal_std(vector<float> resultSet) {
	double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
	double mean =  sum / resultSet.size(); //均值

	double accum  = 0.0;
	std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d) {
		accum  += (d-mean)*(d-mean);
	});

	double stdev = sqrt(accum/(resultSet.size()-1));
	return stdev;
}

float calculate_interval_score(const Mat& source_coordinate_img, const Mat& target_coordinate_img, int source_start_col, int target_end_col, float x_offset, float z_offset) {
    float source_end_col = max_size, target_start_col = -max_size, col;
    float max_outer_col = -max_size, min_outer_col = max_size, max_inner_col = max_size, min_inner_col = -max_size;
    for (int row = 0 ; row < source_coordinate_img.rows ; ++row) {
        float source_x = source_coordinate_img.ptr<float>(row)[source_start_col * 3];
        float source_z = source_coordinate_img.ptr<float>(row)[source_start_col * 3 + 2];
        col = (source_x + x_offset) * fx / -(source_z + z_offset) + source_coordinate_img.cols / 2;
        if (col > target_start_col && source_x != empty_num) target_start_col = col;
        col = (source_x + x_offset / 2) * fx / -(source_z + z_offset / 2);
        if (col < min_outer_col && source_x != empty_num) min_outer_col = col;
        if (col > min_inner_col && source_x != empty_num) min_inner_col = col;

        float target_x = target_coordinate_img.ptr<float>(row)[target_end_col * 3];
        float target_z = target_coordinate_img.ptr<float>(row)[target_end_col * 3 + 2];
        col = (target_x - x_offset) * fx / -(target_z - z_offset) + target_coordinate_img.cols / 2;
        if (col < source_end_col && target_x != empty_num) source_end_col = col;
        col = (target_x - x_offset / 2) * fx / -(target_z - z_offset / 2);
        if (col > max_outer_col && target_x != empty_num) max_outer_col = col;
        if (col < max_inner_col && target_x != empty_num) max_inner_col = col;

    }
    float outer_width = max_outer_col - min_outer_col;
    float inner_width = max_inner_col - min_inner_col;
    float interval_rate = outer_width / inner_width;

    max_distance[source_start_col][target_end_col] = outer_width;
    min_distance[source_start_col][target_end_col] = inner_width;

    vector<float> source_zs;
    for (int col = source_start_col ; col < source_end_col ; ++col) {
        for (int row = 0 ; row < source_coordinate_img.rows ; ++row) {
            if (source_coordinate_img.ptr<float>(row)[col * 3 + 2] != empty_num) {
                source_zs.push_back(source_coordinate_img.ptr<float>(row)[col * 3 + 2]);
            }
        }
    }

    vector<float> target_zs;
    for (int col = target_start_col ; col < target_end_col ; ++col) {
        for (int row = 0 ; row < target_coordinate_img.rows ; ++row) {
            if (target_coordinate_img.ptr<float>(row)[col * 3 + 2] != empty_num) {
                target_zs.push_back(target_coordinate_img.ptr<float>(row)[col * 3 + 2]);
            }
        }
    }
    double target_z_std = cal_std(target_zs);
    double source_z_std = cal_std(source_zs);
    double z_std = source_z_std + target_z_std;

    calculate_according_right_edge(source_coordinate_img, target_coordinate_img, source_start_col, target_end_col);
    if (source_start_col < 0 || source_start_col >= img_width * extern_rate|| target_end_col < 0 || target_end_col >= img_width * extern_rate) return max_size;
    if (outer_width < 0 || inner_width < 0) return max_size;
    return 2 * interval_rate + outer_width / img_width + z_std;
}

float findBestInterval(const Mat& source_coordinate_img, const Mat& target_coordinate_img, int& source_col, int& target_col, float x_offset, float z_offset, float source_max_x, float target_min_x) {
    if (source_col >= img_width * 2 || target_col < 0) {
        std::cout << "unpossible!" << std::endl;
        return max_size;
    }
    if (record[source_col][target_col] >= 0) {
        int move_source_col = source_cols[source_col][target_col];
        int move_target_col = target_cols[source_col][target_col];
        source_col = move_source_col;
        target_col = move_target_col;
        return record[source_col][target_col];
    }
    for (int row = 0 ; row < source_coordinate_img.rows; ++row) {
        float source_x = source_coordinate_img.ptr<float>(row)[source_col * 3];
        float source_z = source_coordinate_img.ptr<float>(row)[source_col * 3 + 2];

        float target_x = target_coordinate_img.ptr<float>(row)[target_col * 3];
        float target_z = target_coordinate_img.ptr<float>(row)[target_col * 3 + 2];
        if (source_x > source_max_x && source_x != empty_num) {
            source_max_x = source_x;
        }
        if (target_x < target_min_x && target_x != empty_num) {
            target_min_x = target_x;
        }
    }
    if (source_max_x + x_offset >= target_min_x) {
        record[source_col][target_col] = calculate_interval_score(source_coordinate_img, target_coordinate_img, source_col, target_col, x_offset, z_offset);
        source_cols[source_col][target_col] = source_col;
        target_cols[source_col][target_col] = target_col;
        return record[source_col][target_col];
    }
    int copy_source_col = source_col;
    int copy_target_col = target_col;
    int move_source_col = copy_source_col + 1;
    int move_target_col = copy_target_col - 1;

    float move_source_result = findBestInterval(source_coordinate_img, target_coordinate_img, move_source_col, copy_target_col, x_offset, z_offset, source_max_x, target_min_x);
    float move_target_result = findBestInterval(source_coordinate_img, target_coordinate_img, copy_source_col, move_target_col, x_offset, z_offset, source_max_x, target_min_x);
    if (move_target_result < move_source_result) {
        record[source_col][target_col] = move_target_result;
        source_cols[source_col][target_col] = copy_source_col;
        target_cols[source_col][target_col] = move_target_col;
        source_col = copy_source_col;
        target_col = move_target_col;
        return move_target_result;
    } else {
        record[source_col][target_col] = move_source_result;
        source_cols[source_col][target_col] = move_source_col;
        target_cols[source_col][target_col] = copy_target_col;
        source_col = move_source_col;
        target_col = copy_target_col;
        return move_source_result;
    }
}


void calculate_cloud_overlap_edge(const Mat& source_coordinate_img, const Mat& target_coordinate_img, float& min_col, float& max_col, float x_offset, float z_offset) {
    min_col = max_size;
    max_col = 0;
    float source_col, target_col;
    for (int col = 0 ; col < source_coordinate_img.cols ; ++col)
        for (int row = 0 ; row < source_coordinate_img.rows ; ++row) {
            float source_x = source_coordinate_img.ptr<float>(row)[col * 3];
            float source_z = source_coordinate_img.ptr<float>(row)[col * 3 + 2];
            source_col = (source_x + x_offset) * fx / -(source_z + z_offset) + source_coordinate_img.cols / 2;
            if (source_col > max_col && source_x != empty_num) max_col = source_col;
        }

    for (int col = 0 ; col < target_coordinate_img.cols ; ++col)
        for (int row = 0 ; row < target_coordinate_img.rows ; ++row) {
            float target_x = target_coordinate_img.ptr<float>(row)[col * 3];
            float target_z = target_coordinate_img.ptr<float>(row)[col * 3 + 2];
            target_col = (target_x - x_offset) * fx / -(target_z - z_offset) + target_coordinate_img.cols / 2;
            if (target_col < min_col && target_x != empty_num) min_col = target_col;
        }
}

