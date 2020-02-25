#include "function.h"

Mat draw_pair(const Mat& source_img, const Mat& target_img, const vector<pair<Point2, Point2> >& matches) {
    Mat draw_img = Mat::zeros(source_img.rows, source_img.cols * 2, CV_8UC3);

    Mat source_imageROI = draw_img(Rect(0, 0, source_img.cols, source_img.rows));
    source_img.copyTo(source_imageROI);
    Mat target_imageROI = draw_img(Rect(source_img.cols, 0, target_img.cols, target_img.rows));
    target_img.copyTo(target_imageROI);

    for (int i = 0 ; i < matches.size() ; ++i) {
        Point2 source_p = matches[i].first;
        Point2 target_p = matches[i].second;
        circle(draw_img, Point(source_p.x, source_p.y), 2, Scalar(255, 0, 0));
        circle(draw_img, Point(target_p.x + source_img.cols, target_p.y), 2, Scalar(255, 0, 0));
        line(draw_img, Point(source_p.x, source_p.y), Point(target_p.x + source_img.cols, target_p.y), Scalar(0,255,0));
    }
    return draw_img;
}

Mat draw_line(Mat image) {
    for (int row = 0 ; row < image.rows ; ++row) {
        for (int col = 0 ; col < image.cols ; col += 40) {
            image.ptr<uchar>(row)[col * 3] = 0;
            image.ptr<uchar>(row)[col * 3 + 1] = 255;
            image.ptr<uchar>(row)[col * 3 + 2] = 0;

            image.ptr<uchar>(row)[(col + 1) * 3] = 0;
            image.ptr<uchar>(row)[(col + 1) * 3 + 1] = 255;
            image.ptr<uchar>(row)[(col + 1) * 3 + 2] = 0;

            image.ptr<uchar>(row)[(col + 2) * 3] = 0;
            image.ptr<uchar>(row)[(col + 2) * 3 + 1] = 255;
            image.ptr<uchar>(row)[(col + 2) * 3 + 2] = 0;
        }
    }

    for (int row = 0 ; row < image.rows ; row += 40) {
        for (int col = 0 ; col < image.cols ; ++col) {
            image.ptr<uchar>(row)[col * 3] = 0;
            image.ptr<uchar>(row)[col * 3 + 1] = 255;
            image.ptr<uchar>(row)[col * 3 + 2] = 0;

            image.ptr<uchar>(row + 1)[col * 3] = 0;
            image.ptr<uchar>(row + 1)[col * 3 + 1] = 255;
            image.ptr<uchar>(row + 1)[col * 3 + 2] = 0;

            image.ptr<uchar>(row + 2)[col * 3] = 0;
            image.ptr<uchar>(row + 2)[col * 3 + 1] = 255;
            image.ptr<uchar>(row + 2)[col * 3 + 2] = 0;
        }
    }
    return image;
}

void draw_white(Mat& image) {
    for (int row = 0 ; row < image.rows ; ++row) {
        for (int col = 0 ; col < image.cols ; ++col) {
            image.at<Vec3b>(int(row), int(col)) = Vec3b(255, 255, 255);
        }
    }
}
Mat draw_point(const Mat& image, vector<Point2> points) {
    Mat draw_img = image.clone();
    for (int i = 0 ; i < points.size() ; ++i) {
        Point2 p = points[i];
        circle(draw_img, Point(p.x, p.y), 2, Scalar(255, 0, 0));
    }
    return draw_img;
}

void cal_crop_rect(const Mat& image, int& col_start, int& col_end, int& row_start, int& row_end) {
    uchar b, g, r;
    col_start = 0;
    for (int col = col_start ; col < image.cols ; ++col) {
        float black_rate = 0;
        for (int row = 0 ; row < image.rows ; ++row) {
            b = image.ptr<uchar>(row)[col * 3];
			g = image.ptr<uchar>(row)[col * 3 + 1];
			r = image.ptr<uchar>(row)[col * 3 + 2];
			if (b == 0 && g == 0 && r == 0) {
			    black_rate += 1;
			}
        }
        black_rate /= image.rows;
        if (black_rate < 0.9) {
            col_start = col;
            break;
        }
    }
    col_end = image.cols;
    for (int col =  col_end - 1; col >= 0 ; --col) {
        float black_rate = 0;
        for (int row = 0 ; row < image.rows ; ++row) {
            b = image.ptr<uchar>(row)[col * 3];
			g = image.ptr<uchar>(row)[col * 3 + 1];
			r = image.ptr<uchar>(row)[col * 3 + 2];
			if (b == 0 && g == 0 && r == 0) {
			    black_rate += 1;
			}
        }
        black_rate /= image.rows;
        if (black_rate < 0.9) {
            col_end = col;
            break;
        }
    }
    row_start = 0;
    for (int row = row_start ; row < image.rows ; ++row) {
        float black_rate = 0;
        for (int col = 0 ; col < image.cols ; ++col) {
            b = image.ptr<uchar>(row)[col * 3];
			g = image.ptr<uchar>(row)[col * 3 + 1];
			r = image.ptr<uchar>(row)[col * 3 + 2];
			if (b == 0 && g == 0 && r == 0) {
			    black_rate += 1;
			}
		}
        black_rate /= image.cols;
        if (black_rate < 0.9) {
            row_start = row;
            break;
        }
    }
    row_end = image.rows;
    for (int row = row_end - 1 ; row >= 0 ; --row) {
        float black_rate = 0;
        for (int col = 0 ; col < image.cols ; ++col) {
            b = image.ptr<uchar>(row)[col * 3];
			g = image.ptr<uchar>(row)[col * 3 + 1];
			r = image.ptr<uchar>(row)[col * 3 + 2];
			if (b == 0 && g == 0 && r == 0) {
			    black_rate += 1;
			}
		}
        black_rate /= image.cols;
        if (black_rate < 0.9) {
            row_end = row;
            break;
        }
    }
}
void crop_stitching_result(Mat& left_image, Mat& right_image) {
    assert(left_image.cols == right_image.cols && left_image.rows == right_image.rows);
    int left_col_start, left_col_end, left_row_start, left_row_end;
    cal_crop_rect(left_image, left_col_start, left_col_end, left_row_start, left_row_end);
    int right_col_start, right_col_end, right_row_start, right_row_end;
    cal_crop_rect(right_image, right_col_start, right_col_end, right_row_start, right_row_end);
    int col_start = max(left_col_start, right_col_start);
    int col_end = min(left_col_end, right_col_end);
    int row_start = max(left_row_start, right_row_start);
    int row_end = min(left_row_end, right_row_end);
    std::cout << left_col_start << " " << left_col_end << " " << left_row_start << " " << left_row_end << std::endl;
    std::cout << right_col_start << " " << right_col_end << " " << right_row_start << " " << right_row_end << std::endl;
    left_image = left_image(Rect(col_start, row_start, col_end - col_start, row_end - row_start));
    right_image = right_image(Rect(col_start, row_start, col_end - col_start, row_end - row_start));
}