#ifndef CASAUAL_COMMON_H
#define CASAUAL_COMMON_H
#include "../Feature/ImageData.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>
#include <string>
#include <math.h>
#include <vector>
using namespace cv;
using namespace std;
typedef Point_<float> Point2;
void stitching_disparity_by_poisson_blending(Mat& source_depth, Mat& target_depth, const vector<int>& paths);
void get_mapping_pair(const vector<pair<Point2, Point2> >& source_left_mapping_pairs, const vector<pair<Point2, Point2> >& target_left_mapping_pairs,
                      vector<pair<Point2, Point2> >& source_mapping_pairs, vector<pair<Point2, Point2> >& target_mapping_pairs,
                      vector<pair<Point2, Point2> >& source_align_pairs, vector<pair<Point2, Point2> >& target_align_pairs,
                      float scale, const Mat& origin_source_depth, const Mat& origin_target_depth, const Mat& warping_source_depth, const Mat& warping_target_depth,
                      const Mat& source_left_img, const Mat& target_left_img, const Mat& source_right_img, Mat& target_right_img);
#endif