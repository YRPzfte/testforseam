#ifndef FUNCTION_H
#define FUNCTION_H
#include "../Feature/FeatureController.h"
#include "../Feature/ImageData.h"
#include "../Feature/MultiImages.h"
#include "../Util/Statistics.h"
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
Mat stitchingByBestLine(const vector<int>& paths, const Mat& source_img, const Mat& target_img);
Mat draw_line(Mat image);
void draw_white(Mat& image);
void findBestLine(const Mat& source_img, const Mat& target_img, vector<int>& paths, int start_col=0, int end_col=0, int start_row=0, int end_row=0, int last_col=-1);
Mat draw_pair(const Mat& source_img, const Mat& target_img, const vector<pair<Point2, Point2> >& matches);
Mat draw_point(const Mat& image, vector<Point2> points);
void crop_stitching_result(Mat& left_image, Mat& right_image);
vector<pair<int, int> > getInitialFeaturePairs(const vector<FeatureDescriptor>& feature_descriptors_1, const vector<FeatureDescriptor>& feature_descriptors_2);
Mat blending_images(const Mat& source_img, const Mat& target_img, int overlap_width, int source_col, int target_col, vector<int>& paths);
#endif