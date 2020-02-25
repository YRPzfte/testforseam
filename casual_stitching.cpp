#include <iostream>
#include "./Stitching/NISwGSP_Stitching.h"
#include "./Stitching/Consistent_Stitching.h"
#include "./Debugger/TimeCalculator.h"
#include "./CasualUtil/common.h"
#include "./CommonUtil/function.h"
using namespace std;


int main(int argc, const char * argv[]) {
    string source_dir = argv[1];
    string target_dir = argv[2];
    Eigen::initParallel(); /* remember to turn off "Hardware Multi-Threading */
    Eigen::nbThreads();
    cout << "[拼接左视图:]" << endl;
    TimeCalculator timer;
    MultiImages multi_left_images(source_dir, target_dir, "left", LINES_FILTER_WIDTH, LINES_FILTER_LENGTH);
    timer.start();
    NISwGSP_Stitching niswgsp(multi_left_images);
    niswgsp.setWeightToAlignmentTerm(1);
    niswgsp.setWeightToLocalSimilarityTerm(0.75);
    niswgsp.setWeightToGlobalSimilarityTerm(6, 20, GLOBAL_ROTATION_3D_METHOD);

    Mat stitch_left_img = niswgsp.solve(BLEND_AVERAGE);
//    imwrite("stitching_left.png", stitch_left_img);
    timer.end("[完成左视图的拼接]");


    MultiImages multi_right_images(source_dir, target_dir, "right", LINES_FILTER_WIDTH, LINES_FILTER_LENGTH);
    vector<pair<Point2, Point2> > source_mapping_pairs, target_mapping_pairs;
    vector<pair<Point2, Point2> > source_align_pairs, target_align_pairs;
    get_mapping_pair(multi_left_images.mapping_pairs[0], multi_left_images.mapping_pairs[1],
                      source_mapping_pairs, target_mapping_pairs,
                      source_align_pairs, target_align_pairs,
                      multi_left_images.scale,
                      multi_left_images.depth_images[0], multi_left_images.depth_images[1], multi_left_images.source_depth, multi_left_images.target_depth,
                      multi_left_images.images_data[0].img, multi_left_images.images_data[1].img, multi_right_images.images_data[0].img, multi_right_images.images_data[1].img);

    vector<vector<pair<Point2, Point2> > > boundaryPairs = niswgsp.getBounderPairs();
    Consistent_Stitching consist(multi_right_images);
    consist.setSize(stitch_left_img.cols, stitch_left_img.rows);
    consist.setPaths(niswgsp.getMappingPaths());
    consist.set_mapping_pair(source_mapping_pairs, target_mapping_pairs);
    consist.set_align_pair(source_align_pairs, target_align_pairs);
    consist.set_boundary_pair(boundaryPairs);
    consist.setWeightToAlignmentTerm(1);
    consist.setWeightToDisparityTerm(1);
    consist.setWeightToSimilarityTerm(0.75, GLOBAL_ROTATION_3D_METHOD);

    Mat stitch_right_img = consist.solve(BLEND_AVERAGE);
//    imwrite("stitching_right.png", stitch_right_img);
    timer.end("[完成右视图的拼接]");
    std::cout << "stitching left---  " << "width:" << stitch_left_img.cols << " height:" << stitch_left_img.rows << std::endl;
    std::cout << "stitching right--- " << "width:" << stitch_right_img.cols << " height:" << stitch_right_img.rows << std::endl;
    crop_stitching_result(stitch_left_img, stitch_right_img);
    imwrite("zhang_left.png", stitch_left_img);
    imwrite("zhang_right.png", stitch_right_img);
    timer.end("[完成视图的裁剪]");
    return 0;

}