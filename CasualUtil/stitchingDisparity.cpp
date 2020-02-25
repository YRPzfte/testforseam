#include "common.h"

//调整视差值
void stitching_disparity_by_poisson_blending(Mat& source_depth, Mat& target_depth, const vector<int>& paths) {
    vector<int> source_depths, target_depths, dest_depths;
    for (int i = 0 ; i < paths.size() ; ++i) {
        int source_d = source_depth.ptr<ushort>(i)[paths[i]];
        int  target_d = target_depth.ptr<ushort>(i)[paths[i]];
        if (source_d == 0 || target_d == 0) continue;
        if (abs(source_d - target_d) > 100) continue;
        source_depths.push_back(source_d);
        target_depths.push_back(target_d);
        dest_depths.push_back((source_d + target_d) / 2);
    }
    double source_offset_sum = 0, target_offset_sum = 0;
    for (int i = 0 ; i < dest_depths.size() ; ++i) {
        source_offset_sum += (dest_depths[i] - source_depths[i]);
        target_offset_sum += (dest_depths[i] - target_depths[i]);
    }
    int source_offset = source_offset_sum / dest_depths.size();
    int target_offset = target_offset_sum / dest_depths.size();
    for (int row = 0 ; row < source_depth.rows ; ++ row) {
        for (int col = 0 ; col < target_depth.cols ; ++ col) {
            if (source_depth.ptr<ushort>(row)[col] != 0) {
                source_depth.ptr<ushort>(row)[col] += source_offset;
            }
            if (target_depth.ptr<ushort>(row)[col] != 0) {
                target_depth.ptr<ushort>(row)[col] += target_offset;
            }
        }
    }
}